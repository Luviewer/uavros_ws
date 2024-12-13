#!/usr/bin/env python3
# coding: utf-8

import time
import rospy
import nav_msgs.msg as nav_msgs
from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Quaternion, Point, Vector3, TransformStamped, TwistWithCovarianceStamped
#
from jethexa_controller_interfaces import msg as jetmsg
from jethexa_controller_interfaces.srv import SetPose1, SetPose1Request, SetPose1Response
from jethexa_controller_interfaces.srv import SetPose2, SetPose2Request, SetPose2Response
from jethexa_controller_interfaces.srv import PoseTransform, PoseTransformRequest, PoseTransformResponse
#
import jethexa_sdk.buzzer as buzzer
from jethexa_controller import jethexa, build_in_pose, config
from jethexa_controller.z_voltage_publisher import VoltagePublisher
from jethexa_controller.z_joint_states_publisher import JointStatesPublisher
import geometry_msgs.msg


class jethexaControlNode:
    def __init__(self, node_name):
        rospy.init_node(node_name, anonymous=True)

        self.tf_prefix = rospy.get_param('~tf_prefix', '')
        self.tf_prefix = (self.tf_prefix + '/') if self.tf_prefix != '' else '' 

        self.controller = jethexa.JetHexa(self)

        # 机器人状态发布
        self.voltage_publisher = VoltagePublisher(node=self, rate=1)  # 母线电压发布
        self.joint_states_publisher = JointStatesPublisher(node=self, rate=20)  # 关节角度发布

        self.controller.set_build_in_pose('DEFAULT_POSE', 2)

        # 机器人姿态设置服务
        self.set_pose1_srv = rospy.Service("/jethexa_controller/set_pose_1", SetPose1, self.set_pose1_cb) # 通过姿态名称设置机器人的姿态， "DEFAULT_POSE" "DEFAULT_POSE_M"
        self.set_pose2_srv = rospy.Service("/jethexa_controller/set_pose_2", SetPose2, self.set_pose2_cb) # 通过六条腿的末端坐标设置机器人的姿态

        # 机器人机体姿态变换
        # 通过 平移和欧拉角旋转变换机器人的姿态， 相对姿态变换, 旋转顺序为 RPY
        self.set_transform2_sub = rospy.Subscriber("/jethexa_controller/pose_transform_euler", jetmsg.TransformEuler, self.pose_transform_euler_cb)
        # 通过 平移和欧拉角旋转变换机器人的姿态， 绝对变换, 旋转顺序为 RPY
        self.set_pose_euler_sub = rospy.Subscriber("/jethexa_controller/set_pose_euler", jetmsg.Pose, self.set_pose_euler_cb)
        # 设置一条腿末端移动到指定位置, 绝对坐标
        self.set_leg_position_sub = rospy.Subscriber("/jethexa_controller/set_leg_absolute", jetmsg.LegPosition, self.set_leg_absolute_cb)
        # 设置一条腿末端移动当相对与当期位置的指定位置
        self.set_leg_position_re_sub = rospy.Subscriber("/jethexa_controller/set_leg_relatively", jetmsg.LegPosition, self.set_leg_relatively_cb)

        # 机器人头部姿态控制
        # 绝对旋转，RPY=0, 0, 0时 云台水平正视前方
        self.set_head_absolute = rospy.Subscriber("/jethexa_cotnroller/set_head_absolute", jetmsg.TransformEuler, self.head_absolute_cb)
        # 相对旋转
        self.set_head_relatively = rospy.Subscriber("/jethexa_controller/set_head_relatively", jetmsg.TransformEuler, self.head_relatively_cb)

        # 机器人移动服务
        # 通过步态参数控制机器人的移动
        self.traveling = rospy.Subscriber("/jethexa_controller/traveling", jetmsg.Traveling, self.set_traveling_cb)
        # 通过线速度、角速度控制机器人的移动，其他参数由上一次执行的 gait大于0的traveling来指定
        self.cmd_vel_sub = rospy.Subscriber("/jethexa_controller/cmd_vel", geometry_msgs.msg.Twist, self.controller.cmd_vel)

        # 机器人动作组运行服务
        self.run_action_sub = rospy.Subscriber("/jethexa_controller/run_actionset", jetmsg.RunActionSet, self.run_action_set_sub_cb)

        # 订阅融合imu、激光雷达后的精确odom, 以获取准确的偏航角提高里程计精度
        self.odom_sub = rospy.Subscriber("odom/filtered", nav_msgs.Odometry, self.odom_callback)

        # 定时发布odom
        odom_enable = rospy.get_param('~odom_enable', False)
        if odom_enable:
            # odom 相关数据发布
            self.last_time_odometry = time.time()
            # 发布数据给 python2 发布tf转换
            self.odom_trans_pub = rospy.Publisher("~middle_tf", TransformStamped, queue_size=2) 
            # 步态原始里程计
            self.odometry_pub = rospy.Publisher("odom/raw", nav_msgs.Odometry, queue_size=2) 
            # 步态原始速度
            self.twist_pub = rospy.Publisher("twist_raw", TwistWithCovarianceStamped, queue_size=2) 
            self.odom_timer = rospy.Timer(rospy.Duration(0.02), self.odometry_publish)

        # 完毕
        buzzer.on()
        time.sleep(0.1)
        buzzer.off()
    
    def odom_callback(self, msg: nav_msgs.Odometry):
        o = msg.pose.pose.orientation
        r = R.from_quat((o.x, o.y, o.z, o.w))
        yaw = r.as_euler('xyz', degrees=False)
        self.controller.real_pose_yaw = yaw[-1]


    def head_absolute_cb(self, msg: jetmsg.TransformEuler):
        yaw = msg.rotation.z
        pitch = msg.rotation.y
        rospy.loginfo("Set head absolutly pitch:{} yaw:{}".format(pitch, yaw))
        try:
            self.controller.set_joint(19, yaw, msg.duration)
            self.controller.set_joint(20, pitch, msg.duration)
        except Exception as e:
            rospy.logerr(str(e))

    def head_relatively_cb(self, msg: jetmsg.TransformEuler):
        yaw = msg.rotation.z
        pitch = msg.rotation.y
        old_yaw = self.controller.joints_state['head_pan_joint']  # 当前的关节角度
        old_pitch = self.controller.joints_state['head_tilt_joint']
        new_yaw = old_yaw + yaw
        new_pitch = old_pitch + pitch
        if new_pitch < -0.35:
            return
        try:
            self.controller.set_joint(19, new_yaw, msg.duration)
            self.controller.set_joint(20, new_pitch, msg.duration)
        except Exception as e:
            rospy.logerr(str(e))

    def set_traveling_cb(self, msg: jetmsg.Traveling):
        gait = msg.gait
        height = msg.height
        stride = msg.stride
        direction = msg.direction
        rotation = msg.rotation
        steps = msg.steps
        time_ = msg.time
        interrupt = msg.interrupt
        relative_height = msg.relative_height
        try:
            if gait > 0:
                self.controller.set_step_mode(
                    gait,
                    stride,
                    height,
                    direction,
                    rotation,
                    time_,
                    steps,
                    interrupt=interrupt,
                    relative_height=relative_height)
            else:
                if gait == 0:
                    self.controller.stop_running(timeout=None, callback=lambda:self.controller.set_pose(None, None, time_))
                elif gait == -1:
                    self.controller.stop_running()
                elif gait == -2:
                    self.controller.set_build_in_pose('DEFAULT_POSE', time_)
                else:
                    pass
        except Exception as e:
            rospy.logerr(str(e))

    def set_leg_absolute_cb(self, leg_pos: jetmsg.LegPosition):
        leg_id = leg_pos.leg_id
        leg_pos = leg_pos.position.x, leg_pos.position.y, leg_pos.position.z
        duration = leg_pos.duration
        self.controller.set_leg_position(leg_id, leg_pos, duration)


    def set_leg_relatively_cb(self, leg_pos: jetmsg.LegPosition):
        leg_id = leg_pos.leg_id
        duration = leg_pos.duration
        leg_pos = leg_pos.position.x, leg_pos.position.y, leg_pos.position.z
        cur_pos = list(self.controller.pose[leg_id - 1])
        new_pos = cur_pos[0] + leg_pos[0], cur_pos[1] + leg_pos[1], cur_pos[2] + leg_pos[2]
        self.controller.set_leg_position(leg_id, new_pos, duration)


    def odometry_publish(self, event):
        cur_time = rospy.Time.now()
        cur_quat = R.from_euler('xyz', [-self.controller.transform[1][0], -self.controller.transform[1][1], self.controller.pose_yaw], False).as_quat()
        cur_position = self.controller.position

        # odom transform message begin
        # odom 到 base_link 的变换
        odom_trans = TransformStamped()
        odom_trans.header.stamp = cur_time
        odom_trans.header.frame_id = "".join([self.tf_prefix, "odom"])
        odom_trans.child_frame_id =  "".join([self.tf_prefix, "base_link"])

        # 平移, 旋转
        translation, rotation = Vector3(), Quaternion()
        translation.x, translation.y, translation.z = cur_position
        rotation.x, rotation.y, rotation.z, rotation.w = cur_quat
        odom_trans.transform.translation, odom_trans.transform.rotation = translation, rotation
        self.odom_trans_pub.publish(odom_trans)
        # odom transform message end
        #
        odom = nav_msgs.Odometry()
        odom.header.stamp = cur_time
        odom.header.frame_id = "".join([self.tf_prefix, "odom"])
        odom.child_frame_id =  "".join([self.tf_prefix, "base_link"])
        # set the positions
        position, orientation = Point(), Quaternion()
        position.x, position.y, position.z = cur_position
        orientation.x, orientation.y, orientation.z, orientation.w = cur_quat
        odom.pose.pose.position, odom.pose.pose.orientation = position, orientation
        #
        odom.pose.covariance[0] = 0.001
        odom.pose.covariance[7] = 0.001
        odom.pose.covariance[14] = 1000000.0
        odom.pose.covariance[21] = 1000000.0
        odom.pose.covariance[28] = 1000000.0
        odom.pose.covariance[35] = 1000.0

        # set the velocity
        odom.twist.twist.linear.x, odom.twist.twist.linear.y = self.controller.linear_x, self.controller.linear_y
        odom.twist.twist.angular.z = self.controller.angular_z
        odom.twist.covariance = odom.pose.covariance
        #
        twist = TwistWithCovarianceStamped()
        twist.header.frame_id = "odom"
        twist.header.stamp = cur_time
        twist.twist.twist.linear.x, twist.twist.twist.linear.y = (self.controller.linear_x, self.controller.linear_y)
        twist.twist.twist.angular.z = self.controller.angular_z
        twist.twist.covariance = odom.pose.covariance
        self.odometry_pub.publish(odom)
        self.twist_pub.publish(twist)
        self.last_time_odometry = cur_time

    #
    def set_pose1_cb(self, req: SetPose1Request):
        rospy.loginfo("set_pose_1 called")
        rsp = SetPose1Response()
        try:
            pose = getattr(build_in_pose, req.pose)
            self.controller.set_pose(pose, req.duration, interrupt=req.interrupt)
        except Exception as e:
            rospy.logerr(str(e))
            rsp.result = -1
            rsp.msg = str(e)
        finally:
            return rsp

    def set_pose2_cb(self, req: SetPose2Request):
        rospy.loginfo("set_pose_2 called")
        rsp = SetPose2Response()
        try:
            pose = [(point.x, point.y, point.z) for point in req.pose]
            self.controller.set_pose(pose, req.duration, interrupt=req.interrupt)
        except Exception as e:
            rospy.logerr(str(e))
            rsp.result = -1
            rsp.msg = str(e)
        return rsp
    
    def set_pose_euler_cb(self, msg: jetmsg.Pose):
        self.controller.transform_absolutely(
            (msg.position.x, msg.position.y, msg.position.z), 
            (msg.orientation.roll, msg.orientation.pitch, msg.orientation.yaw), 0.4)

    def pose_transform_1_cb(self, msg: PoseTransformRequest):
        translation = msg.translation
        rotation = msg.rotation
        duration = msg.duration
        try:
            self.controller.transform_pose(translation, rotation, duration)
        except Exception as e:
            rospy.logerr(str(e))

    def pose_transform_euler_cb(self, msg: jetmsg.TransformEuler):
        translation = msg.translation.x, msg.translation.y, msg.translation.z
        rotation = msg.rotation.x, msg.rotation.y, msg.rotation.z
        duration = msg.duration

        try:
            self.controller.transform_pose_2(translation, "xyz", rotation, duration, degrees=False)
        except Exception as e:
            rospy.logerr(str(e))

    def run_action_set_sub_cb(self, msg: jetmsg.RunActionSet):
        rospy.loginfo("{}, {}".format(msg.action_path, msg.repeat))
        file_path = '/home/hiwonder/ActionSets/' + msg.action_path if msg.default_path else msg.action_path
        self.controller.run_action_set(file_path, msg.repeat)


def main():
    jethexa_controller_node = jethexaControlNode('jethexa_control')
    try:
        rospy.spin()
    except Exception as e:
        rospy.logerr(str(e))

if __name__ == '__main__':
    main()
