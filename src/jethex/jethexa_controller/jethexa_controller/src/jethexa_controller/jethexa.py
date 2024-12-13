# jethexa.py 完成机器人的运动控制的底层逻辑

import sys
import os
import time
import rospy
import kinematics
import math
import threading
import itertools
from scipy.spatial.transform import Rotation as R
from jethexa_controller import config, x_joint_control, kinematics_api, build_in_pose
from .run_actionset import do_action, actionset_runner
from jethexa_controller_interfaces.msg import Pose
from .moving_controller import MovingGenerator, MovingParams, CmdVelGenerator, CmdVelParams
from .pose_transformer import PoseTransformer, PoseTransformerParams
import geometry_msgs.msg

X1 = 93.60
Y1 = 50.805
X2 = 0.0
Y2 = 73.535


class JetHexa:
    TRIPOD_GAIT = 1
    RIPPLE_GAIT = 2
    
    def __init__(self, node, pwm=True):
        self.node = node
        self.joints_state = {}

        for value in config.SERVOS.values():
            self.joints_state[value['name']] = 0.0

        self.lock = threading.RLock()

        self.cur_moving_generator = None
        self.new_moving_generator = None
        self.cur_pose_transformer = None
        self.new_pose_transformer = None
        self.cur_actionset_runner = None
        self.new_actionset_runner = None
        self.cur_pose_setter = None
        self.new_pose_setter = None

        self.pose = build_in_pose.DEFAULT_POSE
        self.org_transform = ((0, 0, 120), (0, 0, 0))
        self.transform = ((0, 0, 120), (0, 0, 0)) #xyz 平移 mm, xyz 欧拉角 rad

        self.pose_yaw = 0 # 原始积分偏航角
        self.real_pose_yaw = None  # 多传感器融合后偏航角
        self.position = (0, 0, 0)
        self.angular_z = 0 # z轴上的角速度
        self.voltage = 0.0
        self.linear_x, self.linear_y, self.linear_z = 0, 0, 0 # 三轴上的线性速度 
        self.stopped = threading.Event()  # 正在运行的动作已经被停止标志
        self.stopping = False  # 停止当前正在运行的任务标志

        self.cmd_gait = 1
        self.cmd_height = 40
        self.cmd_period = 2.0

        self.voltage_timer = time.time()
        self.loop_thread = threading.Thread(target=self.loop, daemon=True)
        self.loop_enable = True
        self.loop_thread.start()


    def reset_all_new_gen(self):
        self.new_pose_setter = None
        self.new_actionset_runner = None
        self.new_pose_transformer = None
        self.new_moving_generator = None

    def reset_all_cur_gen(self):
        self.cur_pose_setter = None
        self.cur_actionset_runner = None
        self.cur_pose_transformer = None
        self.cur_moving_generator = None

    def loop(self):
        """
        实际执行具体操作的线程循环
        """
        t1 = time.perf_counter() + 0.02
        t3 = time.perf_counter()
        os.system("sudo renice -n -19 -p " + str(os.getpid()))
        while self.loop_enable:
            # 动作组运行
            try:
                if self.new_actionset_runner is not None:
                    self.cur_actionset_runner = self.new_actionset_runner
                    self.new_actionset_runner = None
                if self.cur_actionset_runner is not None:
                    while not self.stopping:
                        act = next(self.cur_actionset_runner)
                        duration = act[0] / 1000.0
                        t = time.perf_counter() + duration
                        do_action(act)
                        rospy.sleep(max(duration - 0.01, 0.01))
                        while t > time.perf_counter() and not self.stopping: 
                            pass
                    self.cur_actionset_runner = None
            except Exception as e:
                rospy.logerr("RUN ACTION " + str(e))
                self.cur_actionset_runner = None
                continue

            # 设置姿态
            try:
                if self.new_pose_setter is not None:
                    self.cur_pose_setter = self.new_pose_setter
                    self.new_pose_setter = None
                if self.cur_pose_setter is not None:
                    pose, transform, duration = self.cur_pose_setter
                    if pose is None or transform is None:
                        self.set_pose_base(self.pose, duration, update_pose=True)
                    else:
                        self.set_pose_base(pose, duration, update_pose=True)
                        self.transform = transform
                    self.reset_all_cur_gen()
                    self.reset_all_new_gen()
            except Exception as e:
                rospy.logerr("SET POSE " + str(e))
                self.reset_all_cur_gen()
                self.reset_all_new_gen()
                continue
            pose = self.pose
            transform = self.transform
            moving_pose = None

            # 进行姿态变换
            try:
                if self.new_pose_transformer is not None:
                    self.cur_pose_transformer = self.new_pose_transformer # 如果没有新的变换任务， 那么new_pose_transformer 就是 None
                    self.new_pose_transformer = None

                if self.cur_pose_transformer is not None:
                    pose, transform, last_part = self.cur_pose_transformer.send((pose, transform))
                    if last_part:
                        self.cur_pose_transformer = None
            except Exception as e:
                rospy.logerr("TRANSFORM " + str(e))
                self.cur_pose_transformer = None

            # 进行步态处理
            # 走路有点儿特殊， 停止前总是要先走当前的完一整步，这样可以简化逻辑。
            # 当然可以总是重新计算并实时更新步态路径但是，这样会带来些别的问题。
            # 所以我们让机器人在开始一步之后总是要走完完整一步再停止
            params = None
            try:
                if self.cur_moving_generator is None:
                    if self.new_moving_generator is not None:
                        self.cur_moving_generator = self.new_moving_generator

                if self.cur_moving_generator is not None:
                    moving_pose, last_part, params = self.cur_moving_generator.send(pose)
                    if last_part:
                        if self.cur_moving_generator is not self.new_moving_generator:
                            self.cur_moving_generator = self.new_moving_generator
                        if self.cur_moving_generator is None:
                            self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
                else:
                    self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
                        
            except Exception as e:
                #rospy.logerr("TRAVELNG " + str(e))
                self.cur_moving_generator = None

            #time.sleep(0.01)
            #while True:
            #    t2 = time.perf_counter()
            #    if t2 > t1:
            #        t1 = t2 + 0.02
            #        break
            # 应用新的姿态
            if pose is not self.pose:
                try:
                    self.set_pose_base(pose, 0.02, pseudo=(moving_pose is not None), update_pose=True)
                    self.transform = transform
                    print(self.transform)
                except Exception as e:
                    rospy.logerr("POSE " + str(e))
                    self.cur_pose_transformer = None

            if moving_pose is not None:
                try:
                    self.set_pose_base(moving_pose, 0.02, pseudo=False, update_pose=False)
                    self.transform = transform
                    if isinstance(params, CmdVelParams):
                        self.linear_x = params.velocity_x / 1000.0
                        self.linear_y = params.velocity_y / 1000.0
                        self.angular_z = params.angular_z
                        yaw = self.real_pose_yaw if self.real_pose_yaw else self.pose_yaw
                        x = math.cos(yaw) * (self.linear_x) * 0.02 - math.sin(yaw) * self.linear_y * 0.02;
                        y = math.sin(yaw) * (self.linear_x) * 0.02 + math.cos(yaw) * self.linear_y * 0.02;
                        self.position = self.position[0] + x, self.position[1] + y, self.position[2]
                        self.pose_yaw += self.angular_z * 0.02
                except Exception as e:
                    rospy.logerr("MOVING_POSE " + str(e))
                    self.cur_moving_generator = None
                    self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
            else:
                self.linear_x, self.linear_y, self.angular_z = 0, 0, 0
                
            if self.cur_moving_generator is None and self.cur_pose_setter is None and self.cur_pose_transformer is None and self.cur_actionset_runner is None:
                self.stopping = False
                self.stopped.set()

            rospy.sleep(0.02)


    def stop_running(self, timeout=0, callback=None):
        """
        停止当前正在执行的任务
        :param timeout: 超时实际, 超过这个实际还没停止的话直接返回
        """
        with self.lock:
            self.reset_all_new_gen() # 将所有现有的新指令清空
            self.stopping = True
            self.stopped.clear()
            if timeout is None:
                self.stopped.wait()
            elif timeout > 0:
                self.stopped.wait(timeout)
        if callable(callback):
            callback()

    def set_leg_position(self, leg_id, position, duration, pseudo=False, update_pose=False):
        """
        根据输入的指定的腿及末端位置， 计算、设置舵机角度
        此方法将可能更新类成员pose
        :param leg: 腿的号数
        :param position: 末端位置
        :param duration: 完成此次移动所用时间
        :param pseudo: 是否真的执行移动， 若True则只返回计算得到的对应舵机角度而不真正发送控制指令给舵机
        :param update_pose: 是否更新类成员pose, 此成员记录了机器人的当前姿态
        :return: 末端位置对应的舵机角度（里(id, 角度）， 中(id, 角度）， 外）, 角度为0-1000的数值
        """
        joints = kinematics.set_leg_position(leg_id, position)  # 计算新末端位置对应的各个舵机的角度
        joints_id_radians = zip([(leg_id - 1) * 3 + i + 1 for i, s in enumerate(joints)], joints)
        if not pseudo:
            for joint_id, rad in joints_id_radians:
                new_joints_state = x_joint_control.set_joint(joint_id, rad, duration, self.joints_state)
                self.joints_state = new_joints_state
        if update_pose:
            pose = list(self.pose)
            pose[leg_id - 1] = tuple(position)
            self.pose = tuple(pose)
        return joints

    def set_joint(self, joint_id, radians, duration):
        """
        设置关节角度
        :param joint_id: 关节id
        :param radians: 关节角度， 单位为弧度
        :param duration: 完成此动作的用时
        """
        new_joints_state = x_joint_control.set_joint(joint_id, radians, duration, self.joints_state)
        self.joints_state = new_joints_state

    def set_leg_relatively(self, leg_id, offset, duration):
        cur_pos = list(self.pose[leg_id - 1])
        new_pos = cur_pos[0] + offset[0], cur_pos[1] + offset[1], cur_pos[2] + offset[2]
        self.set_leg_position(leg_id, new_pos, duration)

    def set_pose_base(self, new_pose, duration, pseudo=False, update_pose=False):
        """
        设置机器人的姿态的基础调用，其他 function 都会调用我
        此方法将更新类成员pose
        :param new_pose:  机器人的新姿态，六条腿的末端坐标,形如（(x1, y1, z1), (x2, y2, z2),...)
        :param duration: 完成这次动作所用时间
        :param pseudo: 是否真的控制舵机运动， 若为True则只计算并设置相应变量而不真正发送控制指令给舵机
        :return: None
        """
        joints = [kinematics.set_leg_position(i + 1, position) for i, position in enumerate(new_pose)]
        joints = list(itertools.chain.from_iterable(joints))
        joints_data = [[j, r, duration] for j, r in zip(list(range(1, 19)), joints)]
        if not pseudo:
            self.joints_state = x_joint_control.set_multi_joints(joints_data, self.joints_state)
        if update_pose:
            self.pose = tuple(map(tuple, new_pose))
        

    def set_pose(self, pose, transform, duration, interrupt=True):
        """
        设置机器人的姿态
        :param pose: 新姿态
        :param duration:  完成这次动作所用时间
        :return:
        """
        with self.lock:
            if self.pose is None:
                self.new_pose_setter = (None, None, duration)
            else:
                self.new_pose_setter = (pose, transform, duration)

    
    def set_build_in_pose(self, pose_name, duration, interrupt=True):
        """
        设置机器人的姿态
        :param pose: 新姿态
        :param duration:  完成这次动作所用时间
        :return: None
        """
        with self.lock:
            pose =  getattr(build_in_pose, pose_name)
            transform = getattr(build_in_pose, pose_name + '_TRANSFORM')
            self.new_pose_setter = (pose, transform, duration)



    def transform_pose(self, translate, quaternion, duration):
        """
        使用平移变换加四元数改变机器人的姿态
        :param translate: 机体中心偏移 (x, y, z)
        :param quaternion: 机体的旋转变换四元数 (x, y, z, w)
        :param duration: 完成这个变换的用时
        :return: None
        """
        # 设置新的机器人姿态
        self.set_pose_base(kinematics_api.transform_quat(translate, quaternion), duration) 
    

    def transform_absolutely(self, translate, euler, duration):
        generator = PoseTransformer(PoseTransformerParams(translation=translate, rotation=euler, absolutely=True, duration=duration))
        if generator:
            with self.lock:
                generator.send(None)
                self.new_pose_transformer = generator

    def transform_pose_2(self, translate, axis, euler, duration, degrees=True):
        """
        使用平移变换加欧拉角改变机器人的姿态
        :param translate: 机体中心偏移的平移变换 (x, y, z)
        :param axis: 欧拉角三个轴的顺序 如 'xyz' 或者 'yzx'
        :param euler: 欧拉角的元组, 顺序要与axis一致
        :param duration: 完成这个变换的用时
        :param degrees: 欧拉角单位是否为角度, True为角度, False为弧度
        """
        rotate = R.from_euler(axis, euler, degrees=degrees)  # 用欧拉角建立转换器
        r = rotate.as_euler('xyz', degrees=False) # 转换成固定顺序的欧拉角
        generator = PoseTransformer(PoseTransformerParams(translation=translate, rotation=r, duration=duration))

        if generator:
            with self.lock:
                generator.send(None)
                self.new_pose_transformer = generator
    
    def cmd_vel(self, twist: geometry_msgs.msg.Twist):
        linear_x = twist.linear.x * 1000 # linear_x 单位为 毫米每秒
        linear_y = twist.linear.y * 1000 # linear_y 单位为 毫米每秒
        angular_z = twist.angular.z # 旋转角速度 rad/sec
        generator = CmdVelGenerator(CmdVelParams(
            gait =  self.cmd_gait,
            velocity_x = linear_x,
            velocity_y = linear_y,
            angular_z = angular_z,
            height = self.cmd_height,
            relative_h = False,
            period = self.cmd_period,
            linear_factor = 1.0,
            rotate_factor = 1.0 # 转向时的转向系数， 即给定的梅
        ))
        if generator:
            with self.lock:
                generator.send(None)
                self.new_moving_generator = generator
        # print(twist)
        # freq = 1.0 / (self.cmd_period * 6) # 步频， 单位为 步每秒
        # if linear_x == 0:
        #     angular = angular_z / freq
        #     self.set_step_mode(self.cmd_gait, 0,  self.cmd_height, 0, angular, self.cmd_period, repeat=0)
        # elif angular_z == 0: 
        #     stride = linear_x / freq # 步幅, 单位为米
        #     print(freq, stride)
        #     self.set_step_mode(self.cmd_gait, stride,  self.cmd_height, 0, 0, self.cmd_period, repeat=0)
        # elif angular_z != 0 and linear_x != 0:
        #     angular = angular_z / freq
        #     angular = (angular / math.radians(10))
        #     print(angular)
        #     stride = linear_x / freq # 步幅, 单位为米
        #     self.set_step_mode(self.cmd_gait, stride,  self.cmd_height, 0, angular, self.cmd_period, repeat=0)
        # else:
        #     print("KJLKDFJKLSDJKLF")
        #     self.stop_running(timeout=None)


    def set_step_mode(self,
                      gait,
                      amplitude,
                      height,
                      direction,
                      rotation,
                      duration,
                      repeat=1,
                      relative_height=False,
                      rectify=True,
                      integral=True,
                      interrupt=True,
                      feedback_cb=None):
        if gait == 11 or gait == 12 or gait == 13:
            self.cmd_period = duration
            self.cmd_gait = gait - 10
            self.cmd_height = height
        else:
            self.set_step_mode_base(gait, amplitude, height, direction, rotation, duration, repeat, relative_height, rectify, integral, feedback_cb)


    def set_step_mode_base(self,
                      gait,
                      amplitude,
                      height,
                      direction,
                      rotation,
                      duration,
                      repeat=1,
                      relative_height=False,
                      rectify=True,
                      integral=True,
                      interrupt = True,
                      feedback_cb=None):
        """
        设置机器人的运动步态
        :param gait: 步态
        :param amplitude: 步幅
        :param height: 步高, 即走路时脚尖的抬起高度
        :param direction: 运动方向
        :param rotation: 机器人绕机体中心的旋转角速度
        :param period: 每步用时
        :param repeat: 要走的步数, 0会一直走下去
        :param relative_height: 步高参数是否为相对高度
        :param rectify: 对实际行走距离的校正参数
        :param integral: 是否对行走距离进行积分实现里程计
        :param feedback_cb: 运行中状态报告的回调，不建议使用
        """
        generator = MovingGenerator(MovingParams(
                gait=gait,
                stride = amplitude,
                height = height,
                direction = direction,
                rotation = rotation,
                period = duration,
                repeat = repeat,
                forever = True if repeat == 0 else False,
                relative_h = relative_height,
                ))
        if generator:
            with self.lock:
                generator.send(None)
                self.new_moving_generator = generator


    def run_action_set(self, file, repeat=1, interrupt=True):
        runner = actionset_runner(self, file, repeat)
        if runner:
            with self.lock:
                self.new_actionset_runner = runner
