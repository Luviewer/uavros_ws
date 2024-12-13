import rospy
from copy import deepcopy
from .config import SERVOS, ServoType, SIMULATE
from std_msgs.msg import Float64
import time

topics = None

def initialize_publisher(prefix):
    global topics
    topics = {}
    for value in SERVOS.values():
        topics[value['name']] = rospy.Publisher(prefix + '/' + value['name'] + '_position_controller/command', Float64, queue_size=10)



def set_joint(joint_id: int, radians: float, duration: float, joints_state: dict = None) -> dict:
    """
    设置物理舵机角度

    :param joint_id: 舵机id，这里不是真正的物理ID， 而是在SERVOS里面定义的id
    :param radians: 目标多级角度
    :param duration: 到达目标角度的用时
    :param joints_state: 当前的关节状态字典
    :type servo_id: iint
    :type radians: float
    :type duration: float
    :type joint_state: dict

    :return: 新的关节状态字典
    :rtype: dict
    """
    global topics 

    if not joint_id in SERVOS:
        raise ValueError("Invalid joint id %d" % joint_id)


    servo = SERVOS[joint_id]
    servo_name = servo['name']
    servo_id = servo['id']
    servo_type = servo['type']
    center = servo['center']
    ticks = servo['ticks']
    max_radians = servo['max_radians']
    direction = servo['direction']
    offset = servo['offset']

    # 舵机位置数值的最大最小值
    max_ticks = int(center + ticks / 2)
    min_ticks = int(center - ticks / 2)

    # 最终的舵机真实角度
    real_radians = direction * (offset + radians)
    if abs(real_radians) > abs(max_radians / 2):
        raise ValueError("Invalid radians {:.4f}".format(radians))

    # 计算舵机位置数值
    pos_a = (real_radians - (-max_radians / 2)) / max_radians * ticks
    pos_b = pos_a + min_ticks
    pos = int(pos_b)
    #限幅, 确保数值有效
    pos = pos if pos < max_ticks else max_ticks
    pos = pos if pos > min_ticks else min_ticks

    duration = int(duration * 1000 + 0.5)

    if not SIMULATE:
        """
        if servo_type == ServoType.BUS:
            t = time.time()
            serial_servo.set_position(servo_id, pos, duration)
        elif servo_type == ServoType.PWM:
            pwm_servos = pwm_servo.pwm_servo1, pwm_servo.pwm_servo2
            pwm_servos[servo_id - 1].set_position(pos, duration)
        else:
            raise ValueError("Invalid pwm servo id %d", servo_id)
        """
        pass
    else:
        if not topics:
            initialize_publisher("/jethexa")
        topics[servo_name].publish(Float64(data=real_radians))

    if joints_state is not None:
        joints_state[servo_name] = radians
    return joints_state


def set_multi_joints(data, joints_state: dict = None) -> dict:
    """
    设置多个物理舵机角度

    :return: 新的关节状态字典
    :rtype: dict
    """
    global topics
    servos_data = []
    new_joints_state = deepcopy(joints_state)
    for joint_id, radians, duration in data:
        if not joint_id in SERVOS:
            raise ValueError("Invalid joint id %d" % joint_id)
        servo = SERVOS[joint_id]
        servo_name = servo['name']
        servo_id = servo['id']
        servo_type = servo['type']
        center = servo['center']
        ticks = servo['ticks']
        max_radians = servo['max_radians']
        direction = servo['direction']
        offset = servo['offset']

        # 舵机位置数值的最大最小值
        max_ticks = int(center + ticks / 2)
        min_ticks = int(center - ticks / 2)

        # 最终的舵机真实角度
        real_radians = direction * (offset + radians)
        if abs(real_radians) > abs(max_radians / 2):
            raise ValueError("Invalid radians {:.4f}".format(radians))

        # 计算舵机位置数值
        pos = int((real_radians - (-max_radians / 2)) / max_radians * ticks + min_ticks)
        if not SIMULATE:
            """
            if servo_type == ServoType.BUS:
                servos_data.append([servo_id, pos, int(duration * 1000)])
            elif servo_type == ServoType.PWM:
                pwm_servos = pwm_servo.pwm_servo1, pwm_servo.pwm_servo2
                pwm_servos[servo_id - 1].set_position(pos, duration)
            else:
                raise ValueError("Invalid pwm servo id %d", servo_id)
            """
            pass
        else:
            if not topics:
                initialize_publisher("/jethexa")
            topics[servo_name].publish(Float64(data=real_radians))
        if new_joints_state is not None:
            new_joints_state[servo_name] = radians

    if len(servos_data) > 0:
        serial_servo.set_multi_position(servos_data)

    return new_joints_state
