import math
import numpy as np
import kinematics
import rospy

class MovingParams:
    def __init__(self, **kwargs):
        self.forever = False # 永远运行
        self.repeat = 1 # 重复次数
        self.interrupt = True # 是否中断当前移动

        self.gait = 1 # 步态选择
        self.stride = 0 # 步态步幅
        self.height = 0 # 步态抬腿高度
        self.direction = 0 # 移动方向
        self.rotation = 0 # 每步自转角度
        self.relative_h = False # 步高使用相对高度
        self.linear_factor = 1.0 # 直线行走时的步幅系数， 即走一步给定的步幅和实际的步幅
        self.rotate_factor = 1.0 # 转向时的转向系数， 即给定的梅
        self.velocity_x = 0 
        self.velocity_y = 0 

        self.period = 10 # 每步间隔
        self.__dict__.update(kwargs) # 用具名参数更新实例
    
    def __str__(self) -> str:
        return str(self.__dict__)

class CmdVelParams:
    def __init__(self, **kwargs):
        self.gait=2,
        self.velocity_x = 0 
        self.velocity_y = 0 
        self.angular_z = 0
        self.height = 10.0
        self.relative_h = False # 步高使用相对高度

        self.period = 1 # 每步间隔

        self.linear_factor = 1.0 # 直线行走时的步幅系数， 即走一步给定的步幅和实际的步幅
        self.rotate_factor = 1.0 # 转向时的转向系数， 即给定的梅

        self.__dict__.update(kwargs) # 用具名参数更新实例

    def __str__(self) -> str:
        return str(self.__dict__)


def MovingGenerator(params):
    # 根据是相对高度还是绝对高度计算实际的高度 
    org_pose = None
    part_index = 0
    sub_index = 0

    sub_action_num = params.period / 6.0 / 0.02 # 一步会分为六步幅， 计算每一部分有多少个子动作
    sub_action_num = math.ceil(round(max(sub_action_num, 1), 3)) # 每一分步最少要有一个子动作
    height = params.height
    real_stride = params.stride * params.linear_factor
    real_rotate = params.rotation * params.rotate_factor

    sub_stride = params.stride / (sub_action_num * 6.0)
    sub_rotate = params.rotation / (sub_action_num * 6.0)

    #print("sub_action_num ", params.period, sub_action_num)

    cur_pose = yield None
    while params.repeat > 0 or params.forever:
        org_pose = cur_pose
        if params.relative_h:
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        poses = [] # 计算到的步态的所有姿态
        # 波纹步态时有根据不同的方向会有条腿刚好在下落过程，
        # 如果在立正状态下下落的话会将机体撑起来，所以需要先将这条腿抬起
        if params.gait == 1:  # 波纹步态
            if params.direction >= math.pi:
                start_leg = list(org_pose[0])
                start_leg[2] += height
                start_pose = list(org_pose)
                start_pose[0] = start_leg
            else:
                start_leg = list(org_pose[3])
                start_leg[2] += height
                start_pose = list(org_pose)
                start_pose[3] = start_leg
        else: # 非波纹步态
            start_pose = org_pose
        
        # 计算整个过程的所有步态过程
        for i in range(6):
            ps = kinematics.set_step_mode(sub_action_num, 
                                          i, 
                                          start_pose, 
                                          params.gait, 
                                          real_stride,
                                          height, 
                                          params.direction, 
                                          real_rotate)
            ps = np.array(ps)
            # 出来的数据是每条对的多个姿态，转换为六条腿的子动作的姿态
            ps = ps.reshape((6, -1, 3))  # 转换为[六条腿[子动作[坐标]]]
            ps = np.transpose(ps, (1, 0, 2))  # 转换为[子动作[六条腿[坐标]]]
            start_pose = ps[-1]
            poses.append(ps)
            
        while params.repeat > 0 or params.forever:
            out_pose = poses[part_index][sub_index]

            # 各个计数进行调整
            sub_index = (sub_index + 1) % sub_action_num # 子动作计数 +1
            if sub_index == 0:
                part_index = (part_index + 1) % 6 # 分步计数 +1
                if part_index == 0:
                    params.repeat = max(params.repeat - 1, 0)
            cur_pose = yield out_pose, part_index == 0 and sub_index == 0, params

            # 两次初始姿态变了我们需要重算
            if cur_pose is not org_pose:
                break



def CmdVelGenerator(params):
    height = params.height
    org_pose = None
    phase_index = 0 # 当前相位游标
    phase_num = math.ceil(round((params.period * 1000.0 / 20.0), 1)) # 细分的相位个数
    phase_list = [(i / phase_num) * 2.0 * math.pi for i in range(phase_num)] # 细分相位列表

    aep_offset_x, aep_offset_y, frac_theta_2_sin, frac_theta_2_cos = kinematics.cmd_vel_basic_data(
        params.velocity_x, #* 1.0, #0.633 ,
        params.velocity_y,
        params.angular_z, #*0.720,
        params.period
    )

    cur_pose = yield None
    while True:
        org_pose = cur_pose
        if params.relative_h:
            height = abs(org_pose[0][2]) * (params.height / 100.0) 
        
        aep, pep = kinematics.cmd_vel_aep_pep(
            cur_pose,
            aep_offset_x,
            aep_offset_y,
            frac_theta_2_sin,
            frac_theta_2_cos,
        )

        steps = []
        for phase in phase_list:
            ps = kinematics.cmd_vel_new_point(
                params.gait,
                height,
                phase,
                aep,
                pep
            )
            steps.append(ps)

        while True:
            """
            ps = kinematics.cmd_vel_new_point(
                params.gait,
                height,
                phase_list[phase_index],
                aep,
                pep
            )
            """

            ps = steps[phase_index]
            phase_index =  (phase_index + 1) % phase_num
            if phase_index == 0:
                cur_pose = yield ps, True, params
            else:
                cur_pose = yield ps, False, params
            # 两次初始姿态变了我们需要重算
            if cur_pose is not org_pose:
                break
