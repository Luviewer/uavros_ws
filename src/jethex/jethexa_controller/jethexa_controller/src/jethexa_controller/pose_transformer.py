import math
from jethexa_controller import kinematics_api

class PoseTransformerParams:
    def __init__(self, **kwargs):
        self.translation = (0, 0, 0)
        self.rotation = (0, 0, 0)
        self.absolutely = False # 绝对变换
        self.duration = 1
        self.__dict__.update(kwargs) # 用输入的 named arguments 更新类成员

def PoseTransformer(params:PoseTransformerParams):
    """
    使用平移变换加欧拉角改变机器人的姿态
    :param translate: 机体中心偏移的平移变换 (x, y, z)
    :param rotation: 欧拉角的元组, 'xyz'
    :param duration: 完成这个变换的用时
    """
    div_num = max(math.ceil(params.duration / 0.02), 1)# 计算可以分为多少步, 且最少要有一步

    # 本次变换要进行的总的变换量
    x, y, z = params.translation
    u, v, w = params.rotation


    org_pose, cur_transform = yield None

    # 当前机器人相对于最初始的变换
    (org_x, org_y, org_z), (org_u, org_v, org_w) = cur_transform

    if params.absolutely:
        x, y, z = (x - org_x), (y - org_y), (z - org_z)
        u, v, w = (u - org_u), (v - org_v), (w - org_w)
    
        params.translation = x, y, z
        params.rotation = u, v, w

    div_u, div_v, div_w = u / div_num, v / div_num, w / div_num

    # 每一小步的变换量
    div_x, div_y, div_z = x / div_num, y / div_num, z / div_num
    # 进行完这次变换后的机器人相对于最初始的变换
    final_transform =  (org_x + x, org_y + y, org_z + z), (org_u + u, org_v + v, org_w + w)

    # 进行完这次变换后的机器人姿态(六只脚的脚尖在哪里)
    final_pose = kinematics_api.transform_euler(org_pose, params.translation, 'xyz', params.rotation, degrees=False)
    
    div_num -= 1
    while div_num >= 0:
        if div_num > 0: # 计算每分步的变换后位置及变换后的绝对变换值
            # 平移和欧拉角都是线性的, 可以直接用加法分步加到目标值
            # 注意输入的欧拉角顺序一定为 'xyz'
            nx, ny, nz = x - div_x * div_num, y - div_y * div_num, z - div_z * div_num
            nu, nv, nw = u - div_u * div_num, v - div_v * div_num, w - div_w * div_num

            # 完成本分步后相对与初始姿态(执行 set_build_in_pose 设置的姿态)的变换
            out_tranform = (org_x + nx, org_y + ny, org_z + nz), (org_u + nu, org_v + nv, org_w + nw)
            # 完成本分步后的姿态
            cur_pose = kinematics_api.transform_euler(org_pose, (nx, ny, nz), 'xyz', (nu, nv, nw), degrees=False)


            yield cur_pose, out_tranform, False
        else:
            yield final_pose, final_transform, True
        div_num -= 1