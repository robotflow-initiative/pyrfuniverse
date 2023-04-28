import math
import numpy as np

def EncodeIDAsColor(instance_id: int):
    """
    获取物体在ID图中的颜色

    Args:
        instance_id: 物体ID
    """
    r = (instance_id * 16807 + 187) % 256
    g = (instance_id * 48271 + 79) % 256
    b = (instance_id * 95849 + 233) % 256
    return [r, g, b, 255]

def UnityEularToQuaternion(eular: list) -> list:
    """
    Unity欧拉角转四元数

    Args:
        eular: 欧拉角,长度为3,分别为x,y,z轴的旋转角度,单位为度

    Return:
        四元数,长度为4,分别为x,y,z,w四个分量
    """
    xx = math.radians(eular[0])
    yy = math.radians(eular[1])
    zz = math.radians(eular[2])
    x = math.cos(yy / 2) * math.sin(xx / 2) * math.cos(zz / 2) + math.sin(yy / 2) * math.cos(xx / 2) * math.sin(zz / 2)
    y = math.sin(yy / 2) * math.cos(xx / 2) * math.cos(zz / 2) - math.cos(yy / 2) * math.sin(xx / 2) * math.sin(zz / 2)
    z = math.cos(yy / 2) * math.cos(xx / 2) * math.sin(zz / 2) - math.sin(yy / 2) * math.sin(xx / 2) * math.cos(zz / 2)
    w = math.cos(yy / 2) * math.cos(xx / 2) * math.cos(zz / 2) + math.sin(yy / 2) * math.sin(xx / 2) * math.sin(zz / 2)
    return [x, y, z, w]

def UnityQuaternionToEular(quaternion: list) -> list:
    """
    Unity四元数转欧拉角

    Args:
        quaternion: 四元数,长度为4,分别为x,y,z,w四个分量

    Return:
        欧拉角,长度为3,分别为x,y,z轴的旋转角度,单位为度
    """
    xx = quaternion[0]
    yy = quaternion[1]
    zz = quaternion[2]
    ww = quaternion[3]
    x = math.asin(2 * ww * xx - 2 * yy * zz)
    y = math.atan2(2 * ww * yy + 2 * xx * zz, 1 - 2 * xx * xx - 2 * yy * yy)
    z = math.atan2(2 * ww * zz + 2 * xx * yy, 1 - 2 * xx * xx - 2 * zz * zz)
    x = math.degrees(x)
    y = math.degrees(y)
    z = math.degrees(z)
    return [x, y, z]

def CheckKwargs(kwargs: dict, compulsory_params: list):
    legal = True
    for param in compulsory_params:
        if param not in kwargs.keys():
            legal = False
            assert legal, \
                'Parameters illegal, parameter <%s> missing.' % param

def GetMatrix(pos, quat) -> np.ndarray:
    """
    位置和四元数转换为矩阵

    Args:
        pos: 位置,长度为3,分别为x,y,z轴的坐标
        quat：四元数,长度为4,分别为x,y,z,w四个分量

    Return:
        ndarray矩阵,shape为(4,4)
    """
    q = quat.copy()
    n = np.dot(q, q)
    if n < np.finfo(q.dtype).eps:
        return np.identity(4)
    q = q * np.sqrt(2.0 / n)
    q = np.outer(q, q)
    # rot_matrix = np.array(
    #     [[1.0 - q[2, 2] - q[3, 3], q[1, 2] + q[3, 0], q[1, 3] - q[2, 0], pos[0]],
    #      [q[1, 2] - q[3, 0], 1.0 - q[1, 1] - q[3, 3], q[2, 3] + q[1, 0], pos[1]],
    #      [q[1, 3] + q[2, 0], q[2, 3] - q[1, 0], 1.0 - q[1, 1] - q[2, 2], pos[2]],
    #      [0, 0, 0, 1.0]], dtype=q.dtype)
    matrix = np.array(
        [[1.0 - q[1, 1] - q[2, 2], -(q[2, 3] - q[1, 0]), q[1, 3] + q[2, 0], pos[0]],
         [q[2, 3] + q[1, 0], -(1.0 - q[1, 1] - q[3, 3]), q[1, 2] - q[3, 0], pos[1]],
         [-(q[1, 3] - q[2, 0]), q[1, 2] + q[3, 0], -(1.0 - q[2, 2] - q[3, 3]), pos[2]],
         [0., 0., 0., 1.0]], dtype=float)
    return matrix

