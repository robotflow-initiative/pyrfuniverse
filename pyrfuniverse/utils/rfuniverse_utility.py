import math

from numpy import uint8


def EncodeIDAsColor(instance_id: int):
    r = (instance_id * 16807 + 187) % 256
    g = (instance_id * 48271 + 79) % 256
    b = (instance_id * 95849 + 233) % 256
    return [r, g, b, 255]

def UnityEularToQuaternion(eular):
    xx = math.radians(eular[0])
    yy = math.radians(eular[1])
    zz = math.radians(eular[2])
    x = math.cos(yy / 2) * math.sin(xx / 2) * math.cos(zz / 2) + math.sin(yy / 2) * math.cos(xx / 2) * math.sin(zz / 2)
    y = math.sin(yy / 2) * math.cos(xx / 2) * math.cos(zz / 2) - math.cos(yy / 2) * math.sin(xx / 2) * math.sin(zz / 2)
    z = math.cos(yy / 2) * math.cos(xx / 2) * math.sin(zz / 2) - math.sin(yy / 2) * math.sin(xx / 2) * math.cos(zz / 2)
    w = math.cos(yy / 2) * math.cos(xx / 2) * math.cos(zz / 2) + math.sin(yy / 2) * math.sin(xx / 2) * math.sin(zz / 2)
    return [x, y, z, w]

def UnityQuaternionToEular(quaternion):
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
    """Check keyword arguments, make sure all compulsory parameters are included.
    Args:
        kwargs: Keyword arguments.
        compulsory_params: Compulsory parameters.
    """
    legal = True
    for param in compulsory_params:
        if param not in kwargs.keys():
            legal = False
            assert legal, \
                'Parameters illegal, parameter <%s> missing.' % param