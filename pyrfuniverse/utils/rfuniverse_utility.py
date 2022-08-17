import math

from numpy import uint8


def EncodeIDAsColor(instance_id: int):
    uid = instance_id * 2
    if uid < 0:
        uid = -uid + 1
    sid = (SparsifyBits(uint8(uid >> 16), 3) << 2) | (SparsifyBits(uint8(uid >> 8), 3) << 1) | SparsifyBits(uint8(uid), 3)
    r = uint8(sid >> 8)
    g = uint8(sid >> 16)
    b = uint8(sid)
    return [r, g, b, 255]


def SparsifyBits(value: uint8, sparse: int):
    retval = 0
    for i in range(8):
        retval |= (value & 1)
        retval <<= sparse
        value >>= 1
    return retval >> sparse

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