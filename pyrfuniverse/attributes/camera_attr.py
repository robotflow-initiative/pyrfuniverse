import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility
import base64


def AlignView(kwargs: dict):
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('AlignView')

    return msg

def GetRGB(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetRGB')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetNormal(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetNormal')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetID(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetID')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetDepth(kwargs: dict):
    compulsory_params = ['id', 'zero_dis', 'one_dis']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetDepth')
    msg.write_float32(kwargs['zero_dis'])
    msg.write_float32(kwargs['one_dis'])
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetDepthEXR(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetDepthEXR')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetAmodalMask(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetAmodalMask')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg


class CameraAttr(attr.BaseAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        self.data['width'] = msg.read_int32()
        self.data['height'] = msg.read_int32()
        self.data['fov'] = msg.read_float32()
        if msg.read_bool() is True:
            self.data['rgb'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['normal'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['id_map'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['depth'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['depth_exr'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['amodal_mask'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            ddbbox_count = msg.read_int32()
            self.data['2d_bounding_box'] = []
            for i in range(ddbbox_count):
                self.data['2d_bounding_box'][i] = {}
                self.data['2d_bounding_box'][i]['position'] = [msg.read_float32() for _ in range(2)]
                self.data['2d_bounding_box'][i]['size'] = [msg.read_float32() for _ in range(2)]
        if msg.read_bool() is True:
            dddbbox_count = msg.read_int32()
            self.data['3d_bounding_box'] = []
            for i in range(dddbbox_count):
                self.data['3d_bounding_box'][i] = {}
                self.data['3d_bounding_box'][i]['position'] = [msg.read_float32() for _ in range(3)]
                self.data['3d_bounding_box'][i]['rotation'] = [msg.read_float32() for _ in range(3)]
                self.data['3d_bounding_box'][i]['size'] = [msg.read_float32() for _ in range(3)]
        return self.data

    def AlignView(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AlignView')

        self.env.instance_channel.send_message(msg)

    def GetRGB(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetRGB')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetNormal(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetNormal')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetID(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetID')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetDepth(self, zero_dis: float, one_dis: float, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetDepth')
        msg.write_float32(zero_dis)
        msg.write_float32(one_dis)
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetDepthEXR(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetDepthEXR')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetAmodalMask(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetAmodalMask')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)