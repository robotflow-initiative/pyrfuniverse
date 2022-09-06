import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility
import base64

def parse_message(msg: IncomingMessage) -> dict:
    this_object_data = attr.base_attr.parse_message(msg)
    this_object_data['FOV'] = msg.read_float32()
    this_object_data['width'] = msg.read_int32()
    this_object_data['height'] = msg.read_int32()
    if msg.read_bool() is True:
        this_object_data['rgb'] = base64.b64decode(msg.read_string())
    if msg.read_bool() is True:
        this_object_data['normal'] = base64.b64decode(msg.read_string())
    if msg.read_bool() is True:
        this_object_data['id_map'] = base64.b64decode(msg.read_string())
    if msg.read_bool() is True:
        this_object_data['depth'] = base64.b64decode(msg.read_string())
    if msg.read_bool() is True:
        this_object_data['depth_exr'] = base64.b64decode(msg.read_string())
    if msg.read_bool() is True:
        this_object_data['amodal_mask'] = base64.b64decode(msg.read_string())
    if msg.read_bool() is True:
        ddbbox_count = msg.read_int32()
        this_object_data['amodal_mask'] = []
        for i in range(ddbbox_count):
            this_object_data['amodal_mask'][i] = [msg.read_float32() for _ in range(4)]
    return this_object_data


def AlignView(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('AlignView')

    return msg

def GetRGB(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'width', 'height']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetRGB')
    msg.write_int32(kwargs['width'])
    msg.write_int32(kwargs['height'])

    return msg

def GetNormal(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'width', 'height']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetNormal')
    msg.write_int32(kwargs['width'])
    msg.write_int32(kwargs['height'])

    return msg

def GetID(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'width', 'height']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetID')
    msg.write_int32(kwargs['width'])
    msg.write_int32(kwargs['height'])

    return msg

def GetDepth(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'width', 'height', 'zero_dis', 'one_dis']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetDepth')
    msg.write_int32(kwargs['width'])
    msg.write_int32(kwargs['height'])
    msg.write_int32(kwargs['zero_dis'])
    msg.write_int32(kwargs['one_dis'])

    return msg

def GetDepthEXR(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'width', 'height']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetDepthEXR')
    msg.write_int32(kwargs['width'])
    msg.write_int32(kwargs['height'])

    return msg

def GetAmodalMask(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'width', 'height']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetAmodalMask')
    msg.write_int32(kwargs['width'])
    msg.write_int32(kwargs['height'])

    return msg