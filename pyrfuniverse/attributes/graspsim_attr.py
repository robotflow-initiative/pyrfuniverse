import pyrfuniverse.attributes as attr
import pyrfuniverse.utils.rfuniverse_utility as utility
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


def parse_message(msg: IncomingMessage) -> dict:
    this_object_data = attr.base_attr.parse_message(msg)
    done = msg.read_bool()
    this_object_data['done'] = done
    if done:
        this_object_data['points'] = msg.read_float32_list()
        this_object_data['quaternions'] = msg.read_float32_list()
        this_object_data['width'] = msg.read_float32_list()
    return this_object_data


# 新增接口示例
def StartGraspSim(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'mesh', 'gripper', 'points', 'normals', 'depth_range_min', 'depth_range_max',
                         'depth_lerp_count', 'angle_lerp_count']
    optional_params = ['parallel_count']
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('StartGraspSim')
    msg.write_string(kwargs['mesh'])
    msg.write_string(kwargs['gripper'])
    msg.write_float32_list(kwargs['points'])
    msg.write_float32_list(kwargs['normals'])
    msg.write_float32(kwargs['depth_range_min'])
    msg.write_float32(kwargs['depth_range_max'])
    msg.write_int32(kwargs['depth_lerp_count'])
    msg.write_int32(kwargs['angle_lerp_count'])
    if 'parallel_count' not in kwargs:
        kwargs['parallel_count'] = 100
    msg.write_int32(kwargs['parallel_count'])
    return msg


def GenerateGraspPose(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'mesh', 'gripper', 'points', 'normals', 'depth_range_min', 'depth_range_max',
                         'depth_lerp_count', 'angle_lerp_count']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GenerateGraspPose')
    msg.write_string(kwargs['mesh'])
    msg.write_string(kwargs['gripper'])
    msg.write_float32_list(kwargs['points'])
    msg.write_float32_list(kwargs['normals'])
    msg.write_float32(kwargs['depth_range_min'])
    msg.write_float32(kwargs['depth_range_max'])
    msg.write_int32(kwargs['depth_lerp_count'])
    msg.write_int32(kwargs['angle_lerp_count'])
    return msg


def ShowGraspPose(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'mesh', 'gripper', 'positions', 'quaternions']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('ShowGraspPose')
    msg.write_string(kwargs['mesh'])
    msg.write_string(kwargs['gripper'])
    msg.write_float32_list(kwargs['positions'])
    msg.write_float32_list(kwargs['quaternions'])
    return msg
