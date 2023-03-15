import pyrfuniverse.attributes as attr
import pyrfuniverse.utils.rfuniverse_utility as utility
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


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


def StartGraspTest(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'mesh', 'gripper', 'points', 'quaternions']
    optional_params = ['parallel_count']
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('StartGraspTest')
    msg.write_string(kwargs['mesh'])
    msg.write_string(kwargs['gripper'])
    msg.write_float32_list(kwargs['points'])
    msg.write_float32_list(kwargs['quaternions'])
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


class GraspSimAttr(attr.BaseAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        self.data['done'] = msg.read_bool()
        if self.data['done']:
            mode = msg.read_int32()
            if mode == 0:
                self.data['points'] = msg.read_float32_list()
                self.data['quaternions'] = msg.read_float32_list()
                self.data['width'] = msg.read_float32_list()
            if mode == 1:
                self.data['success'] = msg.read_float32_list()
        return self.data

    def StartGraspSim(self, mesh: str, gripper: str, points: list, normals: list, depth_range_min: float, depth_range_max: float, depth_lerp_count: int, angle_lerp_count: int, parallel_count: int = 100):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('StartGraspSim')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(points)
        msg.write_float32_list(normals)
        msg.write_float32(depth_range_min)
        msg.write_float32(depth_range_max)
        msg.write_int32(depth_lerp_count)
        msg.write_int32(angle_lerp_count)
        msg.write_int32(parallel_count)

        self.env.instance_channel.send_message(msg)

    def GenerateGraspPose(self, mesh: str, gripper: str, points: list, normals: list, depth_range_min: float, depth_range_max: float, depth_lerp_count: int, angle_lerp_count: int):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GenerateGraspPose')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(points)
        msg.write_float32_list(normals)
        msg.write_float32(depth_range_min)
        msg.write_float32(depth_range_max)
        msg.write_int32(depth_lerp_count)
        msg.write_int32(angle_lerp_count)

        self.env.instance_channel.send_message(msg)

    def StartGraspTest(self, mesh: str, gripper: str, points: list, quaternions: list, parallel_count: int = 100):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('StartGraspTest')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(points)
        msg.write_float32_list(quaternions)
        msg.write_int32(parallel_count)

        self.env.instance_channel.send_message(msg)

    def ShowGraspPose(self, mesh: str, gripper: str, positions: list, quaternions: list):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('ShowGraspPose')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(positions)
        msg.write_float32_list(quaternions)

        self.env.instance_channel.send_message(msg)
