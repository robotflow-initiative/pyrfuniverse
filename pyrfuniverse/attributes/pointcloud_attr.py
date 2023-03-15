import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def ShowPointCloud(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'positions', 'colors']
    optional_params = ['radius']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    if 'radius' not in kwargs:
        kwargs['radius'] = 0.01
    msg.write_int32(kwargs['id'])
    msg.write_string('ShowPointCloud')
    msg.write_float32_list(kwargs['positions'])
    msg.write_float32_list(kwargs['colors'])
    msg.write_float32(kwargs['radius'])
    return msg

def SetRadius(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'radius']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetRadius')
    msg.write_float32(kwargs['radius'])
    return msg


class PointCloudAttr(attr.BaseAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        return self.data

    def ShowPointCloud(self, positions: list, colors: list, radius: float = 0.01):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('ShowPointCloud')
        msg.write_float32_list(positions)
        msg.write_float32_list(colors)
        msg.write_float32(radius)

        self.env.instance_channel.send_message(msg)

    def SetRadius(self, radius: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRadius')
        msg.write_float32(radius)

        self.env.instance_channel.send_message(msg)