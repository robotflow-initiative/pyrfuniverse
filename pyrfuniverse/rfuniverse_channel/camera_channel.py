from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import InstanceChannel
import base64
from enum import Enum


class RenderingMode(Enum):
    RGB = -1
    MASK = 0
    DEPTH = 2
    NORMALS = 4
    OPTICAL_FLOW = 5


class CameraChannel:

    def __init__(self, channel: InstanceChannel) -> None:
        self.channel = channel

    def _parse_message(self, msg: IncomingMessage) -> dict:
        this_object_data = {}
        this_object_data['near_plane'] = msg.read_float32()
        this_object_data['far_plane'] = msg.read_float32()
        this_object_data['FOV'] = msg.read_float32()
        this_object_data['target_display'] = msg.read_int32()
        this_object_data['width'] = msg.read_int32()
        this_object_data['height'] = msg.read_int32()
        if msg.read_bool() is True:
            this_object_data['rgb'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            this_object_data['normal'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            this_object_data['id'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            this_object_data['depth'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            this_object_data['depth_exr'] = base64.b64decode(msg.read_string())
        return this_object_data

    def set_action(self, action: str, **kwargs) -> None:
        self.channel.set_action(action, **kwargs)
