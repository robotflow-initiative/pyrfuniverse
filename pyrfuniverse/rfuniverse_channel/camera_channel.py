from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import InstanceChannel
import base64
from io import BytesIO
from PIL import Image
from enum import Enum
import time
import numpy as np


class RenderingMode(Enum):
    RGB = -1
    MASK = 0
    DEPTH = 2
    NORMALS = 4
    OPTICAL_FLOW = 5


class CameraChannel:

    def __init__(self, channel: InstanceChannel) -> None:
        self.data = {}
        self.channel = channel

    def _parse_message(self, msg: IncomingMessage) -> None:
        id = msg.read_int32()
        this_object_data = {}
        this_object_data['position'] = [msg.read_float32() for _ in range(3)]
        this_object_data['rotation'] = [msg.read_float32() for _ in range(3)]
        # this_object_data['rendering_mode'] = RenderingMode(msg.read_int32())
        this_object_data['near_plane'] = msg.read_float32()
        this_object_data['far_plane'] = msg.read_float32()
        this_object_data['FOV'] = msg.read_float32()
        this_object_data['target_display'] = msg.read_int32()
        self.data[id] = this_object_data
        self.channel.data[id] = this_object_data

    def set_action(self, action: str, **kwargs) -> None:
        self.channel.set_action(action, **kwargs)
