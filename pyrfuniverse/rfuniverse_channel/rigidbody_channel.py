from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import InstanceChannel


class RigidbodyChannel:

    def __init__(self, channel: InstanceChannel) -> None:
        self.data = {}
        self.channel = channel

    def _parse_message(self, msg: IncomingMessage) -> None:
        id = msg.read_int32()
        this_object_data = {}
        name = msg.read_string()
        if name[-7:] == '(Clone)':
            name = name[:-7]
        this_object_data['name'] = name
        this_object_data['position'] = [msg.read_float32() for i in range(3)]
        this_object_data['rotation'] = [msg.read_float32() for i in range(3)]
        this_object_data['velocity'] = [msg.read_float32() for i in range(3)]
        this_object_data['angular_vel'] = [msg.read_float32() for i in range(3)]
        self.data[id] = this_object_data
        self.channel.data[id] = this_object_data

    def set_action(self, action: str, **kwargs) -> None:
        self.channel.set_action(action, **kwargs)
