from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import InstanceChannel


class ObiClothWithGraspingChannel:

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

        # Grasp point position
        this_object_data['grasp_position'] = [msg.read_float32() for i in range(3)]
        # Grasp point rotation
        this_object_data['grasp_rotation'] = [msg.read_float32() for i in range(3)]
        # Grasp point velocity
        this_object_data['grasp_velocity'] = [msg.read_float32() for i in range(3)]
        # Grasp point angular velocity
        this_object_data['grasp_angular_vel'] = [msg.read_float32() for i in range(3)]

        # Target position
        this_object_data['target_position'] = [msg.read_float32() for i in range(3)]
        # Target rotation
        this_object_data['target_rotation'] = [msg.read_float32() for i in range(3)]

        self.data[id] = this_object_data
        self.channel.data[id] = this_object_data

    def _parse_raw_list(self, raw_list):
        length = len(raw_list)
        assert length % 3 == 0
        number_of_parts = length // 3
        norm_list = []
        for j in range(number_of_parts):
            transform = [raw_list[3 * j], raw_list[3 * j + 1], raw_list[3 * j + 2]]
            norm_list.append(transform)

        return norm_list

    def set_action(self, action: str, **kwargs) -> None:
        self.channel.set_action(action, **kwargs)
