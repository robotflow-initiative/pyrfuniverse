from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import InstanceChannel
import time


class ArticulationChannel:

    def __init__(self, channel: InstanceChannel) -> None:
        self.data = {}
        self.channel = channel

    def _parse_message(self, msg: IncomingMessage) -> None:
        this_id = msg.read_int32()
        this_object_data = {}
        name = msg.read_string()
        if name[-7:] == '(Clone)':
            name = name[:-7]
        this_object_data['name'] = name
        # Position
        raw_positions = msg.read_float32_list()
        this_object_data['positions'] = self._parse_raw_list_3(raw_positions)
        # RotationEuler
        raw_rotations = msg.read_float32_list()
        this_object_data['rotations'] = self._parse_raw_list_3(raw_rotations)
        # RotationQuaternion
        raw_rotations = msg.read_float32_list()
        this_object_data['rotations_quaternion'] = self._parse_raw_list_4(raw_rotations)
        # Velocity
        raw_velocities = msg.read_float32_list()
        this_object_data['velocities'] = self._parse_raw_list_3(raw_velocities)
        # rasp_point_position
        this_object_data['grasp_point_position'] = msg.read_float32_list()
        # grasp_point_rotation
        this_object_data['grasp_point_rotation'] = msg.read_float32_list()
        # grasp_point_rotation_quaternion
        this_object_data['grasp_point_rotation_quaternion'] = msg.read_float32_list()
        # Number of joints
        this_object_data['number_of_joints'] = msg.read_int32()
        # Each joint position
        this_object_data['joint_positions'] = msg.read_float32_list()
        # Each joint velocity
        this_object_data['joint_velocities'] = msg.read_float32_list()
        # Whether all parts are stable
        this_object_data['all_stable'] = msg.read_bool()

        #################################
        # Upgrade to 2022.1.0a106
        # Each joint force
        # this_object_data['drive_forces'] = msg.read_float32_list()
        #################################

        self.data[this_id] = this_object_data
        self.channel.data[this_id] = this_object_data


    def _parse_raw_list_3(self, raw_list):
        length = len(raw_list)
        assert length % 3 == 0
        number_of_parts = length // 3
        norm_list = []
        for j in range(number_of_parts):
            transform = [raw_list[3 * j], raw_list[3 * j + 1], raw_list[3 * j + 2]]
            norm_list.append(transform)

        return norm_list

    def _parse_raw_list_4(self, raw_list):
        length = len(raw_list)
        assert length % 4 == 0
        number_of_parts = length // 4
        norm_list = []
        for j in range(number_of_parts):
            transform = [raw_list[4 * j], raw_list[4 * j + 1], raw_list[4 * j + 2], raw_list[4 * j + 3]]
            norm_list.append(transform)

        return norm_list

    def set_action(self, action: str, **kwargs) -> None:
        self.channel.set_action(action, **kwargs)
