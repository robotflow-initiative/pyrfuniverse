import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def HumanIKTargetDoMove(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index', 'position', 'duration']
    optional_params = ['speed_based', 'relative']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('HumanIKTargetDoMove')
    msg.write_int32(kwargs['index'])
    msg.write_float32(kwargs['position'][0])
    msg.write_float32(kwargs['position'][1])
    msg.write_float32(kwargs['position'][2])
    msg.write_float32(kwargs['duration'])
    if 'speed_based' in kwargs:
        msg.write_bool(kwargs['speed_based'])
    else:
        msg.write_bool(True)
    if 'relative' in kwargs:
        msg.write_bool(kwargs['relative'])
    else:
        msg.write_bool(False)
    return msg


def HumanIKTargetDoRotate(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index', 'vector3', 'duration']
    optional_params = ['speed_based', 'relative']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('HumanIKTargetDoRotateQuaternion')
    msg.write_int32(kwargs['index'])
    msg.write_float32(kwargs['vector3'][0])
    msg.write_float32(kwargs['vector3'][1])
    msg.write_float32(kwargs['vector3'][2])
    msg.write_float32(kwargs['duration'])
    if 'speed_based' in kwargs:
        msg.write_bool(kwargs['speed_based'])
    else:
        msg.write_bool(True)
    if 'relative' in kwargs:
        msg.write_bool(kwargs['relative'])
    else:
        msg.write_bool(False)
    return msg

def HumanIKTargetDoRotateQuaternion(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index', 'quaternion', 'duration']
    optional_params = ['speed_based', 'relative']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('HumanIKTargetDoRotateQuaternion')
    msg.write_int32(kwargs['index'])
    msg.write_float32(kwargs['quaternion'][0])
    msg.write_float32(kwargs['quaternion'][1])
    msg.write_float32(kwargs['quaternion'][2])
    msg.write_float32(kwargs['quaternion'][3])
    msg.write_float32(kwargs['duration'])
    if 'speed_based' in kwargs:
        msg.write_bool(kwargs['speed_based'])
    else:
        msg.write_bool(True)
    if 'relative' in kwargs:
        msg.write_bool(kwargs['relative'])
    else:
        msg.write_bool(False)
    return msg


def HumanIKTargetDoComplete(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('HumanIKTargetDoComplete')
    msg.write_int32(kwargs['index'])
    return msg

def HumanIKTargetDoKill(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('HumanIKTargetDoKill')
    msg.write_int32(kwargs['index'])
    return msg


class HumanbodyAttr(attr.BaseAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        self.data['move_done'] = msg.read_bool()
        self.data['rotate_done'] = msg.read_bool()
        return self.data

    def HumanIKTargetDoMove(self, index: int, position: list, duration: float, speed_based: bool = True, relative: bool = False):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoMove')
        msg.write_int32(index)
        msg.write_float32(position[0])
        msg.write_float32(position[1])
        msg.write_float32(position[2])
        msg.write_float32(duration)
        msg.write_bool(speed_based)
        msg.write_bool(relative)

        self.env.instance_channel.send_message(msg)

    def HumanIKTargetDoRotate(self, index: int, rotation: list, duration: float, speed_based: bool = True, relative: bool = False):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoRotate')
        msg.write_int32(index)
        msg.write_float32(rotation[0])
        msg.write_float32(rotation[1])
        msg.write_float32(rotation[2])
        msg.write_float32(duration)
        msg.write_bool(speed_based)
        msg.write_bool(relative)

        self.env.instance_channel.send_message(msg)

    def HumanIKTargetDoRotateQuaternion(self, index: int, quaternion: list, duration: float, speed_based: bool = True, relative: bool = False):
        msg = OutgoingMessage()

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoRotateQuaternion')
        msg.write_int32(index)
        msg.write_float32(quaternion[0])
        msg.write_float32(quaternion[1])
        msg.write_float32(quaternion[2])
        msg.write_float32(quaternion[3])
        msg.write_float32(duration)
        msg.write_bool(speed_based)
        msg.write_bool(relative)

        self.env.instance_channel.send_message(msg)

        self.env.instance_channel.send_message(msg)

    def HumanIKTargetDoComplete(self, index: int):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoComplete')
        msg.write_int32(index)

        self.env.instance_channel.send_message(msg)

    def HumanIKTargetDoKill(self, index: int):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoKill')
        msg.write_int32(index)

        self.env.instance_channel.send_message(msg)

    def WaitDo(self):
        while not self.data['move_done'] or not self.data['rotate_done']:
            self.env.step()

