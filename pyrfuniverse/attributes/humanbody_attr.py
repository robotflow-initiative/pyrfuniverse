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
    """
    Human body Inverse Kinematic class.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['move_done']: Whether the movement has finished.

            self.data['rotate_done']: Whether the rotation has finished.
        """
        super().parse_message(msg)
        self.data['move_done'] = msg.read_bool()
        self.data['rotate_done'] = msg.read_bool()
        return self.data

    def HumanIKTargetDoMove(self, index: int, position: list, duration: float, speed_based: bool = True, relative: bool = False):
        """
        Human body Inverse Kinematics target movement.

        Args:
            index: Int, the target for movement. 0 for left hand, 1 for right hand,2 for left foot, 3 for right foot, 4 for head.
            position: A list of length 3, representing the position.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `position` is relative; otherwise, `position` is absolute.
        """
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
        """
        Human body Inverse Kinematics target rotation.

        Args:
            index: Int, the target for movement. 0 for left hand, 1 for right hand,2 for left foot, 3 for right foot, 4 for head.
            rotation: A list of length 3, representing the rotation.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `rotation` is relative; otherwise, `rotation` is absolute.
        """
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
        """
        Human body Inverse Kinematics target rotation using quaternion.

        Args:
            index: Int, the target for movement. 0 for left hand, 1 for right hand,2 for left foot, 3 for right foot, 4 for head.
            quaternion: A list of length 4, representing the quaternion.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `quaternion` is relative; otherwise, `quaternion` is absolute.
        """
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
        """
        Make the human body IK target movement / rotation complete directly.

        Args:
            index: Int, the target for movement. 0 for left hand, 1 for right hand,2 for left foot, 3 for right foot, 4 for head.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoComplete')
        msg.write_int32(index)

        self.env.instance_channel.send_message(msg)

    def HumanIKTargetDoKill(self, index: int):
        """
        Make the human body IK target movement / rotation stop.

        Args:
            index: Int, the target for movement. 0 for left hand, 1 for right hand,2 for left foot, 3 for right foot, 4 for head.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('HumanIKTargetDoKill')
        msg.write_int32(index)

        self.env.instance_channel.send_message(msg)

    def WaitDo(self):
        """
        Wait for the human body IK target movement / rotation complete.
        """
        while not self.data['move_done'] or not self.data['rotate_done']:
            self.env.step()

