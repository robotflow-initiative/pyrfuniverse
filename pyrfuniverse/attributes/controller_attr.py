import numpy as np

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility



def SetJointPosition(kwargs: dict) -> OutgoingMessage:
    """Set the target positions for each joint in a specified articulation body.
    Args:
        Compulsory:
        index: The index of articulation body, specified in returned message.
        joint_positions: A list inferring each joint's position in the specified acticulation body.

        Optional:
        speed_scales: A list inferring each joint's speed scale. The length must be the same with joint_positions.
    """
    compulsory_params = ['id', 'joint_positions']
    optional_params = ['speed_scales']
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    joint_positions = kwargs['joint_positions']
    num_joints = len(joint_positions)

    msg.write_int32(kwargs['id'])
    msg.write_string('SetJointPosition')
    msg.write_float32_list(kwargs['joint_positions'])
    if 'speed_scales' in kwargs.keys():
        assert num_joints == len(kwargs['speed_scales']), \
            'The length of joint_positions and speed_scales are not equal.'
        msg.write_float32_list(kwargs['speed_scales'])
    else:
        msg.write_float32_list([1.0 for i in range(num_joints)])

    return msg


def SetJointPositionDirectly(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'joint_positions']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    joint_positions = kwargs['joint_positions']
    num_joints = len(joint_positions)

    msg.write_int32(kwargs['id'])
    msg.write_string('SetJointPositionDirectly')
    msg.write_float32_list(kwargs['joint_positions'])

    return msg

def SetIndexJointPosition(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index','joint_position']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetIndexJointPosition')
    msg.write_int32(kwargs['index'])
    msg.write_float32(kwargs['joint_position'])

    return msg

def SetIndexJointPositionDirectly(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index','joint_position']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetIndexJointPositionDirectly')
    msg.write_int32(kwargs['index'])
    msg.write_float32(kwargs['joint_position'])

    return msg

def SetJointPositionContinue(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'interval', 'time_joint_positions']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()

    time_joint_positions = kwargs['time_joint_positions']
    num_times = len(time_joint_positions)
    # num_joints = len(time_joint_positions[0])
    interval = kwargs['interval']

    msg.write_int32(kwargs['id'])
    msg.write_string('SetJointPositionContinue')
    msg.write_int32(num_times)
    # msg.write_int32(num_joints)
    msg.write_int32(interval)
    for i in range(num_times):
        msg.write_float32_list(time_joint_positions[i])

    return msg


def SetJointVelocity(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'joint_velocitys']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    joint_velocitys = kwargs['joint_velocitys']
    num_joints = len(joint_velocitys)

    msg.write_int32(kwargs['id'])
    msg.write_string('SetJointVelocity')
    msg.write_float32_list(kwargs['joint_velocitys'])

    return msg


def AddJointForce(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'joint_forces']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    joint_positions = kwargs['joint_positions']
    num_joints = len(joint_positions)

    msg.write_int32(kwargs['id'])
    msg.write_string('AddJointForce')
    msg.write_int32(num_joints)
    for i in range(num_joints):
        msg.write_float32(joint_positions[i][0])
        msg.write_float32(joint_positions[i][1])
        msg.write_float32(joint_positions[i][2])

    return msg


def AddJointForceAtPosition(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'joint_forces', 'forces_position']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    joint_positions = kwargs['joint_forces']
    forces_position = kwargs['forces_position']
    num_joints = len(joint_positions)

    msg.write_int32(kwargs['id'])
    msg.write_string('AddJointForceAtPosition')
    msg.write_int32(num_joints)
    for i in range(num_joints):
        msg.write_float32(joint_positions[i][0])
        msg.write_float32(joint_positions[i][1])
        msg.write_float32(joint_positions[i][2])
        msg.write_float32(forces_position[i][0])
        msg.write_float32(forces_position[i][1])
        msg.write_float32(forces_position[i][2])

    return msg


def AddJointTorque(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'joint_torque']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    joint_torque = kwargs['joint_torque']
    num_joints = len(joint_torque)

    msg.write_int32(kwargs['id'])
    msg.write_string('AddJointTorque')
    msg.write_int32(num_joints)
    for i in range(num_joints):
        msg.write_float32(joint_torque[i][0])
        msg.write_float32(joint_torque[i][1])
        msg.write_float32(joint_torque[i][2])

    return msg


# only work on unity 2022.1+
def GetJointInverseDynamicsForce(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetJointInverseDynamicsForce')
    return msg


def SetImmovable(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'immovable']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetImmovable')
    msg.write_bool(kwargs['immovable'])
    return msg


def MoveForward(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'distance', 'speed']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('MoveForward')
    msg.write_float32(kwargs['distance'])
    msg.write_float32(kwargs['speed'])
    return msg


def MoveBack(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'distance', 'speed']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('MoveBack')
    msg.write_float32(kwargs['distance'])
    msg.write_float32(kwargs['speed'])
    return msg


def TurnLeft(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'angle', 'speed']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('TurnLeft')
    msg.write_float32(kwargs['angle'])
    msg.write_float32(kwargs['speed'])
    return msg


def TurnRight(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'angle', 'speed']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('TurnRight')
    msg.write_float32(kwargs['angle'])
    msg.write_float32(kwargs['speed'])
    return msg

def GripperOpen(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GripperOpen')
    return msg

def GripperClose(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GripperClose')
    return msg


def EnabledNativeIK(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'enabled']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('EnabledNativeIK')
    msg.write_bool(kwargs['enabled'])
    return msg


def IKTargetDoMove(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'position', 'duration']
    optional_params = ['speed_based', 'relative']
    utility.CheckKwargs(kwargs, compulsory_params)

    if 'speed_based' not in kwargs:
        kwargs['speed_based'] = True
    if 'relative' not in kwargs:
        kwargs['relative'] = False

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('IKTargetDoMove')
    msg.write_float32(kwargs['position'][0])
    msg.write_float32(kwargs['position'][1])
    msg.write_float32(kwargs['position'][2])
    msg.write_float32(kwargs['duration'])
    msg.write_bool(kwargs['speed_based'])
    msg.write_bool(kwargs['relative'])
    return msg

def IKTargetDoRotate(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'vector3', 'duration']
    optional_params = ['speed_based', 'relative']
    utility.CheckKwargs(kwargs, compulsory_params)

    if 'speed_based' not in kwargs:
        kwargs['speed_based'] = True
    if 'relative' not in kwargs:
        kwargs['relative'] = False

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('IKTargetDoRotate')
    msg.write_float32(kwargs['vector3'][0])
    msg.write_float32(kwargs['vector3'][1])
    msg.write_float32(kwargs['vector3'][2])
    msg.write_float32(kwargs['duration'])
    msg.write_bool(kwargs['speed_based'])
    msg.write_bool(kwargs['relative'])
    return msg

def IKTargetDoRotateQuaternion(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'quaternion', 'duration']
    optional_params = ['speed_based', 'relative']
    utility.CheckKwargs(kwargs, compulsory_params)

    if 'speed_based' not in kwargs:
        kwargs['speed_based'] = True
    if 'relative' not in kwargs:
        kwargs['relative'] = False

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('IKTargetDoRotateQuaternion')
    msg.write_float32(kwargs['quaternion'][0])
    msg.write_float32(kwargs['quaternion'][1])
    msg.write_float32(kwargs['quaternion'][2])
    msg.write_float32(kwargs['quaternion'][3])
    msg.write_float32(kwargs['duration'])
    msg.write_bool(kwargs['speed_based'])
    msg.write_bool(kwargs['relative'])
    return msg


def IKTargetDoComplete(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('IKTargetDoComplete')
    return msg


def IKTargetDoKill(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('IKTargetDoKill')
    return msg

def SetIKTargetOffset(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = ['position', 'rotation', 'is_quaternion']
    utility.CheckKwargs(kwargs, compulsory_params)

    if 'position' not in kwargs:
        kwargs['position'] = [0,0,0]
    if 'rotation' not in kwargs:
        kwargs['rotation'] = [0,0,0,0]
    if 'is_quaternion' not in kwargs:
        kwargs['is_quaternion'] = False

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetIKTargetOffset')
    msg.write_float32(kwargs['position'][0])
    msg.write_float32(kwargs['position'][1])
    msg.write_float32(kwargs['position'][2])
    msg.write_bool(kwargs['is_quaternion'])
    if kwargs['is_quaternion']:
        msg.write_float32(kwargs['rotation'][0])
        msg.write_float32(kwargs['rotation'][1])
        msg.write_float32(kwargs['rotation'][2])
        msg.write_float32(kwargs['rotation'][2])
    else:
        msg.write_float32(kwargs['rotation'][0])
        msg.write_float32(kwargs['rotation'][1])
        msg.write_float32(kwargs['rotation'][2])
    return msg


class ControllerAttr(attr.ColliderAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        self.data['number_of_joints'] = msg.read_int32()
        # Position
        self.data['positions'] = np.array(msg.read_float32_list()).reshape([-1, 3]).tolist()
        # RotationEuler
        self.data['rotations'] = np.array(msg.read_float32_list()).reshape([-1, 3]).tolist()
        # RotationQuaternion
        self.data['quaternion'] = np.array(msg.read_float32_list()).reshape([-1, 4]).tolist()
        # LocalPosition
        self.data['local_positions'] = np.array(msg.read_float32_list()).reshape([-1, 3]).tolist()
        # LocalRotationEuler
        self.data['local_rotations'] = np.array(msg.read_float32_list()).reshape([-1, 3]).tolist()
        # LocalRotationQuaternion
        self.data['local_quaternion'] = np.array(msg.read_float32_list()).reshape([-1, 4]).tolist()
        # Velocity
        self.data['velocities'] = np.array(msg.read_float32_list()).reshape([-1, 3]).tolist()
        #
        self.data['number_of_moveable_joints'] = msg.read_int32()
        # Each joint position
        self.data['joint_positions'] = msg.read_float32_list()
        # Each joint velocity
        self.data['joint_velocities'] = msg.read_float32_list()
        # Whether all parts are stable
        self.data['all_stable'] = msg.read_bool()
        self.data['move_done'] = msg.read_bool()
        self.data['rotate_done'] = msg.read_bool()
        if msg.read_bool() is True:
            self.data['gravity_forces'] = msg.read_float32_list()
            self.data['coriolis_centrifugal_forces'] = msg.read_float32_list()
            self.data['drive_forces'] = msg.read_float32_list()
        return self.data

    def SetJointPosition(self, joint_positions: list, speed_scales = None):
        num_joints = len(joint_positions)
        if speed_scales is None:
            speed_scales = [1.0 for i in range(num_joints)]
        assert num_joints == len(speed_scales), \
            'The length of joint_positions and speed_scales are not equal.'

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointPosition')
        msg.write_float32_list(joint_positions)
        msg.write_float32_list(speed_scales)

        self.env.instance_channel.send_message(msg)

    def SetJointPositionDirectly(self, joint_positions: list):
        num_joints = len(joint_positions)

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointPositionDirectly')
        msg.write_float32_list(joint_positions)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointPosition(self, index: int, joint_position: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointPosition')
        msg.write_int32(index)
        msg.write_float32(joint_position)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointPositionDirectly(self, index: int, joint_position: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointPositionDirectly')
        msg.write_int32(index)
        msg.write_float32(joint_position)

        self.env.instance_channel.send_message(msg)

    def SetJointPositionContinue(self, interval: int, time_joint_positions: list):
        num_times = len(time_joint_positions)

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointPositionContinue')
        msg.write_int32(num_times)
        msg.write_int32(interval)
        for i in range(num_times):
            msg.write_float32_list(time_joint_positions[i])

        self.env.instance_channel.send_message(msg)

    def SetJointVelocity(self, joint_velocitys: list):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointVelocity')
        msg.write_float32_list(joint_velocitys)

        self.env.instance_channel.send_message(msg)

    def AddJointForce(self, joint_forces: list):
        num_joints = len(joint_positions)

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AddJointForce')
        msg.write_int32(num_joints)
        for i in range(num_joints):
            msg.write_float32(joint_forces[i][0])
            msg.write_float32(joint_forces[i][1])
            msg.write_float32(joint_forces[i][2])

        self.env.instance_channel.send_message(msg)

    def AddJointForceAtPosition(self, joint_forces: list, force_positions: list):
        num_joints = len(joint_positions)
        assert len(joint_forces) == len(force_positions), \
            'The length of joint_forces and force_positions are not equal.'

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AddJointForceAtPosition')
        msg.write_int32(num_joints)
        for i in range(num_joints):
            msg.write_float32(joint_forces[i][0])
            msg.write_float32(joint_forces[i][1])
            msg.write_float32(joint_forces[i][2])
            msg.write_float32(force_positions[i][0])
            msg.write_float32(force_positions[i][1])
            msg.write_float32(force_positions[i][2])

        self.env.instance_channel.send_message(msg)

    def AddJointTorque(self, joint_torques: list):
        num_joints = len(joint_torque)

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AddJointTorque')
        msg.write_int32(num_joints)
        for i in range(num_joints):
            msg.write_float32(joint_torques[i][0])
            msg.write_float32(joint_torques[i][1])
            msg.write_float32(joint_torques[i][2])

        self.env.instance_channel.send_message(msg)

    # only work on unity 2022.1+
    def GetJointInverseDynamicsForce(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetJointInverseDynamicsForce')

        self.env.instance_channel.send_message(msg)

    def SetImmovable(self, immovable: bool):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetImmovable')
        msg.write_bool(immovable)

        self.env.instance_channel.send_message(msg)

    def MoveForward(self, distance: float, speed: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('MoveForward')
        msg.write_float32(distance)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def MoveBack(self, distance: float, speed: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('MoveBack')
        msg.write_float32(distance)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def TurnLeft(self, angle: float, speed: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('TurnLeft')
        msg.write_float32(angle)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def TurnRight(self, angle: float, speed: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('TurnRight')
        msg.write_float32(angle)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def GripperOpen(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GripperOpen')

        self.env.instance_channel.send_message(msg)

    def GripperClose(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GripperClose')

        self.env.instance_channel.send_message(msg)

    def EnabledNativeIK(self, enabled: bool):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('EnabledNativeIK')
        msg.write_bool(enabled)

        self.env.instance_channel.send_message(msg)

    def IKTargetDoMove(self, position: list, duration: float, speed_based: bool = True, relative: bool = False):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoMove')
        msg.write_float32(position[0])
        msg.write_float32(position[1])
        msg.write_float32(position[2])
        msg.write_float32(duration)
        msg.write_bool(speed_based)
        msg.write_bool(relative)

        self.env.instance_channel.send_message(msg)

    def IKTargetDoRotate(self, rotation: list, duration: float, speed_based: bool = True, relative: bool = False):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoRotate')
        msg.write_float32(rotation[0])
        msg.write_float32(rotation[1])
        msg.write_float32(rotation[2])
        msg.write_float32(duration)
        msg.write_bool(speed_based)
        msg.write_bool(relative)

        self.env.instance_channel.send_message(msg)

    def IKTargetDoRotateQuaternion(self, quaternion: list, duration: float, speed_based: bool = True, relative: bool = False):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoRotateQuaternion')
        msg.write_float32(quaternion[0])
        msg.write_float32(quaternion[1])
        msg.write_float32(quaternion[2])
        msg.write_float32(quaternion[3])
        msg.write_float32(duration)
        msg.write_bool(speed_based)
        msg.write_bool(relative)

        self.env.instance_channel.send_message(msg)

    def IKTargetDoComplete(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoComplete')

        self.env.instance_channel.send_message(msg)

    def IKTargetDoKill(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoKill')

        self.env.instance_channel.send_message(msg)

    def SetIKTargetOffset(self, position: list = [0.,0.,0.], rotation: list = [0.,0.,0.], quaternion: list = None):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIKTargetOffset')
        msg.write_float32(position[0])
        msg.write_float32(position[1])
        msg.write_float32(position[2])
        msg.write_bool(quaternion is not None)
        if quaternion is not None:
            msg.write_float32(quaternion[0])
            msg.write_float32(quaternion[1])
            msg.write_float32(quaternion[2])
            msg.write_float32(quaternion[2])
        else:
            msg.write_float32(rotation[0])
            msg.write_float32(rotation[1])
            msg.write_float32(rotation[2])

        self.env.instance_channel.send_message(msg)

    def WaitDo(self):
        while not self.data['move_done'] or not self.data['rotate_done']:
            self.env.step()