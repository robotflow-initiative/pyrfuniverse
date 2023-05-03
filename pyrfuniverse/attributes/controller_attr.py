import numpy as np

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility



def SetJointPosition(kwargs: dict) -> OutgoingMessage:
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


def SetIndexJointVelocity(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'index', 'joint_velocity']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetIndexJointVelocity')
    msg.write_int32(kwargs['index'])
    msg.write_float32(kwargs['joint_velocity'])

    self.env.instance_channel.send_message(msg)

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
    """
    Robot controller class, which will control robot arms, hands and embodied robots.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['number_of_joints']: The number of joints in an articulation.

            self.data['positions']: The position of each part in an articulation.

            self.data['rotations']: The rotation of each part in an articulation.

            self.data['quaternion']: The quaternion of each part in an articulation.

            self.data['local_positions']: The local position of each part in an articulation.

            self.data['local_rotations']: The local rotation of each part in an articulation.

            self.data['local_quaternion']: The local quaternion of each part in an articulation.

            self.data['velocities']: The velocity of each part in an articulation.

            self.data['number_of_moveable_joints']: The number of moveable joints in an articulation.

            self.data['joint_positions']: The joint position of each moveable joint in an articulation.

            self.data['joint_velocities']: The joint velocity of each moveable joint in an articulation.

            self.data['all_stable']: Whether all joints have finished moving.

            self.data['move_done']: Whether robot arm IK has finished moving.

            self.data['rotate_done']: Whether robot arm IK has finished rotating.

            self.data['gravity_forces']: Inverse Dynamics force needed to counteract gravity.

            self.data['coriolis_centrifugal_forces']: Inverse Dynamics force needed to counteract coriolis centrifugal forces.

            self.data['drive_forces']: Inverse Dynamics drive forces.
        """
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
        """
        Set the target joint position for each moveable joint and move with PD control.

        Args:
            joint_positions: A list of float, representing the target joint positions.
            speed_scales: A list of float, representing the speed scale.
        """
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
        """
        Set the target joint position for each moveable joint and move directly.

        Args:
            joint_positions: A list of float, representing the target joint positions.
        """
        num_joints = len(joint_positions)

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointPositionDirectly')
        msg.write_float32_list(joint_positions)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointPosition(self, index: int, joint_position: float):
        """
        Set the target joint position for a given joint and move with PD control.

        Args:
            index: Int, joint index.
            joint_position: Float, the target joint position.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointPosition')
        msg.write_int32(index)
        msg.write_float32(joint_position)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointPositionDirectly(self, index: int, joint_position: float):
        """
        Set the target joint position for a given joint and move directly.

        Args:
            index: Int, joint index.
            joint_position: Float, the target joint position.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointPositionDirectly')
        msg.write_int32(index)
        msg.write_float32(joint_position)

        self.env.instance_channel.send_message(msg)

    def SetJointPositionContinue(self, interval: int, time_joint_positions: list):
        """
        Set the target joint position for each moveable joint and move with PD control continuously.

        Args:
            interval: Float, the time interval.
            time_joint_positions: A list of float list, representing the target joint positions at each time step.

        Returns:

        """
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
        """
        Set the target joint velocity for each moveable joint.

        Args:
            joint_velocitys: A list of float, representing the target joint velocities.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointVelocity')
        msg.write_float32_list(joint_velocitys)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointVelocity(self, index: int, joint_velocity: float):
        """
        Set the target joint velocity for a given joint.

        Args:
            index: Int, joint index.
            joint_velocity: A list of float, representing the target joint velocities.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointVelocity')
        msg.write_int32(index)
        msg.write_float32(joint_velocity)

        self.env.instance_channel.send_message(msg)

    def AddJointForce(self, joint_forces: list):
        """
        Add force to each moveable joint.

        Args:
            joint_forces: A list of forces, representing the added forces.
        """
        num_joints = len(joint_forces)

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
        """
        Add force to each moveable joint at a given position.

        Args:
            joint_forces: A list of forces, representing the added forces.
            force_positions: A list of positions, representing the positions for forces.
        """
        num_joints = len(joint_forces)
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
        """
        Add torque to each moveable joint.

        Args:
            joint_torques: A list of torques, representing the added torques.
        """
        num_joints = len(joint_torques)

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
        """
        Get the joint inverse dynamic force of each moveable joint. Note that this function only works in Unity version >= 2022.1.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetJointInverseDynamicsForce')

        self.env.instance_channel.send_message(msg)

    def SetImmovable(self, immovable: bool):
        """
        Set whether the base of articulation is immovable.

        Args:
            immovable: Bool, True for immovable, False for movable.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetImmovable')
        msg.write_bool(immovable)

        self.env.instance_channel.send_message(msg)

    def MoveForward(self, distance: float, speed: float):
        """
        Move robot forward. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            distance: Float, distance.
            speed: Float, velocity.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('MoveForward')
        msg.write_float32(distance)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def MoveBack(self, distance: float, speed: float):
        """
        Move robot backword. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            distance: Float, distance.
            speed: Float, velocity.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('MoveBack')
        msg.write_float32(distance)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def TurnLeft(self, angle: float, speed: float):
        """
        Turn robot left. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            angle: Float, rotation angle.
            speed: Float, velocity.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('TurnLeft')
        msg.write_float32(angle)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def TurnRight(self, angle: float, speed: float):
        """
        Turn robot right. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            angle: Float, rotation angle.
            speed: Float, velocity.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('TurnRight')
        msg.write_float32(angle)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def GripperOpen(self):
        """
        Open the gripper. Only works if the robot controller has implemented functions inherited from `ICustomGripper.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomGripper.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/GeneralGripper.cs for more details.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GripperOpen')

        self.env.instance_channel.send_message(msg)

    def GripperClose(self):
        """
        Close the gripper. Only works if the robot controller has implemented functions inherited from `ICustomGripper.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomGripper.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/GeneralGripper.cs for more details.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GripperClose')

        self.env.instance_channel.send_message(msg)

    def EnabledNativeIK(self, enabled: bool):
        """
        Enable or disable the native IK algorithm.

        Args:
            enabled: Bool, True for enable and False for disable.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('EnabledNativeIK')
        msg.write_bool(enabled)

        self.env.instance_channel.send_message(msg)

    def IKTargetDoMove(self, position: list, duration: float, speed_based: bool = True, relative: bool = False):
        """
        Native IK target movement.

        Args:
            position: A list of length 3, representing the position.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `position` is relative; otherwise, `position` is absolute.
        """
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
        """
        Native IK target rotation.

        Args:
            rotation: A list of length 3, representing the rotation.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `rotation` is relative; otherwise, `rotation` is absolute.
        """
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
        """
        Native IK target rotation using quaternion.

        Args:
            quaternion: A list of length 4, representing the quaternion.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `quaternion` is relative; otherwise, `quaternion` is absolute.
        """
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
        """
        Make native IK target movement / rotation complete directly.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoComplete')

        self.env.instance_channel.send_message(msg)

    def IKTargetDoKill(self):
        """
        Make native IK target movement / rotation stop.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoKill')

        self.env.instance_channel.send_message(msg)

    def SetIKTargetOffset(self, position: list = [0.,0.,0.], rotation: list = [0.,0.,0.], quaternion: list = None):
        """
        Set the new IK target by setting offset to the original target of native IK.

        Args:
            position: A list of length 3, representing the position offset to original target.
            rotation: A list of length 3, representing the rotation offset to original target.
            quaternion: A list of length 4, representing the quaternion offset to original target. If this parameter is specified, `rotation` will be ignored.
        """
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
        """
        Wait for the native IK target movement / rotation complete.
        """
        while not self.data['move_done'] or not self.data['rotate_done']:
            self.env.step()