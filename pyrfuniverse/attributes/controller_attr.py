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
    机械臂控制器类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:
            self.data['number_of_joints'] 机械臂关节数量

            self.data['positions'] 机械臂节点位置

            self.data['rotations'] 机械臂节点旋转角度

            self.data['quaternion'] 机械臂节点旋转四元数

            self.data['local_positions'] 机械臂节点局部位置

            self.data['local_rotations'] 机械臂节点局部旋转角度

            self.data['local_quaternion'] 机械臂节点局部旋转四元数

            self.data['velocities'] 机械臂节点速度

            self.data['number_of_moveable_joints'] 机械臂活动关节数量

            self.data['joint_positions'] 机械臂关节在自由度空间下位置

            self.data['joint_velocities'] 机械臂关节在自由度空间下速度

            self.data['all_stable'] 机械臂所有关节移动完成

            self.data['move_done'] 机械臂IK移动完成状态

            self.data['rotate_done'] 机械臂IK旋转完成状态

            self.data['gravity_forces'] InverseDynamics抵消重力所需的力

            self.data['coriolis_centrifugal_forces'] InverseDynamics抵消离心力所需的力

            self.data['drive_forces'] InverseDynamics驱动力
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
        设置机械臂所有关节目标位置

        Args:
            joint_positions: 目标位置
            speed_scales: 速度倍数
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
        立即设置机械臂所有关节位置

        Args:
            joint_positions: 目标位置
        """
        num_joints = len(joint_positions)

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointPositionDirectly')
        msg.write_float32_list(joint_positions)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointPosition(self, index: int, joint_position: float):
        """
        设置机械臂指定关节目标位置

        Args:
            index: 关节序号
            joint_position: 目标位置
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointPosition')
        msg.write_int32(index)
        msg.write_float32(joint_position)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointPositionDirectly(self, index: int, joint_position: float):
        """
        立即设置机械臂指定关节位置

        Args:
            index: 关节序号
            joint_position: 目标位置
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointPositionDirectly')
        msg.write_int32(index)
        msg.write_float32(joint_position)

        self.env.instance_channel.send_message(msg)

    def SetJointPositionContinue(self, interval: int, time_joint_positions: list):
        """
        持续设置机械臂所有关节目标位置

        Args:
            interval: 设置时间间隔(毫秒)
            time_joint_positions: 每个time的目标位置

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
        设置机械臂所有关节目标速度

        Args:
            joint_velocitys: 目标速度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetJointVelocity')
        msg.write_float32_list(joint_velocitys)

        self.env.instance_channel.send_message(msg)

    def SetIndexJointVelocity(self, index: int, joint_velocity: float):
        """
        设置机械臂指定关节目标速度

        Args:
            index: 关节序号
            joint_velocity: 目标速度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIndexJointVelocity')
        msg.write_int32(index)
        msg.write_float32(joint_velocity)

        self.env.instance_channel.send_message(msg)

    def AddJointForce(self, joint_forces: list):
        """
        为机械臂所有关节施加力

        Args:
            joint_forces: 力
        """
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
        """
        为机械臂所有关节指定位置施加力

        Args:
            joint_forces: 力
            force_positions: 施力点
        """
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
        """
        为机械臂所有关节施加力矩

        Args:
            joint_torques: 力矩
        """
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
        """
        获取机械臂所有关节逆动力学数据

        Returns:
            调用此接口并step后,从
            self.data['gravity_forces']
            self.data['coriolis_centrifugal_forces']
            self.data['drive_forces']
            获取结果
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetJointInverseDynamicsForce')

        self.env.instance_channel.send_message(msg)

    def SetImmovable(self, immovable: bool):
        """
        设置机械臂是否不可移动

        Args:
            immovable: 是否不可移动
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetImmovable')
        msg.write_bool(immovable)

        self.env.instance_channel.send_message(msg)

    def MoveForward(self, distance: float, speed: float):
        """
        如果机器人在Unity端添加了继承ICustomMove接口的脚本,可通过此接口驱动前进
        最初用于Tobor机器人

        Args:
            distance:距离
            speed:速度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('MoveForward')
        msg.write_float32(distance)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def MoveBack(self, distance: float, speed: float):
        """
        如果机器人在Unity端添加了继承ICustomMove接口的脚本,可通过此接口驱动后退
        最初用于Tobor机器人

        Args:
            distance:距离
            speed:速度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('MoveBack')
        msg.write_float32(distance)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def TurnLeft(self, angle: float, speed: float):
        """
        如果机器人在Unity端添加了继承ICustomMove接口的脚本,可通过此接口驱动左转
        最初用于Tobor机器人

        Args:
            angle:角度
            speed:速度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('TurnLeft')
        msg.write_float32(angle)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def TurnRight(self, angle: float, speed: float):
        """
        如果机器人在Unity端添加了继承ICustomMove接口的脚本,可通过此接口驱动右转
        最初用于Tobor机器人

        Args:
            angle:角度
            speed:速度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('TurnRight')
        msg.write_float32(angle)
        msg.write_float32(speed)

        self.env.instance_channel.send_message(msg)

    def GripperOpen(self):
        """
        如果夹爪在Unity端添加了继承ICustomGripper接口的脚本,可通过此接口驱动张开
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GripperOpen')

        self.env.instance_channel.send_message(msg)

    def GripperClose(self):
        """
        如果夹爪在Unity端添加了继承ICustomGripper接口的脚本,可通过此接口驱动闭合
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GripperClose')

        self.env.instance_channel.send_message(msg)

    def EnabledNativeIK(self, enabled: bool):
        """
        启用/禁用机械臂的原生IK

        Args:
            enabled: 启用/禁用
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('EnabledNativeIK')
        msg.write_bool(enabled)

        self.env.instance_channel.send_message(msg)

    def IKTargetDoMove(self, position: list, duration: float, speed_based: bool = True, relative: bool = False):
        """
        原生IK末端点移动

        Args:
            position: 绝对位置或相对位置
            duration: 移动持续时间或移动速度
            speed_based: 指定duration表示移动持续时间还是移动速度
            relative: 指定position表示绝对位置还是相对位置

        Returns:
            当移动完成时,self.data['move_done']会被置为True
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
        原生IK末端点Vector3旋转

        Args:
            rotation: 绝对旋转或相对旋转
            duration: 旋转持续时间或旋转速度
            speed_based: 指定duration表示旋转持续时间还是旋转速度
            relative: 指定position表示绝对旋转还是相对旋转

        Returns:
            当旋转完成时,self.data['rotate_done']会被置为True
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
        原生IK末端点四元数旋转

        Args:
            quaternion: 绝对旋转或相对旋转
            duration: 旋转持续时间或旋转速度
            speed_based: 指定duration表示旋转持续时间还是旋转速度
            relative: 指定position表示绝对旋转还是相对旋转

        Returns:
            当旋转完成时,self.data['rotate_done']会被置为True
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
        使原生IK末端点移动/旋转立即完成
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoComplete')

        self.env.instance_channel.send_message(msg)

    def IKTargetDoKill(self):
        """
        使原生IK末端点移动/旋转停止
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('IKTargetDoKill')

        self.env.instance_channel.send_message(msg)

    def SetIKTargetOffset(self, position: list = [0.,0.,0.], rotation: list = [0.,0.,0.], quaternion: list = None):
        """
        设置原生IK末端点的偏移

        Args:
            position: 偏移位置
            rotation: 偏移旋转Vector3
            quaternion: 偏移旋转四元数,此值覆盖rotation
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
        等待原生IK末端点移动/旋转完成
        """
        while not self.data['move_done'] or not self.data['rotate_done']:
            self.env.step()