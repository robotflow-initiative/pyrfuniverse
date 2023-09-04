import numpy as np
import pyrfuniverse.attributes as attr


class ControllerAttr(attr.ColliderAttr):
    """
    Robot controller class, which will control robot arms, hands and embodied robots.
    """

    def parse_message(self, data: dict):
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
        super().parse_message(data)

    def SetJointPosition(self, joint_positions: list, speed_scales: list = None):
        """
        Set the target joint position for each moveable joint and move with PD control.

        Args:
            joint_positions: A list of float, representing the target joint positions.
            speed_scales: A list of float, representing the speed scale.
        """
        if speed_scales is not None:
            assert len(joint_positions) == len(
                speed_scales
            ), "The length of joint_positions and speed_scales are not equal."
            speed_scales = [float(i) for i in speed_scales]
        joint_positions = [float(i) for i in joint_positions]
        self._send_data("SetJointPosition", joint_positions, speed_scales)

    def SetJointPositionDirectly(self, joint_positions: list):
        """
        Set the target joint position for each moveable joint and move directly.

        Args:
            joint_positions: A list of float, representing the target joint positions.
        """
        joint_positions = [float(i) for i in joint_positions]
        self._send_data("SetJointPositionDirectly", joint_positions)

    def SetIndexJointPosition(self, index: int, joint_position: float):
        """
        Set the target joint position for a given joint and move with PD control.

        Args:
            index: Int, joint index.
            joint_position: Float, the target joint position.
        """
        self._send_data("SetIndexJointPosition", index, float(joint_position))

    def SetIndexJointPositionDirectly(self, index: int, joint_position: float):
        """
        Set the target joint position for a given joint and move directly.

        Args:
            index: Int, joint index.
            joint_position: Float, the target joint position.
        """
        self._send_data("SetIndexJointPositionDirectly", index, float(joint_position))

    def SetJointPositionContinue(self, interval: int, time_joint_positions: list):
        """
        Set the target joint position for each moveable joint and move with PD control continuously.

        Args:
            interval: Float, the time interval.
            time_joint_positions: A list of float list, representing the target joint positions at each time step.
        """
        for i in range(len(time_joint_positions)):
            time_joint_positions[i] = [float(j) for j in time_joint_positions[i]]
        self._send_data("SetJointPositionContinue", interval, time_joint_positions)

    def SetJointVelocity(self, joint_velocitys: list):
        """
        Set the target joint velocity for each moveable joint.

        Args:
            joint_velocitys: A list of float, representing the target joint velocities.
        """
        joint_velocitys = [float(i) for i in joint_velocitys]
        self._send_data("SetJointVelocity", joint_velocitys)

    def SetIndexJointVelocity(self, index: int, joint_velocity: float):
        """
        Set the target joint velocity for a given joint.

        Args:
            index: Int, joint index.
            joint_velocity: A list of float, representing the target joint velocities.
        """
        self._send_data("SetIndexJointVelocity", index, float(joint_velocitys))

    def AddJointForce(self, joint_forces: list):
        """
        Add force to each moveable joint.

        Args:
            joint_forces: A list of forces, representing the added forces.
        """
        joint_forces = [float(i) for i in joint_forces]
        self._send_data("AddJointForce", joint_forces)

    def AddJointForceAtPosition(self, joint_forces: list, force_positions: list):
        """
        Add force to each moveable joint at a given position.

        Args:
            joint_forces: A list of forces, representing the added forces.
            force_positions: A list of positions, representing the positions for forces.
        """
        assert len(joint_forces) == len(
            force_positions
        ), "The length of joint_forces and force_positions are not equal."
        joint_forces = [float(i) for i in joint_forces]
        force_positions = [float(i) for i in force_positions]
        self._send_data("AddJointForceAtPosition", joint_forces, force_positions)

    def AddJointTorque(self, joint_torques: list):
        """
        Add torque to each moveable joint.

        Args:
            joint_torques: A list of torques, representing the added torques.
        """
        joint_torques = [float(i) for i in joint_torques]
        self._send_data("AddJointTorque", joint_torques)

    # only work on unity 2022.1+
    def GetJointInverseDynamicsForce(self):
        """
        Get the joint inverse dynamic force of each moveable joint. Note that this function only works in Unity version >= 2022.1.
        """
        self._send_data("GetJointInverseDynamicsForce")

    def SetImmovable(self, immovable: bool):
        """
        Set whether the base of articulation is immovable.

        Args:
            immovable: Bool, True for immovable, False for movable.
        """
        self._send_data("SetImmovable", immovable)

    def MoveForward(self, distance: float, speed: float):
        """
        Move robot forward. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            distance: Float, distance.
            speed: Float, velocity.
        """
        self._send_data("MoveForward", float(distance), float(speed))

    def MoveBack(self, distance: float, speed: float):
        """
        Move robot backword. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            distance: Float, distance.
            speed: Float, velocity.
        """
        self._send_data("MoveBack", float(distance), float(speed))

    def TurnLeft(self, angle: float, speed: float):
        """
        Turn robot left. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            angle: Float, rotation angle.
            speed: Float, velocity.
        """
        self._send_data("TurnLeft", float(angle), float(speed))

    def TurnRight(self, angle: float, speed: float):
        """
        Turn robot right. Only works if the robot controller has implemented functions inherited from `ICustomMove.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomMove.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ToborMove.cs for more details.

        Args:
            angle: Float, rotation angle.
            speed: Float, velocity.
        """
        self._send_data("TurnRight", float(angle), float(speed))

    def GripperOpen(self):
        """
        Open the gripper. Only works if the robot controller has implemented functions inherited from `ICustomGripper.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomGripper.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/GeneralGripper.cs for more details.
        """
        self._send_data("GripperOpen")

    def GripperClose(self):
        """
        Close the gripper. Only works if the robot controller has implemented functions inherited from `ICustomGripper.cs`. See https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/ICustomGripper.cs and https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Utils/GeneralGripper.cs for more details.
        """
        self._send_data("GripperClose")

    def EnabledNativeIK(self, enabled: bool):
        """
        Enable or disable the native IK algorithm.

        Args:
            enabled: Bool, True for enable and False for disable.When it is True, through the IKTatGetDo*** interface, according to the end pose.When it is False, through the SetJoint*** interface, according to the joint movement.NativeIK can only take effect when it is started during initialization.
        """
        self._send_data("EnabledNativeIK", enabled)

    def IKTargetDoMove(
        self,
        position: list,
        duration: float,
        speed_based: bool = True,
        relative: bool = False,
    ):
        """
        Native IK target movement.

        Args:
            position: A list of length 3, representing the position.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `position` is relative; otherwise, `position` is absolute.
        """
        if position is not None:
            assert len(position) == 3, "position length must be 3"
            position = [float(i) for i in position]

        self._send_data(
            "IKTargetDoMove", position, float(duration), speed_based, relative
        )

    def IKTargetDoRotate(
        self,
        rotation: list,
        duration: float,
        speed_based: bool = True,
        relative: bool = False,
    ):
        """
        Native IK target rotation.

        Args:
            rotation: A list of length 3, representing the rotation.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `rotation` is relative; otherwise, `rotation` is absolute.
        """
        if rotation is not None:
            assert len(rotation) == 3, "rotation length must be 3"
            rotation = [float(i) for i in rotation]

        self._send_data(
            "IKTargetDoRotate", rotation, float(duration), speed_based, relative
        )

    def IKTargetDoRotateQuaternion(
        self,
        quaternion: list,
        duration: float,
        speed_based: bool = True,
        relative: bool = False,
    ):
        """
        Native IK target rotation using quaternion.

        Args:
            quaternion: A list of length 4, representing the quaternion.
            duration: Float, if `speed_based` is True, it represents movement duration; otherwise, it represents movement speed.
            speed_based: Bool.
            relative: Bool, if True, `quaternion` is relative; otherwise, `quaternion` is absolute.
        """
        if quaternion is not None:
            assert len(quaternion) == 4, "quaternion length must be 4"
            quaternion = [float(i) for i in quaternion]

        self._send_data(
            "IKTargetDoRotateQuaternion",
            quaternion,
            float(duration),
            speed_based,
            relative,
        )

    def IKTargetDoComplete(self):
        """
        Make native IK target movement / rotation complete directly.
        """
        self._send_data("IKTargetDoComplete")

    def IKTargetDoKill(self):
        """
        Make native IK target movement / rotation stop.
        """
        self._send_data("IKTargetDoKill")

    def SetIKTargetOffset(
        self,
        position: list = [0.0, 0.0, 0.0],
        rotation: list = [0.0, 0.0, 0.0],
        quaternion: list = None,
    ):
        """
        Set the new IK target by setting offset to the original target of native IK.

        Args:
            position: A list of length 3, representing the position offset to original target.
            rotation: A list of length 3, representing the rotation offset to original target.
            quaternion: A list of length 4, representing the quaternion offset to original target. If this parameter is specified, `rotation` will be ignored.
        """
        if position is not None:
            assert len(position) == 3, "position length must be 3"
            position = [float(i) for i in position]
        if rotation is not None:
            assert len(rotation) == 3, "rotation length must be 3"
            rotation = [float(i) for i in rotation]
        if quaternion is not None:
            assert len(quaternion) == 4, "quaternion length must be 4"
            quaternion = [float(i) for i in quaternion]

        self._send_data("SetIKTargetOffset", position, rotation, quaternion)

    def WaitDo(self):
        """
        Wait for the native IK target movement / rotation complete.
        """
        while not self.data["move_done"] or not self.data["rotate_done"]:
            self.env._step()
