import numpy as np
from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
from abc import ABC
from abc import abstractmethod


class BaseAction(ABC):
    """
    Currently, we only consider the action on Tobor robot with left and right end effector equipped with Robotiq85.
    This is because pyrfuniverse has implemented Inverse Kinematics controller on Tobor's both arm, with gripper
    operation of Robotiq85.
    For other end effectors or robots, user can implement their API in the same way with Tobor-Robotiq85's.
    """
    def __init__(self, env: ToborRobotiq85ManipulationEnv):
        self.env = env
        self.status = {}
        self.synchronize_status()

    def synchronize_status(self):
        """
        Update status according to current environment.
        """
        self.status['left_hand_empty'] = self.env.left_gripper_open
        self.status['left_joint_positions'] = self.env.left_joint_positions

        self.status['right_hand_empty'] = self.env.right_gripper_open
        self.status['right_joint_positions'] = self.env.right_joint_positions

    @abstractmethod
    def _check_pre_conditions(self):
        """
        Check whether current status satisfy the pre-conditions of this action.
        Returns:
            is_satisfied: bool
        """
        raise NotImplementedError

    @abstractmethod
    def _predict_contact_pose(self):
        """
        Predict the contact pose of gripper from a frame in environment.
        Returns:
            position: The position of end-effector in world coordinate.
            orientation: (Optional) The orientation of end-effector in world coordinate.
        """
        raise NotImplementedError

    @abstractmethod
    def _pre_movement(self, position=np.array([0, 0, 0]), orientation=None):
        """
        Given the pose of end effector when robot contacts with the object, robot will execute.
        Args:
            position: The position of end-effector in world coordinate.
            orientation: (Optional) The orientation of end-effector in world coordinate.

        Returns:
            None
        """
        raise NotImplementedError

    @abstractmethod
    def _gripper_movement(self):
        """
        After end effector reaches contact pose, gripper may move in this function.
        Returns:
            None
        """
        raise NotImplementedError

    @abstractmethod
    def _post_movement(self):
        """
        After gripper movement, robot may execute post movement to finally finish this action.
        Returns:
            None
        """
        raise NotImplementedError

    @abstractmethod
    def _effect(self):
        """
        After all execution, robot should check its success or failure and change its current status.
        Returns:
            success: bool
        """
        raise NotImplementedError

    def _pre_movement_constraints(self):
        """
        Preset constraints before pre-movement.
        Returns:
            None
        """
        pass

    def _gripper_movement_constraints(self):
        """
        Preset constraints before gripper movement.
        Returns:
            None
        """
        pass

    def _post_movement_constraints(self):
        """
        Preset constraints before post-movement.
        Returns:
            None
        """
        pass

    def execute(self):
        """
        Execute auxiliary functions sequentially and return success.
        Returns:
            success: bool
        """
        self.synchronize_status()
        is_satisfied = self._check_pre_conditions()
        if not is_satisfied:
            print('Pre-conditions not satisfied.')
            return False

        position, orientation = self._predict_contact_pose()
        self._pre_movement_constraints()
        self._pre_movement(position, orientation)

        self._gripper_movement_constraints()
        self._gripper_movement()

        self._post_movement_constraints()
        self._post_movement()
        success = self._effect()

        return success
