from pyrfuniverse.actions import BaseAction
from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np


class SingleGrasp(BaseAction):
    """
    To clasp or embrace an object with the fingers.
    """
    def __init__(
            self,
            env: ToborRobotiq85ManipulationEnv,
            used_arm='left',
    ):
        super().__init__(env)
        self.used_arm = used_arm

        # For heuristic only
        self._heuristic = False
        self._next_action_position = np.array([0, 0, 0])
        self._next_action_orientation = None

    def heuristics(self, position, orientation=None):
        self._heuristic = True
        self._next_action_position = position
        self._next_action_orientation = orientation

    def _check_pre_conditions(self):
        return self.status[self.used_arm + '_hand_empty']

    def _predict_contact_pose(self):
        if self._heuristic:
            self._heuristic = False
            return self._next_action_position, self._next_action_orientation
        return np.array([0, 0, 0]), None

    def _pre_movement(self, position=np.array([0, 0, 0]), orientation=None):
        self.env.step(
            mode=self.used_arm,
            position=position,
            orientation=orientation
        )

    def _gripper_movement(self):
        self.env.close_gripper(mode=self.used_arm)

    def _post_movement(self):
        self.env.step(
            mode=self.used_arm,
            position=self._next_action_position + np.array([0, 0.5, 0]),
            orientation=self._next_action_orientation
        )
        return

    def _effect(self):
        self.synchronize_status()
