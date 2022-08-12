from pyrfuniverse.actions import BaseAction
from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np


class SingleTransport(BaseAction):
    """
    To transfer or convey an object from one place to another.
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

    def set_wavepoints(self, wave_points: list):
        self._pre_movement_wavepoints = wave_points.copy()

    def _check_pre_conditions(self):
        return True

    def _predict_contact_pose(self):
        if self._heuristic:
            self._heuristic = False
            return self._next_action_position, self._next_action_orientation
        return np.array([0, 0, 0]), None

    def _pre_movement(self, position=np.array([0, 0, 0]), orientation=None):
        for wave_point in self._pre_movement_wavepoints:
            self.env.step(
                mode=self.used_arm,
                position=wave_point,
                orientation=orientation
            )

    def _gripper_movement(self):
        return

    def _post_movement(self):
        return

    def _effect(self):
        self.synchronize_status()
