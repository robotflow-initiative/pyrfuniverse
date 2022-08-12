from pyrfuniverse.actions import BaseAction
from pyrfuniverse.envs import ToborRobotiq85ManipulationEnv
import numpy as np


class SinglePlace(BaseAction):
    """
    To put in or as if in a particular place or position.
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
        return not self.status[self.used_arm + '_hand_empty']

    def _predict_contact_pose(self):
        return np.array([0, 0, 0]), None

    def _pre_movement(self, position=np.array([0, 0, 0]), orientation=None):
        return

    def _gripper_movement(self):
        self.env.open_gripper(self.used_arm)

    def _post_movement(self):
        for wave_point in self._pre_movement_wavepoints:
            self.env.step(
                mode=self.used_arm,
                position=wave_point,
            )

    def _effect(self):
        self.synchronize_status()
