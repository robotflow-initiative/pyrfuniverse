from pyrfuniverse.envs import RFUniverseBaseEnv
from pyrfuniverse.envs import RFUniverseGymWrapper
import numpy as np
from gym import spaces
from gym.utils import seeding
import math
import copy


class Robotiq85InsertEnv(RFUniverseGymWrapper):
    def __init__(
        self,
        max_episode_length=40,
        prismatic_factor=0.05,
        insert_factor=0.05,
        hole_range=0.15,
        min_success_reward=0.05,
        executable_file=None,
    ):
        super().__init__(
            executable_file=executable_file,
            articulation_channel=True,
            game_object_channel=True,
        )
        self.max_episode_length = max_episode_length
        self.hole_range = hole_range
        self.min_success_reward = min_success_reward
        self.prismatic_factor = prismatic_factor
        self.insert_factor = insert_factor
        self.bit_wise_factor = np.array(
            [self.prismatic_factor, self.prismatic_factor, self.insert_factor]
        )

        self.t = 0
        self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        obs = self._get_obs()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=obs.shape, dtype=np.float32
        )

    def step(self, a: np.ndarray):
        action = a.copy()
        action_ctrl = action * self.bit_wise_factor
        curr_state = self._get_gripper_extra_param()
        target_state = action_ctrl + curr_state

        self._set_target_state(target_state)
        self.t += 1

        obs = self._get_obs()
        reward = self._compute_reward(obs)
        info = {"is_success": reward > self.min_success_reward}
        done = False
        if self.t == self.max_episode_length:
            done = True
            obs = self.reset()

        return obs, reward, done, info

    def reset(self):
        self.env.reset()
        self.t = 0

        hole_x = self._generate_random_float(-self.hole_range, self.hole_range)
        hole_z = self._generate_random_float(-self.hole_range, self.hole_range)
        self.game_object_channel.set_action(
            "SetTransform", index=0, position=[hole_x, 0, hole_z]
        )
        self._step()

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def _get_obs(self):
        bar_position = np.array(self.articulation_channel.data[0]["positions"][10])
        bar_velocity = np.array(self.articulation_channel.data[0]["velocities"][10])
        gripper_extra_param = self._get_gripper_extra_param()

        hole_pos = np.array(self.game_object_channel.data[0]["position"])

        obs = np.concatenate(
            (bar_position, bar_velocity, gripper_extra_param, hole_pos)
        )

        return obs

    def _get_gripper_extra_param(self):
        joint_positions = np.array(self.articulation_channel.data[0]["joint_positions"])
        x_prismatic_joint_position = joint_positions[0]
        z_prismatic_joint_position = joint_positions[1]
        bar_prismatic_joint_position = joint_positions[2]

        return np.array(joint_positions)

    def _set_target_state(self, target_state):
        self.articulation_channel.set_action(
            "SetJointPosition", index=0, joint_positions=list(target_state)
        )
        self._step()

    def _compute_reward(self, obs):
        bar_height = obs[1]
        return bar_height * -1

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min <= max, "Min value is {}, while max value is {}.".format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float
