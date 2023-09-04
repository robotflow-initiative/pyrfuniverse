from pyrfuniverse.envs import RFUniverseBaseEnv
from pyrfuniverse.envs import RFUniverseGymWrapper
import numpy as np
from gym import spaces
from gym.utils import seeding
import math
import copy


class Robotiq85NailCanEnv(RFUniverseGymWrapper):
    def __init__(
        self,
        max_episode_length=40,
        prismatic_factor=0.05,
        revolute_facotr=0,
        min_success_degree=45,
        radius=0.15,
        executable_file=None,
    ):
        super().__init__(
            executable_file=executable_file,
            articulation_channel=True,
        )
        self.max_episode_length = max_episode_length
        self.prismatic_factor = prismatic_factor
        self.revolute_factor = revolute_facotr
        self.bit_wise_factor = np.array(
            [
                self.prismatic_factor,
                self.prismatic_factor,
                self.prismatic_factor,
                self.revolute_factor,
            ]
        )
        self.min_success_degree = min_success_degree
        self.radius = radius

        self.t = 0
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
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
        info = {"is_success": reward * 10 > self.min_success_degree}
        done = False
        if self.t == self.max_episode_length:
            done = True
            obs = self.reset()

        return obs, reward, done, info

    def reset(self):
        self.env.reset()
        self.t = 0

        theta = self._generate_random_float(-1 * math.pi / 2, math.pi / 2)
        self.articulation_channel.set_action(
            "SetJointPositionDirectly",
            index=0,
            joint_positions=[
                -self.radius * math.cos(theta),
                0,
                -self.radius * math.sin(theta),
                45,
            ],
        )
        self._step()

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def _get_obs(self):
        gripper_position = np.array(self.articulation_channel.data[0]["positions"][17])
        gripper_velocity = np.array(self.articulation_channel.data[0]["velocities"][17])
        gripper_extra_param = self._get_gripper_extra_param()

        # print(self.articulation_channel.data[1])
        ring_pos = np.array(self.articulation_channel.data[1]["positions"][1])
        ring_rotation = np.array(
            [self.articulation_channel.data[1]["joint_positions"][0]]
        )

        obs = np.concatenate(
            (
                gripper_position,
                gripper_velocity,
                gripper_extra_param,
                ring_pos,
                ring_rotation,
            )
        )

        return obs

    def _get_gripper_extra_param(self):
        joint_positions = np.array(self.articulation_channel.data[0]["joint_positions"])
        x_prismatic_joint_position = joint_positions[0]
        y_prismatic_joint_position = joint_positions[1]
        z_prismatic_joint_position = joint_positions[2]
        revolute_joint_position = joint_positions[3]

        return np.array(joint_positions)

    def _set_target_state(self, target_state):
        self.articulation_channel.set_action(
            "SetJointPosition", index=0, joint_positions=list(target_state)
        )
        self._step()

    def _compute_reward(self, obs):
        ring_rotation = obs[-1] * -1
        if ring_rotation < 5:
            return 0
        else:
            return ring_rotation / 10

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min < max, "Min value is {}, while max value is {}.".format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float
