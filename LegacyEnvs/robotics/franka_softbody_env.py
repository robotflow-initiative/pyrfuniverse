from pyrfuniverse.envs.gym_goal_wrapper_env import RFUniverseGymGoalWrapper

# from pyrfuniverse.utils import RFUniverseController
import numpy as np
import math
import copy
from gym import spaces
from gym.utils import seeding


class FrankaSoftbodyEnv(RFUniverseGymGoalWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        asset_bundle_file,
        reward_type,
        open_gripper=True,
        tolerance=0.05,
        goal_xz_range=0.15,
        softbody_xz_range=0.15,
        executable_file=None,
    ):
        super().__init__(
            executable_file=executable_file,
            articulation_channel=True,
            game_object_channel=True,
            obi_softbody_channel=True,
        )
        self.asset_bundle_file = asset_bundle_file
        self.reward_type = reward_type
        self.open_gripper = open_gripper
        self.tolerance = tolerance

        # Fixed parameters
        self.scale = 10.0
        self.goal_center_position = np.array([-0.6, 0.015, 0])
        self.softbody_center_position = np.array([-0.6, 0.022, 0])
        self.goal_range_low = self.goal_center_position - np.array(
            [goal_xz_range, 0, goal_xz_range]
        )
        self.goal_range_high = self.goal_center_position + np.array(
            [goal_xz_range, 0, goal_xz_range]
        )
        self.softbody_range_low = self.softbody_center_position - np.array(
            [softbody_xz_range, 0, softbody_xz_range]
        )
        self.softbody_range_high = self.softbody_center_position + np.array(
            [softbody_xz_range, 0, softbody_xz_range]
        )

        # Env setup
        self.seed()
        self.load_softbody = False
        self._load_obi_softbody()
        self.ik_controller = RFUniverseController(
            "franka", base_pos=np.array([0, 0, 0])
        )
        self.t = 0
        self.goal = self._sample_goal()
        self.action_space = spaces.Box(low=-1, high=1, shape=(4,), dtype=np.float32)
        obs = self._get_obs()
        self.observation_space = spaces.Dict(
            {
                "observation": spaces.Box(
                    -np.inf, np.inf, shape=obs["observation"].shape, dtype=np.float32
                ),
                "desired_goal": spaces.Box(
                    -np.inf, np.inf, shape=obs["desired_goal"].shape, dtype=np.float32
                ),
                "achieved_goal": spaces.Box(
                    -np.inf, np.inf, shape=obs["achieved_goal"].shape, dtype=np.float32
                ),
            }
        )

    def step(self, action: np.ndarray):
        pos_ctrl = action[:3] * 0.05
        curr_pos = self._get_gripper_position()
        pos_ctrl = curr_pos + pos_ctrl
        joint_positions = self.ik_controller.calculate_ik(pos_ctrl)

        if self.open_gripper:
            curr_gripper_width = self._get_gripper_width()
            target_gripper_width = curr_gripper_width + action[3] * 0.2
            target_gripper_width = np.clip(target_gripper_width, 0, 0.08)
            joint_positions.append(target_gripper_width)
        else:
            joint_positions.append(0)

        self._set_franka_joints(np.array(joint_positions))
        self.t += 1

        obs = self._get_obs()
        done = False
        info = {"is_success": self._check_success(obs)}
        # if info['is_success'] > 0:
        #     print('Success')
        reward = self.compute_reward(obs["achieved_goal"], obs["desired_goal"], info)

        return obs, reward, done, info

    def reset(self):
        super().reset()

        self._destroy_obi_softbody()
        self.env.reset()
        if not self.open_gripper:
            self.articulation_channel.set_action(
                "SetJointPositionDirectly",
                index=1,
                joint_positions=[0, 0],
            )

        obi_softbody_solver_position = self._load_obi_softbody()
        self.goal = self._sample_goal()
        self.game_object_channel.set_action(
            "SetTransform", index=0, position=list(self.goal * self.scale)
        )

        self.t = 0
        self.ik_controller.reset()
        joint_positions = self.ik_controller.calculate_ik_recursive(
            list(obi_softbody_solver_position),
        )
        self.articulation_channel.set_action(
            "SetJointPositionDirectly",
            index=0,
            joint_positions=list(joint_positions),
        )
        self._step()

        return self._get_obs()

    def seed(self, seed=1234):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def compute_reward(self, achieved_goal, desired_goal, info):
        distance = self._compute_goal_distance(achieved_goal, desired_goal)
        if self.reward_type == "sparse":
            return -(distance > self.tolerance).astype(np.float32)
        else:
            return -distance

    def _get_obs(self):  #
        gripper_position = np.array(self.articulation_channel.data[1]["positions"][3])
        gripper_velocity = np.array(self.articulation_channel.data[1]["velocities"][3])
        gripper_width = self._get_gripper_width()
        panda_obs = (
            np.concatenate((gripper_position, gripper_velocity, [gripper_width]))
            / self.scale
        )

        softbody_position = (
            np.array(self.obi_softbody_channel.data[0]["position"]) / self.scale
        )
        softbody_orientation = np.array(
            self.obi_softbody_channel.data[0]["orientation"]
        )
        softbody_velocity = (
            np.array(self.obi_softbody_channel.data[0]["velocity"]) / self.scale
        )
        softbody_angular_vel = np.array(
            self.obi_softbody_channel.data[0]["angular_vel"]
        )
        softbody_obs = np.concatenate(
            (
                softbody_position,
                softbody_orientation,
                softbody_velocity,
                softbody_angular_vel,
            )
        )

        obs = np.concatenate((panda_obs, softbody_obs))

        return {
            "observation": obs.copy(),
            "achieved_goal": softbody_position.copy(),
            "desired_goal": self.goal.copy(),
        }

    def _load_obi_softbody(self):
        if self.load_softbody:
            return
        obi_softbody_solver_position = self.np_random.uniform(
            self.softbody_range_low, self.softbody_range_high
        )
        self.asset_channel.set_action(
            "LoadObiSoftbody",
            filename=self.asset_bundle_file,
            name="Obi Softbody Solver",
            position=list(obi_softbody_solver_position * self.scale),
        )
        self._step()
        self.load_softbody = True

        return obi_softbody_solver_position

    def _destroy_obi_softbody(self):
        if not self.load_softbody:
            return
        self.obi_softbody_channel.set_action("Destroy", index=0)
        self._step()
        self.load_softbody = False

    def _sample_goal(self):
        goal = self.np_random.uniform(self.goal_range_low, self.goal_range_high)
        return goal.copy()

    def _get_gripper_position(self):
        return np.array(self.articulation_channel.data[1]["positions"][3]) / self.scale

    def _get_gripper_width(self):
        gripper_joint_positions = copy.deepcopy(
            self.articulation_channel.data[1]["joint_positions"]
        )
        return (
            -1 * (gripper_joint_positions[0] + gripper_joint_positions[1]) / self.scale
        )

    def _set_franka_joints(self, a: np.ndarray):
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=0,
            joint_positions=list(a[0:7]),
        )
        self._step()

        a[7] = -1 * a[7] / 2 * self.scale
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=1,
            joint_positions=[a[7], a[7]],
        )
        self._step()

    def _compute_goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _check_success(self, obs):
        achieved_goal = obs["achieved_goal"]
        desired_goal = obs["desired_goal"]
        distance = self._compute_goal_distance(achieved_goal, desired_goal)

        return (distance < self.tolerance).astype(np.float32)
