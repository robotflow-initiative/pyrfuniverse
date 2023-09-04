from pyrfuniverse.envs.gym_goal_wrapper_env import RFUniverseGymGoalWrapper
from pyrfuniverse.utils.ur5_controller import RFUniverseUR5Controller
import numpy as np
import pybullet as p
from gym import spaces
from gym.utils import seeding
import math


class UR5WaterShootingEnv(RFUniverseGymGoalWrapper):
    metadata = {"render.modes": ["human", "rgb_array"]}
    object2id = {
        "ur5": 1001,
        "robotiq85": 10010,
        "target": 1003,
        "right_finger": 100100,
        "left_finger": 100101,
        "cube": 1002,
        "goal": 1000,
    }

    def __init__(
        self,
        urdf_file,
        max_steps=100,
        object_range_low=(-20, 1, -5),
        object_range_high=(-16, 1, 5),
        goal_range_x=(-26, -23),
        goal_range_z=(-3, 3),
        tolerance=2,
        volume_per_time_step_range=(0.05, 0.15),
        liquid_init_velocity_range=(2, 3),
        executable_file=None,
        assets: list = [],
    ):
        super().__init__(
            executable_file=executable_file,
            scene_file="WaterShooting.json",
            assets=assets,
        )

        self.scale = 15
        self.max_steps = max_steps
        self.object_range_low = np.array(object_range_low)
        self.object_range_high = np.array(object_range_high)
        self.goal_range_x = np.array(goal_range_x)
        self.goal_range_z = np.array(goal_range_z)
        self.tolerance = tolerance
        self.volume_per_time_step_range = np.array(volume_per_time_step_range)
        self.liquid_init_velocity_range = np.array(liquid_init_velocity_range)

        self.ik_controller = RFUniverseUR5Controller(
            urdf_file, init_joint_positions=[0, -2 * math.pi / 3, math.pi / 2, 0, 0, 0]
        )
        self.eef_orn = p.getQuaternionFromEuler(
            np.array([-math.pi / 2, 0, -math.pi / 2])
        )
        self.init_pos = np.array([-8, 4, 0]) / self.scale
        self.init_gripper_width = 0.04

        self._env_setup()
        self.seed()
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
        delta_pos = action[:3].copy() * 0.05
        curr_pos = self._get_eef_position() / self.scale
        target_pos = curr_pos + delta_pos
        joint_positions = self.ik_controller.calculate_ik_recursive(
            unity_eef_pos=target_pos, eef_orn=self.eef_orn
        )

        delta_width = action[3] * 0.02
        curr_width = self._get_gripper_width() / self.scale
        target_width = np.clip(float(curr_width + delta_width), 0.04, 0.085)
        joint_positions.append(target_width)

        self._set_ur5_robotiq85(joint_positions)
        self._set_liquid_parameters(self._get_gripper_width() / self.scale)
        self.t += 1
        obs = self._get_obs()
        info = {
            "is_success": self._check_success(obs["achieved_goal"], obs["desired_goal"])
        }
        reward = self.compute_reward(obs["achieved_goal"], obs["desired_goal"], info)
        done = False
        fail = self._check_fail(obs["achieved_goal"])
        if fail or self.t == self.max_steps or info["is_success"] > 0:
            done = True

        return obs, reward, done, info

    def reset(self):
        self.ik_controller.reset()
        init_joint_positions = self.ik_controller.calculate_ik_recursive(
            unity_eef_pos=self.init_pos, eef_orn=self.eef_orn
        )
        init_joint_positions.append(self.init_gripper_width)
        self._set_ur5_robotiq85_directly(init_joint_positions)

        self.t = 0
        self._reset_liquid()
        object_pos = self._reset_object()
        self.goal = self._sample_goal(object_pos)

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def compute_reward(self, achieved_goal, desired_goal, info):
        distance = self._distance(achieved_goal, desired_goal) / self.scale
        return -distance.astype(np.float32)

    def heuristic(self):
        self.ik_controller.reset()
        target_pos = self._get_target_pos() / self.scale
        joint_positions = self.ik_controller.calculate_ik_recursive(
            unity_eef_pos=target_pos, eef_orn=self.eef_orn
        )
        joint_positions.append(0)
        self._set_ur5_robotiq85_directly(joint_positions)

    def _get_obs(self):
        eef_pos = self._get_eef_position()
        eef_vel = self._get_eef_velocity()
        eef_width = self._get_gripper_width()
        robot_obs = np.concatenate((eef_pos, eef_vel, [eef_width]))

        object_pos = self._get_cube_pos()
        object_vel = self._get_cube_velocity()
        object_rot = self._get_cube_rotation()
        object_obs = np.concatenate((object_pos, object_rot, object_vel))

        achieved_goal = object_pos.copy()
        desired_goal = self.goal.copy()

        return {
            "observation": np.concatenate((robot_obs, object_obs)),
            "achieved_goal": achieved_goal,
            "desired_goal": desired_goal,
        }

    def _env_setup(self):
        self._step()

    def _sample_goal(self, cube_pos=None):
        goal_x = self.np_random.uniform(
            low=self.goal_range_x[0], high=self.goal_range_x[1]
        )
        if cube_pos is not None:
            goal_z = cube_pos[2] + self.np_random.uniform(
                low=self.goal_range_z[0], high=self.goal_range_z[1]
            )
        else:
            goal_z = 0

        goal = [float(goal_x), 0, float(goal_z)]
        self.attrs[self.object2id["goal"]].SetTransform(position=list(goal))
        self._step()

        return np.array(goal)

    def _reset_liquid(self):
        self.SendMessage("SetZibraLiquid", False)
        self._step()
        self.SendMessage("SetZibraLiquid", True)
        self._step()

    def _reset_object(self):
        object_pos = self.np_random.uniform(
            low=self.object_range_low, high=self.object_range_high, size=(3,)
        )
        self.attrs[self.object2id["cube"]].SetTransform(
            position=list(object_pos), rotation=[0, 0, 0]
        )
        self._step()
        return object_pos.copy()

    def _get_eef_position(self):
        return np.array(self.attrs[self.object2id["robotiq85"]].data["positions"][7])

    def _get_eef_velocity(self):
        return np.array(self.attrs[self.object2id["robotiq85"]].data["velocities"][7])

    def _get_gripper_width(self):
        left_finger_pos = np.array(
            self.attrs[self.object2id["left_finger"]].data["position"]
        )
        right_finger_pos = np.array(
            self.attrs[self.object2id["right_finger"]].data["position"]
        )
        return self._distance(left_finger_pos, right_finger_pos)

    def _get_target_pos(self):
        return np.array(self.attrs[self.object2id["target"]].data["position"])

    def _get_cube_pos(self):
        return np.array(self.attrs[self.object2id["cube"]].data["position"])

    def _get_cube_velocity(self):
        return np.array(self.attrs[self.object2id["cube"]].data["velocity"])

    def _get_cube_rotation(self):
        return (
            np.array(self.attrs[self.object2id["cube"]].data["rotation"])
            / 180
            * math.pi
        )

    def _set_ur5_robotiq85(self, joint_positions):
        self.attrs[self.object2id["ur5"]].SetJointPosition(
            joint_positions=list(joint_positions[:6])
        )
        self._step()

        width = joint_positions[6]
        gripper_angle = self._compute_gripper_angle(width)
        self.attrs[self.object2id["robotiq85"]].SetJointPosition(
            joint_positions=[gripper_angle, gripper_angle]
        )
        self._step()

    def _set_ur5_robotiq85_directly(self, joint_positions):
        self.attrs[self.object2id["ur5"]].SetJointPositionDirectly(
            joint_positions=list(joint_positions[:6])
        )
        self._step()

        width = joint_positions[6]
        gripper_angle = self._compute_gripper_angle(width)
        self.attrs[self.object2id["robotiq85"]].SetJointPositionDirectly(
            joint_positions=[gripper_angle, gripper_angle]
        )
        self._step()

    def _distance(self, pos1, pos2):
        assert pos2.shape == pos1.shape
        return np.linalg.norm(pos1 - pos2, axis=-1)

    def _compute_gripper_angle(self, width):
        angle_rad = 0.715 - math.asin((width - 0.01) / 0.1143)
        angle_deg = angle_rad * 180 / math.pi

        return angle_deg

    def _set_liquid_parameters(self, gripper_width):
        prop = gripper_width / 0.085
        volume_per_time_step = (
            prop * self.volume_per_time_step_range[1]
            + (1 - prop) * self.volume_per_time_step_range[0]
        )
        liquid_init_velocity = (
            prop * self.liquid_init_velocity_range[1]
            + (1 - prop) * self.liquid_init_velocity_range[0]
        )
        self.SendMessage(
            "SetZibraLiquidEmitter",
            volume_per_time_step,
            0.0,
            -liquid_init_velocity,
            0.0,
        )
        self._step()

    def _check_success(self, achieved_goal, desired_goal):
        distance = abs(achieved_goal - desired_goal)
        return (max(distance[0], distance[2]) < self.tolerance).astype(np.float32)

    def _check_fail(self, achieved_goal):
        if achieved_goal[0] < -28 or abs(achieved_goal[2]) > 10:
            return True
        return False
