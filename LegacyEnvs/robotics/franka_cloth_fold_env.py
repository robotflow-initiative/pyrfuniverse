from pyrfuniverse.envs.gym_goal_wrapper_env import RFUniverseGymGoalWrapper

# from pyrfuniverse.utils import RFUniverseController
import numpy as np
from gym import spaces
from gym.utils import seeding
import copy


class FrankaClothFoldEnv(RFUniverseGymGoalWrapper):
    metadata = {"render.modes": ["human"]}
    height_offset = 0.01

    def __init__(
        self,
        asset_bundle_file,
        executable_file=None,
        reward_type="sparse",
        tolerance=0.05,
        object_xz_range=0.055,
        object_rotation_range=180,
    ):
        super().__init__(
            executable_file=executable_file,
            articulation_channel=True,
            obi_cloth_with_grasping_channel=True,
        )
        self.reward_type = reward_type
        self.tolerance = tolerance
        self.object_xz_range = object_xz_range
        self.object_rotation_range = object_rotation_range
        self.object_range_low = np.array(
            [-object_xz_range, self.height_offset, -object_xz_range]
        )
        self.object_range_high = np.array(
            [object_xz_range, self.height_offset, object_xz_range]
        )
        self.asset_bundle_file = asset_bundle_file
        self.ik_controller = RFUniverseController(
            "franka", base_pos=np.array([-0.6, 0, 0])
        )
        self.t = 0

        self.has_loaded_object = False
        self.goal = None
        self.seed()
        self._load_object()

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
        curr_pos = np.array(self.articulation_channel.data[1]["positions"][3])
        pos_ctrl = curr_pos + pos_ctrl
        joint_positions = self.ik_controller.calculate_ik_recursive(pos_ctrl)

        curr_gripper_width = self._get_gripper_width()
        target_gripper_width = curr_gripper_width + action[3] * 0.2
        target_gripper_width = np.clip(target_gripper_width, 0, 0.08)
        joint_positions.append(target_gripper_width)

        self._set_franka_joints(np.array(joint_positions))
        self.t += 1

        obs = self._get_obs()
        done = False
        info = {"is_success": self._check_success(obs)}
        reward = self.compute_reward(obs["achieved_goal"], obs["desired_goal"], info)

        return obs, reward, done, info

    def reset(self):
        super().reset()
        self._destroy_object()
        self.env.reset()
        self.ik_controller.reset()
        self.t = 0
        self._load_object()

        grasp_position = self._get_grasp_position()
        joint_positions = self.ik_controller.calculate_ik_recursive(grasp_position)
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

    def _get_obs(self):
        gripper_position = np.array(self.articulation_channel.data[1]["positions"][3])
        gripper_velocity = np.array(self.articulation_channel.data[1]["velocities"][3])
        gripper_width = self._get_gripper_width()

        panda_obs = np.concatenate(
            (gripper_position, gripper_velocity, [gripper_width])
        )

        grasp_position = np.array(
            self.obi_cloth_with_grasping_channel.data[0]["grasp_position"]
        )
        grasp_rotation = np.array(
            self.obi_cloth_with_grasping_channel.data[0]["grasp_rotation"]
        )
        grasp_velocity = np.array(
            self.obi_cloth_with_grasping_channel.data[0]["grasp_velocity"]
        )
        grasp_angular_vel = np.array(
            self.obi_cloth_with_grasping_channel.data[0]["grasp_angular_vel"]
        )
        achieved_goal = grasp_position.copy()

        object_obs = np.concatenate(
            (grasp_position, grasp_rotation, grasp_velocity, grasp_angular_vel)
        )

        obs = np.concatenate((panda_obs, object_obs))

        return {
            "observation": obs.copy(),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": self.goal.copy(),
        }

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min < max, "Min value is {}, while max value is {}.".format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float

    def _set_franka_joints(self, a: np.ndarray):
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=0,
            joint_positions=list(a[0:7]),
        )
        self._step()

        a[7] = -1 * a[7] / 2
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=1,
            joint_positions=[a[7], a[7]],
        )
        self._step()

    def _get_gripper_width(self):
        gripper_joint_positions = copy.deepcopy(
            self.articulation_channel.data[1]["joint_positions"]
        )
        return -1 * (gripper_joint_positions[0] + gripper_joint_positions[1])

    def _check_success(self, obs):
        achieved_goal = obs["achieved_goal"]
        desired_goal = obs["desired_goal"]
        distance = self._compute_goal_distance(achieved_goal, desired_goal)

        return (distance < self.tolerance).astype(np.float32)

    def _compute_goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _load_object(self):
        assert (
            self.asset_bundle_file is not None
        ), "There must be an asset bundle file to load."

        object_name = "Obi Solver For Grasp"
        position = self.np_random.uniform(self.object_range_low, self.object_range_high)
        rotation = [
            0,
            self._generate_random_float(
                -self.object_rotation_range, self.object_rotation_range
            ),
            0,
        ]

        self.asset_channel.set_action(
            "LoadObiClothWithGrasping",
            filename=self.asset_bundle_file,
            name=object_name,
            position=position,
            rotation=rotation,
        )
        self._step()
        self.has_loaded_object = True
        self.goal = np.array(
            self.obi_cloth_with_grasping_channel.data[0]["target_position"]
        )

    def _destroy_object(self):
        self.obi_cloth_with_grasping_channel.set_action("Destroy", index=0)
        self._step()
        self.has_loaded_object = False
        self.goal = None

    def _get_grasp_position(self):
        assert self.has_loaded_object, "No object loaded."
        grasp_position = np.array(
            self.obi_cloth_with_grasping_channel.data[0]["grasp_position"]
        )

        return grasp_position
