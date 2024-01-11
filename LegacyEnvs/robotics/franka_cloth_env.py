from pyrfuniverse.envs.gym_goal_wrapper_env import RFUniverseGymGoalWrapper
import numpy as np
import math
import copy
from gym import spaces
from gym.utils import seeding


class FrankaClothEnv(RFUniverseGymGoalWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        asset_bundle_file,
        reward_type,
        tolerance=0.08,
        target_x_range=(-0.1, -0.1),
        target_z_range=(-0.1, 0.1),
        executable_file=None,
    ):
        super().__init__(
            executable_file=executable_file,
            articulation_channel=True,
            game_object_channel=True,
            obi_cloth_channel=True,
        )
        self.asset_bundle_file = asset_bundle_file
        self.reward_type = reward_type
        self.tolerance = tolerance
        self.target_x_range = target_x_range
        self.target_z_range = target_z_range

        # Fixed parameters
        self.obi_solver_position = np.array([-0.1, 0.31, -0.1])
        self.number_of_particles = 289
        self.number_of_sampled_particles = 10

        # Env setup
        self.load_cloth = False
        self._load_obi_cloth()
        self.ik_controller = RFUniverseController(
            "franka", base_pos=np.array([-0.6, 0, 0])
        )
        self.eef_orn = self.ik_controller.bullet_client.getQuaternionFromEuler(
            [math.pi / 2, 0, math.pi / 2]
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
        curr_pos = np.array(self.articulation_channel.data[1]["positions"][3])
        pos_ctrl = curr_pos + pos_ctrl
        joint_positions = self.ik_controller.calculate_ik(
            pos_ctrl, eef_orn=self.eef_orn
        )

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

        self._destroy_obi_cloth()
        self.env.reset()
        self._load_obi_cloth()
        self.goal = self._sample_goal()

        self.t = 0
        self.ik_controller.reset()
        joint_positions = self.ik_controller.calculate_ik([0, 0.3, 0.1], self.eef_orn)
        self.articulation_channel.set_action(
            "SetJointPositionDirectly",
            index=0,
            joint_positions=joint_positions,
        )

        self.game_object_channel.set_action(
            "SetTransform", index=0, position=list(self.goal.copy())
        )

        self._step()

        return self._get_obs()

    def seed(self, seed=None):
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

        cloth_particles = np.array(self.obi_cloth_channel.data[0]["positions"])
        random_indices = np.random.randint(
            0, self.number_of_particles, size=(self.number_of_sampled_particles,)
        )
        sampled_particles = cloth_particles[random_indices] + self.obi_solver_position
        sampled_particles = np.reshape(
            sampled_particles, newshape=(3 * self.number_of_sampled_particles,)
        )

        obs = np.concatenate((panda_obs, sampled_particles))

        cloth_center = np.average(cloth_particles, axis=0) + self.obi_solver_position

        return {
            "observation": obs.copy(),
            "achieved_goal": cloth_center.copy(),
            "desired_goal": self.goal.copy(),
        }

    def _load_obi_cloth(self):
        if self.load_cloth:
            return
        self.asset_channel.set_action(
            "LoadObiCloth",
            filename=self.asset_bundle_file,
            name="Obi Solver",
            position=list(self.obi_solver_position),
        )
        self._step()
        self.load_cloth = True

    def _destroy_obi_cloth(self):
        if not self.load_cloth:
            return
        self.obi_cloth_channel.set_action("Destroy", index=0)
        self._step()
        self.load_cloth = False

    def _get_gripper_width(self):
        gripper_joint_positions = copy.deepcopy(
            self.articulation_channel.data[1]["joint_positions"]
        )
        return -1 * (gripper_joint_positions[0] + gripper_joint_positions[1])

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

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min <= max, "Min value is {}, while max value is {}.".format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float

    def _compute_goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _check_success(self, obs):
        achieved_goal = obs["achieved_goal"]
        desired_goal = obs["desired_goal"]
        distance = self._compute_goal_distance(achieved_goal, desired_goal)

        return (distance < self.tolerance).astype(np.float32)

    def _sample_goal(self):
        target_x = self._generate_random_float(
            self.target_x_range[0], self.target_x_range[1]
        )
        target_z = self._generate_random_float(
            self.target_z_range[0], self.target_z_range[1]
        )

        return np.array([target_x, 0.3, target_z])
