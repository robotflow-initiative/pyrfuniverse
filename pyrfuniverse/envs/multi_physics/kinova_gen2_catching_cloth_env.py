import math
from pyrfuniverse.envs.gym_wrapper_env import RFUniverseGymWrapper
import pyrfuniverse.attributes as attr
from pyrfuniverse.utils.jaco_controller import RFUniverseJacoController
import numpy as np
from gym import spaces
from gym.utils import seeding
import pybullet as p


class KinovaGen2CatchingClothEnv(RFUniverseGymWrapper):
    metadata = {"render.modes": ["human", "rgb_array"]}
    scale = 8
    y_bound = 3.8
    kinova_init_y = 4.8
    object2id = {"kinova": 1001, "target": 1000, "cloth": 89212}

    def __init__(
        self,
        urdf_file,
        lock_eef_height=False,
        with_force_zone=False,
        force_zone_intensity=1,
        force_zone_turbulence=10,
        max_steps=100,
        cloth_init_pos_min=(-1.5, 8, 3.3),
        cloth_init_pos_max=(1.5, 8, 6.3),
        executable_file=None,
        assets=["FallingClothSolver"],
    ):
        super().__init__(
            executable_file=executable_file,
            scene_file="CatchingCloth.json",
            assets=assets,
        )
        self.lock_eef_height = lock_eef_height
        self.with_force_zone = with_force_zone
        self.force_zone_intensity = force_zone_intensity
        self.force_zone_turbulence = force_zone_turbulence
        self.max_steps = max_steps
        self.cloth_init_pos_min = np.array(cloth_init_pos_min)
        self.cloth_init_pos_max = np.array(cloth_init_pos_max)
        self.init_eef_pos = np.array([0, self.kinova_init_y, 4.8]) / self.scale
        self.eef_orn = p.getQuaternionFromEuler(np.array([0, 0, 0]))

        self.seed()
        self._load_cloth()
        self.ik_controller = RFUniverseJacoController(urdf_file)
        self.t = 0
        if not self.lock_eef_height:
            self.action_space = spaces.Box(low=-1, high=1, shape=(3,), dtype=np.float32)
        else:
            self.action_space = spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
        obs = self._get_obs()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=obs.shape, dtype=np.float32
        )

    def step(self, action: np.ndarray):
        delta_pos = action * 0.05
        curr_pos = self._get_eef_position()
        if self.lock_eef_height:
            target_pos = np.array(
                [
                    curr_pos[0] + delta_pos[0],
                    self.kinova_init_y / self.scale,
                    curr_pos[2] + delta_pos[1],
                ]
            )
        else:
            target_pos = curr_pos + delta_pos
        joint_positions = self.ik_controller.calculate_ik_recursive(
            unity_eef_pos=target_pos, eef_orn=self.eef_orn
        )
        self._set_kinova_joints(joint_positions)

        self.t += 1
        obs = self._get_obs()
        reward = self._compute_reward(obs)
        done = False
        fail = self._check_fail(obs)
        if fail or self.t == self.max_steps:
            done = True
        if fail:
            reward -= self.max_steps
        info = {"is_success": done and not fail}

        return obs, reward, done, info

    def reset(self):
        self._destroy_cloth()
        self.ik_controller.reset()
        self.t = 0

        init_joint_positions = self.ik_controller.calculate_ik_recursive(
            unity_eef_pos=self.init_eef_pos, eef_orn=self.eef_orn
        )
        self._set_kinova_joints_directly(init_joint_positions)
        self._load_cloth()

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def heuristic(self):
        target_position = self._get_target_position()
        joint_positions = self.ik_controller.calculate_ik_recursive(
            unity_eef_pos=target_position, eef_orn=self.eef_orn
        )
        self._set_kinova_joints_directly(joint_positions)

    def _get_obs(self):
        catcher_position = self._get_eef_position()
        cloth_position = self._get_cloth_position()
        eef_velocity = self._get_eef_velocity()
        cloth_velocity = self._get_cloth_velocity()
        force_zone_parameters = (
            self._get_force_zone_parameters() if self.with_force_zone else np.array([])
        )

        return np.concatenate(
            (
                catcher_position,
                cloth_position,
                eef_velocity,
                cloth_velocity,
                force_zone_parameters,
            )
        )

    def _init_scene(self):
        self.attrs[self.object2id["kinova"]].SetTransform(
            scale=[self.scale, self.scale, self.scale]
        )
        self._step()

    def _load_cloth(self):
        cloth = self.InstanceObject(
            name="FallingClothSolver",
            id=self.object2id["cloth"],
            attr_type=attr.FallingClothAttr,
        )
        cloth.SetTransform(
            position=list(
                self.np_random.uniform(
                    low=self.cloth_init_pos_min, high=self.cloth_init_pos_max
                )
            )
        )
        cloth.SetSolverParameters(gravity=[0, -9.8, 0])
        if self.with_force_zone:
            cloth.SetForceZoneParameters(
                orientation=self.np_random.uniform(-180, 180),
                intensity=self.force_zone_intensity,
                turbulence=self.force_zone_turbulence,
                turbulence_frequency=2,
            )
        self._step()

    def _destroy_cloth(self):
        self.attrs[self.object2id["cloth"]].Destroy()
        self._step()

    def _set_kinova_joints(self, joint_positions):
        joint_positions[7] = 50
        joint_positions[8] = 50
        joint_positions[9] = 50
        self.attrs[self.object2id["kinova"]].SetJointPosition(
            joint_positions=list(joint_positions[0:10])
        )
        self._step()

    def _set_kinova_joints_directly(self, joint_positions):
        joint_positions[7] = 50
        joint_positions[8] = 50
        joint_positions[9] = 50
        self.attrs[self.object2id["kinova"]].SetJointPositionDirectly(
            joint_positions=list(joint_positions[0:10])
        )
        self._step()

    def _get_eef_position(self):
        return (
            np.array(self.attrs[self.object2id["kinova"]].data["positions"][15])
            / self.scale
        )

    def _get_eef_velocity(self):
        return (
            np.array(self.attrs[self.object2id["kinova"]].data["velocities"][15])
            / self.scale
        )

    def _get_target_position(self):
        return (
            np.array(self.attrs[self.object2id["target"]].data["position"]) / self.scale
        )

    def _get_cloth_position(self):
        return (
            np.array(self.attrs[self.object2id["cloth"]].data["avg_position"])
            / self.scale
        )

    def _get_cloth_velocity(self):
        return (
            np.array(self.attrs[self.object2id["cloth"]].data["avg_velocity"])
            / self.scale
        )

    def _get_force_zone_parameters(self):
        return np.array(
            [
                self.attrs[self.object2id["cloth"]].data["force_zone_orientation"]
                / 180
                * math.pi,
                self.attrs[self.object2id["cloth"]].data["force_zone_intensity"],
                self.attrs[self.object2id["cloth"]].data["force_zone_turbulence"],
            ]
        )

    def _check_fail(self, obs):
        cloth_position = obs[3:6]
        return cloth_position[1] < self.y_bound / self.scale

    def _compute_reward(self, obs):
        catcher_position = obs[:3]
        cloth_position = obs[3:6]
        eef_velocity = np.array(obs[6:9])
        cloth_velocity = np.array(obs[9:12])
        """
        Reward constrains the catcher to get closer to cloth as well as maximizing the cloth height and
        minimizing the velocity of end effector. The velocity of end effector should be considered more.
        """

        reward = 0
        # reward += np.tanh(10.0 * cloth_position[1])
        # reward += 1 - np.tanh(10.0 * self._distance(catcher_position, cloth_position))
        reward += 1 - np.tanh(10.0 * abs(cloth_velocity[1]))
        reward += 1 - np.tanh(10.0 * np.linalg.norm(eef_velocity))
        return reward

    def _distance(self, pos1, pos2):
        return np.linalg.norm(pos1 - pos2, axis=-1)
