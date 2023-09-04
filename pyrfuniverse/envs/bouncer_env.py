from pyrfuniverse.envs import RFUniverseGymWrapper
from pyrfuniverse.side_channel import SideChannel, IncomingMessage
import numpy as np
from gym import spaces
from gym.utils import seeding
import uuid


class SuccessChannel(SideChannel):
    def __init__(self):
        super().__init__(uuid.UUID("f32e9f8c-b04d-4720-afe6-d7574ea713a5"))
        self.data = {}

    def on_message_received(self, msg: IncomingMessage) -> None:
        self.data["success"] = msg.read_bool()
        self.data["finish_last_step"] = msg.read_bool()


class BouncerEnv(RFUniverseGymWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(self, executable_file=None):
        self.success_channel = SuccessChannel()
        super().__init__(
            executable_file,
            custom_channels=[self.success_channel],
            rigidbody_channel=True,
        )
        self.action_space = spaces.Box(
            low=np.array([-1, -1, -1]), high=np.array([1, 1, 1]), dtype=np.float32
        )
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
        )
        self.strength = 500
        self.t = 0
        self.max_steps = 20

    def step(self, a: np.ndarray):
        force = a.copy()
        force[1] = (force[1] + 1) / 2 + 1
        self.env_param_channel.set_float_parameter("lookDirX", force[0])
        self.env_param_channel.set_float_parameter("lookDirY", force[1] - 1)
        self.env_param_channel.set_float_parameter("lookDirZ", force[2])

        force = force * self.strength
        self.rigidbody_channel.set_action("AddForce", index=0, force=list(force))

        self.success_channel.data = {}
        success = False
        done = False
        reward = -0.05 * (np.linalg.norm(a) ** 2) / 3
        info = {}

        while (
            self.success_channel.data == {}
            or self.success_channel.data["finish_last_step"] == False
        ):
            self._step()
            if (
                "success" in self.success_channel.data.keys()
                and self.success_channel.data["success"]
            ):
                success = True
            if self._check_fail():
                success = False
                reward = -1
                done = True
                break

        if success:
            reward = 1
            info["is_success"] = True
        else:
            info["is_success"] = False

        self.t += 1
        if self.t == self.max_steps:
            done = True

        if done:
            self.reset()

        return self._get_obs(), reward, done, info

    def _check_fail(self):
        position = self.rigidbody_channel.data[0]["position"]
        if position[1] < -1:
            return True
        if (
            position[0] < -19
            or position[0] > 19
            or position[2] < -19
            or position[2] > 19
        ):
            return True

        return False

    def _get_obs(self):
        object_pos = self.rigidbody_channel.data[0]["position"]
        object_pos = np.array(object_pos)
        target_pos = self.rigidbody_channel.data[1]["position"]
        target_pos = np.array(target_pos)

        return np.concatenate((object_pos, target_pos))

    def reset(self):
        self.t = 0
        self.env_param_channel.set_float_parameter(
            "bouncerPosX", self._generate_random_float(-5, 5)
        )
        self.env_param_channel.set_float_parameter(
            "bouncerPosZ", self._generate_random_float(-5, 5)
        )
        self.env_param_channel.set_float_parameter(
            "targetPosX", self._generate_random_float(-5, 5)
        )
        self.env_param_channel.set_float_parameter(
            "targetPosY", self._generate_random_float(2, 7)
        )
        self.env_param_channel.set_float_parameter(
            "targetPosZ", self._generate_random_float(-5, 5)
        )
        self.env_param_channel.set_float_parameter("lookDirX", 0)
        self.env_param_channel.set_float_parameter("lookDirY", 0)
        self.env_param_channel.set_float_parameter("lookDirZ", 0)

        self.env.reset()

        return self._get_obs()

    def seed(self, seed=1234):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min < max, "Min value is {}, while max value is {}.".format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float


class BouncerEnvV0(BouncerEnv):
    def __init__(self):
        super().__init__(
            "/home/haoyuan/workspace/rfuniverse/build/Bouncer/RFUniverse.x86_64"
        )
