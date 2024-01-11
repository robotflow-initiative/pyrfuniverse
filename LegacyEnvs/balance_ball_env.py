from pyrfuniverse.envs import RFUniverseGymWrapper
import numpy as np
from gym import spaces
from gym.utils import seeding


class BalanceBallEnv(RFUniverseGymWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(self, executable_file=None):
        super().__init__(
            executable_file,
            rigidbody_channel=True,
            game_object_channel=True,
        )
        self.action_space = spaces.Box(low=-2.0, high=2.0, shape=(2,), dtype=np.float32)
        self.observation_space = spaces.Box(
            low=np.array(
                [-1, -1, -3, -2, -3, -float("inf"), -float("inf"), -float("inf")]
            ),
            high=np.array([1, 1, 3, 4, 3, float("inf"), float("inf"), float("inf")]),
            dtype=np.float32,
        )
        self.t = 0
        self.r = 0
        self.max_steps = 50

    def step(self, a: np.ndarray):
        """
        Params:
            a: 2-d numpy array. The first dimension is for cube's x axis rotation, while the second dimension is for
            cube's z axis rotation.
        """
        self.game_object_channel.set_action("Rotate", index=0, rotation=[a[0], 0, a[1]])
        self._step()

        done = False
        info = {}
        cube_position = self.game_object_channel.data[0]["position"]
        sphere_position = self.rigidbody_channel.data[0]["position"]
        is_fail = self._check_fail(cube_position, sphere_position)
        if is_fail:
            self.r = -1
            done = True
            self.reset()
            info["done"] = True
            info["is_success"] = False
        elif self.t == self.max_steps:
            self.r = 0.1
            done = True
            self.reset()
            info["done"] = True
            info["is_success"] = True
        else:
            self.r = 0.1
            self.t += 1
            info["done"] = False

        return self._get_obs(), self.r, done, info

    def _get_obs(self):
        cube_quaternion = self.game_object_channel.data[0]["quaternion"]
        cube_position = self.game_object_channel.data[0]["position"]
        sphere_position = self.rigidbody_channel.data[0]["position"]
        sphere_velocify = self.rigidbody_channel.data[0]["velocity"]

        rotation = np.array([cube_quaternion[0], cube_quaternion[2]])
        relative_position = np.array(sphere_position) - np.array(cube_position)
        sphere_velocify = np.array(sphere_velocify)

        return np.concatenate((rotation, relative_position, sphere_velocify))

    def reset(self):
        self.t = 0

        cubeRotationX = self._generate_random_float(-10, 10)
        cubeRotationZ = self._generate_random_float(-10, 10)
        spherePositionX = self._generate_random_float(-1.5, 1.5)
        spherePositionZ = self._generate_random_float(-1.5, 1.5)

        self.env_param_channel.set_float_parameter("cubeRotationX", cubeRotationX)
        self.env_param_channel.set_float_parameter("cubeRotationZ", cubeRotationZ)
        self.env_param_channel.set_float_parameter("spherePositionX", spherePositionX)
        self.env_param_channel.set_float_parameter("spherePositionZ", spherePositionZ)

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

    def _check_fail(self, cube_position, sphere_position):
        if sphere_position[1] - cube_position[1] < -2:
            return True

        if abs(sphere_position[0] - cube_position[0]) > 3:
            return True

        if abs(sphere_position[2] - cube_position[2]) > 3:
            return True

        return False


class BalanceBallEnvV0(BalanceBallEnv):
    def __init__(self):
        super().__init__(
            "/home/haoyuan/workspace/rfuniverse/build/BalanceBall/RFUniverse.x86_64"
        )
