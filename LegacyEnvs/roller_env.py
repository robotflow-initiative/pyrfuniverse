from pyrfuniverse.envs import RFUniverseGymWrapper
import numpy as np
from gym import spaces
from gym.utils import seeding


class RollerEnv(RFUniverseGymWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(self, executable_file=None):
        super().__init__(
            executable_file, rigidbody_channel=True, game_object_channel=True
        )
        self.action_space = spaces.Discrete(5)
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(6,), dtype=np.float32
        )
        self.t = 0
        self.max_steps = 100
        self.strength = 250

    def step(self, a):
        direction = None
        if a == 0:
            direction = [0, 0, 0]
        elif a == 1:
            direction = [1, 0, 0]
        elif a == 2:
            direction = [-1, 0, 0]
        elif a == 3:
            direction = [0, 0, 1]
        elif a == 4:
            direction = [0, 0, -1]
        direction = np.array(direction)
        force = direction * self.strength

        self.rigidbody_channel.set_action("AddForce", index=0, force=list(force))
        self._step()

        done = False
        success = self._check_success()
        info = {"is_success": False}
        reward = -1 / self.max_steps
        self.t += 1

        if success:
            reward = 1
            done = True
            info["is_success"] = True
        elif self._check_fail():
            reward = -1
            done = True
        elif self.t == self.max_steps:
            done = True

        if done:
            self.reset()

        return self._get_obs(), reward, done, info

    def _check_fail(self):
        roller_position = self.rigidbody_channel.data[0]["position"]

        if roller_position[1] < -1:
            return True
        elif (
            roller_position[0] < -19
            or roller_position[0] > 19
            or roller_position[2] < -19
            or roller_position[2] > 19
        ):
            return True
        else:
            return False

    def _check_success(self):
        roller_position = self.rigidbody_channel.data[0]["position"]
        roller_position_2d = np.array([roller_position[0], roller_position[2]])
        target_position = self.game_object_channel.data[0]["position"]
        target_position_2d = np.array([target_position[0], target_position[2]])

        distance = abs(target_position_2d - roller_position_2d)
        if distance[0] < 2.5 and distance[1] < 2.5:
            return True
        else:
            return False

    def _get_obs(self):
        roller_position = self.rigidbody_channel.data[0]["position"]
        roller_position_2d = np.array([roller_position[0], roller_position[2]])
        roller_velocity = self.rigidbody_channel.data[0]["velocity"]
        roller_velocity_2d = np.array([roller_velocity[0], roller_velocity[2]])
        target_position = self.game_object_channel.data[0]["position"]
        target_position_2d = np.array([target_position[0], target_position[2]])

        return np.concatenate(
            (roller_position_2d, roller_velocity_2d, target_position_2d)
        )

    def reset(self):
        self.t = 0

        roller_pos = self.np_random.uniform(-15, 15, size=2)
        target_pos = roller_pos.copy()
        while self._compute_distance(roller_pos, target_pos) < 5:
            target_pos = self.np_random.uniform(-15, 15, size=2)

        self.env_param_channel.set_float_parameter("rollerPosX", roller_pos[0])
        self.env_param_channel.set_float_parameter("rollerPosZ", roller_pos[1])
        self.env_param_channel.set_float_parameter("targetPosX", target_pos[0])
        self.env_param_channel.set_float_parameter("targetPosZ", target_pos[1])

        self.env.reset()
        return self._get_obs()

    def seed(self, seed=1234):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min < max, "Min value is {}, while max value is {}.".format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float

    def _compute_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)


class RollerEnvV0(RollerEnv):
    def __init__(self):
        super().__init__(
            "/home/haoyuan/workspace/rfuniverse/build/Roller/RFUniverse.x86_64"
        )
