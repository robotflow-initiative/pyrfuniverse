from pyrfuniverse.envs import RFUniverseBaseEnv
from pyrfuniverse.envs import RFUniverseGymWrapper
from pyrfuniverse.side_channel import SideChannel, IncomingMessage
import numpy as np
from gym import spaces
from gym.utils import seeding
import uuid


class CollisionDetectionChannel(SideChannel):
    def __init__(self):
        super().__init__(uuid.UUID("bee25cbc-07e2-11ec-9e67-18c04d443e7d"))
        self.num_collision = 0

    def on_message_received(self, msg: IncomingMessage) -> None:
        self.num_collision += msg.read_int32()

    def get_num_collision(self):
        num_collision = self.num_collision
        self.clear()

        return num_collision

    def clear(self):
        self.num_collision = 0


class MultiAgentNavigationEnv(RFUniverseGymWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        num_agents,
        asset_bundle_file,
        reset_on_collision=False,
        max_episode_length=100,
        executable_file=None,
    ):
        self.collision_detection_channel = CollisionDetectionChannel()
        super().__init__(
            executable_file=executable_file,
            custom_channels=[self.collision_detection_channel],
            rigidbody_channel=True,
        )
        self.num_agents = num_agents
        self.asset_bundle_file = asset_bundle_file
        self.max_episode_length = max_episode_length
        self.reset_on_collision = reset_on_collision

        # Fixed parameters
        self.agent_name = "NavRobot"
        self.y_offset = 0.15
        self.reset_agent_min_distance = 1.1
        self.world_range_low = np.array([-4, self.y_offset, -4])
        self.world_range_high = np.array([4, self.y_offset, 4])
        self.strength = 1000
        self.collision_multiplier = 50
        self.seed()

        self.action_space = spaces.Box(
            low=-1, high=1, shape=(2 * self.num_agents,), dtype=np.float32
        )
        self._env_setup()
        obs = self._get_obs()
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=obs.shape, dtype=np.float32
        )
        self.t = 0

        self.accumulative_collisions = 0

    def step(self, action: np.ndarray):
        a = action.copy()
        # We promise each agent will move at least 0.1m on x-axis and z-axis
        # a = a * 0.4
        # for i in range(action.shape[0]):
        #     if a[i] >= 0:
        #         a[i] += 0.1
        #     else:
        #         a[i] -= 0.1

        for i in range(self.num_agents):
            target_force = np.array([a[2 * i], 0, a[2 * i + 1]]) * self.strength
            self._set_agent_force(i, target_force)

        reward = self._get_total_velocity() / self.num_agents
        num_collisions = self.collision_detection_channel.get_num_collision()
        reward += -1 * num_collisions * self.collision_multiplier
        self.accumulative_collisions += num_collisions

        self.t += 1
        done = False
        info = {"is_success": num_collisions < 1}

        if self.reset_on_collision and num_collisions > 0:
            self.reset()
            done = True

        elif self.t == self.max_episode_length:
            self.reset()
            done = True

        return self._get_obs(), reward, done, info

    def reset(self):
        # print(self.accumulative_collisions)
        self.env.reset()
        self.t = 0
        self.accumulative_collisions = 0

        positions = []
        for i in range(self.num_agents):
            while True:
                agent_position = self.np_random.uniform(
                    self.world_range_low, self.world_range_high
                )
                if self._check_reset_position_legality(positions, agent_position):
                    break
            positions.append(agent_position.copy())
            self._set_agent_position(i, agent_position)

        # Ignore the collision when organizing agents
        self.collision_detection_channel.clear()

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def _env_setup(self):
        for _ in range(self.num_agents):
            agent_position = self.np_random.uniform(
                self.world_range_low, self.world_range_high
            )
            self.asset_channel.set_action(
                "LoadRigidbody",
                filename=self.asset_bundle_file,
                name=self.agent_name,
                position=list(agent_position),
            )
            self._step()

    def _get_obs(self):
        agent_obs = np.array([])
        for i in range(self.num_agents):
            agent_position = self.rigidbody_channel.data[i]["position"]
            agent_velocity = self.rigidbody_channel.data[i]["velocity"]
            # We ignore y-axis
            this_agent_obs = np.array(
                [
                    agent_position[0],
                    agent_position[2],
                    agent_velocity[0],
                    agent_velocity[2],
                ]
            )
            agent_obs = np.concatenate((agent_obs, this_agent_obs))

        return agent_obs.copy()

    def _set_agent_position(self, index, position):
        self.rigidbody_channel.set_action(
            "SetTransform", index=index, position=list(position), rotation=[0, 0, 0]
        )
        self._step()

    def _set_agent_force(self, index, force):
        self.rigidbody_channel.set_action("AddForce", index=index, force=force)
        self._step()

    def _get_total_velocity(self):
        total_velocity = 0
        for i in range(self.num_agents):
            velocity = self.rigidbody_channel.data[i]["velocity"]
            total_velocity += np.linalg.norm(
                np.array([velocity[0], velocity[2]]), axis=-1
            )

        return total_velocity

    def _check_reset_position_legality(self, positions, agent_pos):
        for position in positions:
            if np.linalg.norm(position - agent_pos) < self.reset_agent_min_distance:
                return False

        return True
