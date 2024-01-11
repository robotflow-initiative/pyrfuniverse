from pyrfuniverse.envs import RFUniverseBaseEnv
from pyrfuniverse.side_channel import SideChannel
from pyrfuniverse.side_channel import IncomingMessage
from pyrfuniverse.envs import RFUniverseGymWrapper

import numpy as np
from gym import spaces
from gym.utils import seeding
import uuid


class CollisionAndCleaningDetectionChannel(SideChannel):
    def __init__(self, agents, num_floors=64):
        super().__init__(uuid.UUID("bee25cbc-09e2-11ec-9e67-18c04d443e7d"))
        self._agent_collisions = {}
        self._agent_cleanings = {}
        self._num_collisions = 0
        self._cleaning_states = np.zeros((num_floors,), dtype=bool)
        for agent in agents:
            self._agent_collisions[agent] = 0
            self._agent_cleanings[agent] = 0

    def on_message_received(self, msg: IncomingMessage) -> None:
        self._num_collisions += msg.read_int32()
        self._cleaning_states = np.array([msg.read_float32_list()]) > 0
        for i in range(len(self._agent_collisions.items())):
            agent_name = msg.read_string()
            num_collisions = msg.read_int32()
            num_cleanings = msg.read_int32()

            if agent_name == "":
                continue
            if agent_name[-7:] == "(Clone)":
                agent_name = agent_name[:-7]
            self._agent_collisions[agent_name] += num_collisions
            self._agent_cleanings[agent_name] += num_cleanings

    def get_agent_collisions(self):
        agent_collisions = self._agent_collisions.copy()
        for key, value in self._agent_collisions.items():
            self._agent_collisions[key] = 0

        return agent_collisions

    def get_agent_cleanings(self):
        agent_cleanings = self._agent_cleanings.copy()
        for key, value in self._agent_cleanings.items():
            self._agent_cleanings[key] = 0

        return agent_cleanings

    def get_num_collisions(self):
        num_collisions = self._num_collisions
        self._num_collisions = 0

        return num_collisions

    def get_cleaning_states(self):
        return self._cleaning_states.copy()

    def clear(self):
        self._num_collisions = 0
        for key, value in self._agent_collisions.items():
            self._agent_collisions[key] = 0
            self._agent_cleanings[key] = 0
        self._cleaning_states = np.zeros(self._cleaning_states.shape, dtype=bool)


class CleanerEnv(RFUniverseGymWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        asset_bundle_file,
        max_episode_length=100,
        reset_on_collision=True,
        velocity_reward=False,
        obs_type="multi",
        grid_reward_per_step=False,
        num_agents=3,
        strength=1000,
        collision_multiplier=5,
        num_floors=64,
        executable_file=None,
    ):
        self.num_agents = num_agents
        self.num_floors = num_floors
        self.agent_names = ["agent_" + str(i) for i in range(self.num_agents)]
        self.channel = CollisionAndCleaningDetectionChannel(
            self.agent_names, self.num_floors
        )
        super().__init__(
            executable_file=executable_file,
            custom_channels=[self.channel],
            rigidbody_channel=True,
        )

        self.asset_bundle_file = asset_bundle_file
        self.max_episode_length = max_episode_length
        self.reset_on_collision = reset_on_collision
        self.strength = strength
        self.collision_multiplier = collision_multiplier
        self.velocity_reward = velocity_reward
        self.obs_type = obs_type
        self.grid_reward_per_step = grid_reward_per_step

        self.agent_name = "CleanRobot"
        self.y_offset = 0.1
        self.reset_agent_min_distance = 1.0
        self.world_range_low = np.array([-3, self.y_offset, -3])
        self.world_range_high = np.array([3, self.y_offset, 3])
        self.seed()

        self.action_space = spaces.Box(
            low=-1, high=1, shape=(2 * self.num_agents,), dtype=np.float32
        )

        if self.obs_type == "multi":
            self.observation_space = spaces.Dict(
                {
                    "agent_obs": spaces.Box(
                        low=-np.inf,
                        high=np.inf,
                        shape=(4 * self.num_agents,),
                        dtype=float,
                    ),
                    "floor_obs": spaces.MultiBinary(self.num_floors),
                }
            )
        else:
            self.observation_space = spaces.Box(
                low=-np.inf, high=np.inf, shape=(4 * self.num_agents,), dtype=float
            )
        self._env_setup()
        self.t = 0
        self.num_clean_grids = 0

    def step(self, action: np.ndarray):
        a = action.copy()

        for i in range(self.num_agents):
            target_force = np.array([a[2 * i], 0, a[2 * i + 1]]) * self.strength
            self._set_agent_force(i, target_force)

        done = False
        num_collisions = self.channel.get_num_collisions()
        cleaning_states = self.channel.get_cleaning_states().astype(np.int)
        total_num_clean_grids = np.sum(cleaning_states)

        reward = 0
        if self.grid_reward_per_step:
            assert total_num_clean_grids >= self.num_clean_grids
            reward += total_num_clean_grids - self.num_clean_grids
        else:
            reward += total_num_clean_grids / 10
        if self.velocity_reward:
            reward += self._get_total_velocity() / self.num_agents
        if not self.reset_on_collision:
            reward += -1 * num_collisions * self.collision_multiplier

        info = {"is_success": total_num_clean_grids > self.num_floors * 0.8}
        self.t += 1
        self.num_clean_grids = total_num_clean_grids

        if self.reset_on_collision and num_collisions > 0:
            reward = -1 * self.collision_multiplier * num_collisions
            self.reset()
            done = True

        elif self.t == self.max_episode_length:
            self.reset()
            done = True

        return self._get_obs(), float(reward), done, info

    def reset(self):
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

        self.t = 0
        self.num_clean_grids = 0
        self.env.reset()
        self.channel.clear()

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def _env_setup(self):
        for i in range(self.num_agents):
            agent_position = self.np_random.uniform(
                self.world_range_low, self.world_range_high
            )
            self.asset_channel.set_action(
                "LoadRigidbodyWithName",
                filename=self.asset_bundle_file,
                name=self.agent_name + " " + str(i),
                replace_name=self.agent_names[i],
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

        floor_obs = self.channel.get_cleaning_states().astype(np.int8)

        if self.obs_type == "multi":
            return {"agent_obs": agent_obs.copy(), "floor_obs": floor_obs.copy()[0]}
        else:
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
