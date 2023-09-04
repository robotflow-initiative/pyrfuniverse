from pyrfuniverse.envs import RFUniverseBaseEnv
from pyrfuniverse.side_channel import SideChannel
from pyrfuniverse.side_channel import IncomingMessage
import numpy as np
import uuid
from gym import spaces
from gym.utils import seeding
from gym.utils import EzPickle
import pettingzoo
from pettingzoo.utils.conversions import parallel_wrapper_fn, to_parallel
from pettingzoo import AECEnv
from pettingzoo.utils import agent_selector
from pettingzoo.utils import wrappers
import os
import time


class CollisionDetectionChannel(SideChannel):
    def __init__(self, possible_agents):
        super().__init__(uuid.UUID("bee25cbc-07e2-11ec-9e67-18c04d443e7d"))
        self.agent_collisions = {}
        for agent in possible_agents:
            self.agent_collisions[agent] = 0

    def on_message_received(self, msg: IncomingMessage) -> None:
        for i in range(len(self.agent_collisions.items())):
            agent_name = msg.read_string()
            num_collisions = msg.read_int32()

            if agent_name == "":
                continue
            if agent_name[-7:] == "(Clone)":
                agent_name = agent_name[:-7]
            self.agent_collisions[agent_name] += num_collisions

        # print(self.agent_collisions)

    def get_agent_collisions(self):
        agent_collisions = self.agent_collisions.copy()
        self.clear()

        return agent_collisions

    def clear(self):
        for key, value in self.agent_collisions.items():
            self.agent_collisions[key] = 0


def env(**kwargs):
    env = raw_env(**kwargs)
    # if env.continuous:
    # env = wrappers.ClipOutOfBoundsWrapper(env)
    # else:
    # env = wrappers.AssertOutOfBoundsWrapper(env)
    env = wrappers.OrderEnforcingWrapper(env)
    return env


parallel_env = parallel_wrapper_fn(env)


class raw_env(AECEnv, RFUniverseBaseEnv, EzPickle):
    metadata = {"render.modes": ["human"], "name": "navigation_v1"}
    reward_range = (-float("inf"), float("inf"))

    def __init__(
        self,
        num_agents,
        asset_bundle_file,
        log_dir,
        reset_on_collision=False,
        max_episode_length=100,
        executable_file=None,
        log_monitor=True,
        strength=1000,
        collision_multiplier=5,
    ):
        """
        The init method takes in environment arguments and
         should define the following attributes:
        - possible_agents
        - action_spaces
        - observation_spaces

        These attributes should not be changed after initialization.
        """
        EzPickle.__init__(
            self,
            num_agents,
            asset_bundle_file,
            log_dir,
            reset_on_collision,
            max_episode_length,
            executable_file,
            log_monitor,
        )

        self.possible_agents = ["agent_" + str(r) for r in range(num_agents)]
        self.collision_detection_channel = CollisionDetectionChannel(
            self.possible_agents
        )
        RFUniverseBaseEnv.__init__(
            self,
            executable_file=executable_file,
            custom_channels=[self.collision_detection_channel],
            rigidbody_channel=True,
        )

        self.asset_bundle_file = asset_bundle_file
        self.reset_on_collision = reset_on_collision
        self.max_episode_length = max_episode_length
        self.log_dir = log_dir
        self.log_monitor = log_monitor
        self.collision_multiplier = collision_multiplier
        self.strength = strength

        if self.log_monitor:
            os.makedirs(self.log_dir, exist_ok=True)
            self.monitor_id = 0
            while os.path.exists(
                os.path.join(self.log_dir, "{}.monitor.csv".format(self.monitor_id))
            ):
                self.monitor_id += 1
            with open(
                os.path.join(self.log_dir, "{}.monitor.csv".format(self.monitor_id)),
                "a+",
            ) as f:
                f.write("r,l,t\n")
        self.counter = 0
        self.total_reward = 0
        self.start_time = time.time()

        # Fixed parameters
        self.agent_name = "NavRobot"
        self.y_offset = 0.15
        self.reset_agent_min_distance = 1.1
        self.world_range_low = np.array([-4, self.y_offset, -4])
        self.world_range_high = np.array([4, self.y_offset, 4])

        self.agent_name_mapping = dict(
            zip(self.possible_agents, list(range(len(self.possible_agents))))
        )
        # print(self.agent_name_mapping)

        self.action_spaces = {
            agent: spaces.Box(low=-1, high=1, shape=(2,), dtype=np.float32)
            for agent in self.possible_agents
        }
        self.observation_spaces = {
            agent: spaces.Box(
                low=-np.inf, high=np.inf, shape=(num_agents * 4,), dtype=np.float32
            )
            for agent in self.possible_agents
        }

        self.seed()
        self._env_setup(num_agents)
        self.t = 0

    def observe(self, agent):
        """
        Observe should return the observation of the specified agent. This function
        should return a sane observation (though not necessarily the most up to date possible)
        at any time after reset() is called.
        """
        obs = self._get_obs_for_agent(agent)
        return obs

    def reset(self):
        """
        Reset needs to initialize the following attributes
        - agents
        - rewards
        - _cumulative_rewards
        - dones
        - infos
        - agent_selection
        And must set up the environment so that render(), step(), and observe()
        can be called without issues.

        Here it sets up the state dictionary which is used by step() and the observations dictionary which is used by step() and observe()
        """
        self.agents = self.possible_agents[:]
        self.rewards = {agent: 0 for agent in self.agents}
        self._cumulative_rewards = {agent: 0 for agent in self.agents}
        self.dones = {agent: False for agent in self.agents}
        self.infos = {agent: {} for agent in self.agents}

        self.t = 0
        self._env_reset()
        self.collision_detection_channel.clear()
        """
        Our agent_selector utility allows easy cyclic stepping through the agents list.
        """
        self._agent_selector = agent_selector(self.agents)
        self.agent_selection = self._agent_selector.next()

    def step(self, action):
        """
        step(action) takes in an action for the current agent (specified by
        agent_selection) and needs to update
        - rewards
        - _cumulative_rewards (accumulating the rewards)
        - dones
        - infos
        - agent_selection (to the next agent)
        And any internal state used by observe() or render()
        """
        if self.dones[self.agent_selection]:
            # handles stepping an agent which is already done
            # accepts a None action for the one agent, and moves the agent_selection to
            # the next done agent,  or if there are no more done agents, to the next live agent
            return self._was_done_step(action)

        agent = self.agent_selection

        # the agent which stepped last had its _cumulative_rewards accounted for
        # (because it was returned by last()), so the _cumulative_rewards for this
        # agent should start again at 0
        self._cumulative_rewards[agent] = 0

        self._set_agent_force(
            self.agent_name_mapping[agent],
            np.array([action[0], 0, action[1]]) * self.strength,
        )

        done = False
        # collect reward if it is the last agent to act
        if self._agent_selector.is_last():
            agent_collisions = self.collision_detection_channel.get_agent_collisions()
            # print(agent_collisions)
            # rewards for all agents are placed in the .rewards dictionary
            for agent_name in self.agents:
                velocity = self._get_velocity_for_agent(agent_name)
                reward = (
                    velocity - agent_collisions[agent_name] * self.collision_multiplier
                )
                self.rewards[agent_name] = reward

            self.t += 1
            # The dones dictionary must be updated for all players.
            done = self.t >= self.max_episode_length
            self.dones = {agent: done for agent in self.agents}
        else:
            self._clear_rewards()

        # selects the next agent.
        self.agent_selection = self._agent_selector.next()
        # Adds .rewards to ._cumulative_rewards
        self._accumulate_rewards()

        if self.log_monitor and done:
            self._log()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def close(self):
        RFUniverseBaseEnv.close(self)

    def _env_setup(self, num_agents):
        for i in range(num_agents):
            agent_position = self.np_random.uniform(
                self.world_range_low, self.world_range_high
            )
            self.asset_channel.set_action(
                "LoadRigidbodyWithName",
                filename=self.asset_bundle_file,
                name=self.agent_name,
                replace_name=self.possible_agents[i],
                position=list(agent_position),
            )
            self._step()

    def _env_reset(self):
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

    def _set_agent_position(self, index, position):
        self.rigidbody_channel.set_action(
            "SetTransform", index=index, position=list(position), rotation=[0, 0, 0]
        )
        self._step()

    def _set_agent_force(self, index, force):
        self.rigidbody_channel.set_action("AddForce", index=index, force=force)
        self._step()

    def _check_reset_position_legality(self, positions, agent_pos):
        for position in positions:
            if np.linalg.norm(position - agent_pos) < self.reset_agent_min_distance:
                return False

        return True

    def _get_obs_for_agent(self, agent):
        agent_id = self.agent_name_mapping[agent]
        self_pos = self.rigidbody_channel.data[agent_id]["position"]
        self_vel = self.rigidbody_channel.data[agent_id]["velocity"]
        self_obs = np.array([self_pos[0], self_pos[2], self_vel[0], self_vel[2]])

        other_all_obs = np.array([])
        for i in range(self.num_agents):
            if i == agent_id:  # Ignore itself
                continue
            other_pos = self.rigidbody_channel.data[i]["position"]
            other_vel = self.rigidbody_channel.data[i]["velocity"]
            other_obs = (
                np.array([other_pos[0], other_pos[2], other_vel[0], other_vel[2]])
                - self_obs
            )
            other_all_obs = np.concatenate((other_all_obs, other_obs))

        return np.concatenate((self_obs, other_all_obs)).copy()

    def _get_velocity_for_agent(self, agent):
        agent_id = self.agent_name_mapping[agent]
        self_vel = self.rigidbody_channel.data[agent_id]["velocity"]

        return np.linalg.norm(np.array([self_vel[0], self_vel[2]]))

    def _log(self):
        self.counter += self.max_episode_length
        total_reward = 0
        for agent_name in self.agents:
            total_reward += self.rewards[agent_name]
        with open(
            os.path.join(self.log_dir, "{}.monitor.csv".format(self.monitor_id)), "a+"
        ) as f:
            f.write(
                "{},{},{}\n".format(
                    total_reward, self.counter, time.time() - self.start_time
                )
            )
