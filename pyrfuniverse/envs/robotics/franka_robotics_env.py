from pyrfuniverse.envs import RFUniverseGymGoalWrapper
from pyrfuniverse.utils import RFUniverseController
import numpy as np
from gym import spaces
from gym.utils import seeding
import copy


class FrankaRoboticsEnv(RFUniverseGymGoalWrapper):
    metadata = {'render.modes': ['human']}
    # height_offset is just the object width / 2, which is the object's stable height value.
    height_offset = 0.42

    def __init__(
            self,
            max_episode_length,
            reward_type,
            tolerance,
            load_object,
            target_in_air,
            block_gripper,
            target_xz_range,
            target_y_range,
            object_xz_range,
            seed=1234,
            executable_file=None,
            scene_file=None,
            asset_bundle_file=None,
            assets: list = []

    ):
        super().__init__(
            executable_file,
            scene_file,
            assets=assets
        )
        self.max_steps = max_episode_length
        self.reward_type = reward_type
        self.tolerance = tolerance
        self.load_object = load_object
        self.target_in_air = target_in_air
        self.block_gripper = block_gripper
        self.goal_range_low = np.array([-target_xz_range, self.height_offset, -target_xz_range])
        self.goal_range_high = np.array([target_xz_range, target_y_range, target_xz_range])
        self.object_range_low = np.array([-object_xz_range, self.height_offset, -object_xz_range])
        self.object_range_high = np.array([object_xz_range, self.height_offset, object_xz_range])
        self.asset_bundle_file = asset_bundle_file

        self.seed(seed)
        self._env_setup()
        self.ik_controller = RFUniverseController('franka', base_pos=np.array([-0.6, 0, 0]))
        self.t = 0
        self.goal = self._sample_goal()
        self.action_space = spaces.Box(
            low=-1, high=1, shape=(4,), dtype=np.float32
        )

        obs = self._get_obs()
        self.observation_space = spaces.Dict({
            'observation': spaces.Box(-np.inf, np.inf, shape=obs['observation'].shape, dtype=np.float32),
            'desired_goal': spaces.Box(-np.inf, np.inf, shape=obs['desired_goal'].shape, dtype=np.float32),
            'achieved_goal': spaces.Box(-np.inf, np.inf, shape=obs['achieved_goal'].shape, dtype=np.float32)
        })

    def step(self, action: np.ndarray):
        """
        Params:
            action: 4-d numpy array.
        """
        # print(action)
        pos_ctrl = action[:3] * 0.05
        curr_pos = np.array(self.instance_channel.data[9658740]['positions'][3])
        # print(curr_pos)
        pos_ctrl = curr_pos + pos_ctrl

        # print(pos_ctrl)

        joint_positions = self.ik_controller.calculate_ik(pos_ctrl)
        if self.block_gripper:
            joint_positions.append(0)
        else:
            curr_gripper_width = self._get_gripper_width()
            target_gripper_width = curr_gripper_width + action[3] * 0.2
            target_gripper_width = np.clip(target_gripper_width, 0, 0.08)
            joint_positions.append(target_gripper_width)
        self._set_franka_joints(np.array(joint_positions))
        self.t += 1

        obs = self._get_obs()
        done = False
        info = {
            'is_success': self._check_success(obs)
        }
        reward = self.compute_reward(obs['achieved_goal'], obs['desired_goal'], info)

        # if self.t == self.max_steps:
        #     done = True

        return obs, reward, done, info

    def reset(self):
        super().reset()
        self.env.reset()
        self.t = 0
        self.goal = self._sample_goal()
        object_pos = None

        if self.load_object:
            object_pos = self._reset_object()

        self.instance_channel.set_action(
            'SetTransform',
            id=0,
            position=list(self.goal)
        )
        self._step()

        self.ik_controller.reset()

        if self.load_object and self.target_in_air:
            # Move the robot arm to the object's position, which can accelerate training process
            joint_positions = self.ik_controller.calculate_ik(object_pos)
            self.instance_channel.set_action(
                'SetJointPositionDirectly',
                id=965874,
                joint_positions=list(joint_positions),
            )
            self.instance_channel.set_action(
                'SetJointPositionDirectly',
                id=9658740,
                joint_positions=[-0.04, -0.04],
            )
            self._step()

        return self._get_obs()

    def seed(self, seed=1234):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode='human'):
        self._step()

    def compute_reward(self, achieved_goal, desired_goal, info):
        distance = self._compute_goal_distance(achieved_goal, desired_goal)
        if self.reward_type == 'sparse':
            return -(distance > self.tolerance).astype(np.float32)
        else:
            return -distance

    def _get_obs(self):
        gripper_position = np.array(self.instance_channel.data[9658740]['positions'][3])
        gripper_velocity = np.array(self.instance_channel.data[9658740]['velocities'][3])
        gripper_width = self._get_gripper_width()
        # gripper_joint_position = np.array(self.articulation_channel.data[1]['joint_positions'])
        # gripper_joint_velocity = np.array(self.articulation_channel.data[1]['joint_velocities'])

        panda_obs = np.concatenate((gripper_position, gripper_velocity, [gripper_width]))

        if self.load_object:
            object_position = np.array(self.instance_channel.data[0]['position'])
            object_rotation = np.array(self.instance_channel.data[0]['rotation'])
            object_velocity = np.array(self.instance_channel.data[0]['velocity'])
            object_angular_vel = np.array(self.instance_channel.data[0]['angular_vel'])
            # object_rel_pos = object_position - gripper_position
            # object_velocity = object_velocity - gripper_velocity
            achieved_goal = object_position.copy()
        else:
            # object_position = object_rotation = object_velocity = object_angular_vel = object_rel_pos = np.zeros(0)
            object_position = object_rotation = object_velocity = object_angular_vel = np.zeros(0)
            achieved_goal = gripper_position.copy()

        object_obs = np.concatenate((object_position, object_rotation, object_velocity, object_angular_vel))

        # obs = np.concatenate(
        #     [gripper_position, object_position, object_rel_pos, gripper_joint_position, object_rotation,
        #     object_velocity, object_angular_vel, gripper_velocity, gripper_joint_velocity]
        # )
        obs = np.concatenate((panda_obs, object_obs))

        return {
            'observation': obs.copy(),
            'achieved_goal': achieved_goal.copy(),
            'desired_goal': self.goal.copy()
        }

    def _env_setup(self):
        if self.load_object:
            self._load_object()
        else:
            # Align the time step
            self._step()
        self._step()

    def _generate_random_float(self, min: float, max: float) -> float:
        assert min < max, \
            'Min value is {}, while max value is {}.'.format(min, max)
        random_float = np.random.rand()
        random_float = random_float * (max - min) + min

        return random_float

    def _set_franka_joints(self, a: np.ndarray):
        self.instance_channel.set_action(
            'SetJointPosition',
            id=965874,
            joint_positions=list(a[0:7]),
        )
        self._step()

        a[7] = -1 * a[7] / 2
        self.instance_channel.set_action(
            'SetJointPosition',
            id=9658740,
            joint_positions=[a[7], a[7]],
        )
        self._step()

    def _get_gripper_width(self):
        gripper_joint_positions = copy.deepcopy(self.instance_channel.data[9658740]['joint_positions'])
        return -1 * (gripper_joint_positions[0] + gripper_joint_positions[1])

    def _check_success(self, obs):
        achieved_goal = obs['achieved_goal']
        desired_goal = obs['desired_goal']
        distance = self._compute_goal_distance(achieved_goal, desired_goal)

        return (distance < self.tolerance).astype(np.float32)

    def _compute_goal_distance(self, goal_a, goal_b):
        assert goal_a.shape == goal_b.shape
        return np.linalg.norm(goal_a - goal_b, axis=-1)

    def _load_object(self):
        '''
        assert self.asset_bundle_file is not None, \
            'There must be an asset bundle file to load.'

        object_name = 'robotics_object'
        '''
        self.asset_channel.set_action(
            'InstanceObject',
            name='Rigidbody_Box',
            id=0
        )
        self.instance_channel.set_action(
            'SetTransform',
            id=0,
            position=[0, self.height_offset, 0],
            scale=[0.05, 0.05, 0.05],
        )
        self._step()

    def _sample_goal(self):
        if self.load_object:
            goal = np.array([0.0, 0.4, 0.0])
            noise = self.np_random.uniform(self.goal_range_low, self.goal_range_high)
            if self.target_in_air and self.np_random.random() < 0.3:
                noise[1] = self.height_offset
            goal += noise
        else:
            goal = self.np_random.uniform(self.goal_range_low, self.goal_range_high)

        return goal.copy()

    def _reset_object(self):
        object_pos = self.np_random.uniform(self.object_range_low, self.object_range_high)

        self.instance_channel.set_action(
            'SetTransform',
            id=0,
            position=list(object_pos),
            rotation=[0, 0, 0]
        )
        self._step()

        return object_pos.copy()
