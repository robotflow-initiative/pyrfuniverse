from pyrfuniverse.envs.gym_goal_wrapper_env import RFUniverseGymGoalWrapper
import numpy as np
from gym import spaces
from gym.utils import seeding
import math
from pyrfuniverse.utils.interpolate_utils import sine_interpolate


class FlexivCuttingEnv(RFUniverseGymGoalWrapper):
    metadata = {"render.modes": ["human", "rgb_array"]}
    object2id = {
        "flexiv": 621325,
        "ag95": 6213250,
        "target": 9413,
        "knife": 35452,
        "goal_knife": 35451,
    }

    def __init__(
        self,
        max_steps=200,
        knife_pos_min=(-0.5, 0.05, -0.07),
        knife_pos_max=(-0.6, 0.07, 0.06),
        knife_rot_min=(90, 0, -30),
        knife_rot_max=(90, 0, 30),
        offset_tolerance=0.04,
        angle_tolerance=5,
        reward_type="dense",
        consider_rotation=True,
        executable_file=None,
        assets: list = [],
    ):
        super().__init__(executable_file=executable_file, assets=assets)
        self._reload_scene()
        self.scale = 5
        self.fixed_delta_time = 0.02
        self.max_steps = max_steps
        self.knife_pos_min = np.array(knife_pos_min)
        self.knife_pos_max = np.array(knife_pos_max)
        self.knife_rot_min = np.array(knife_rot_min)
        self.knife_rot_max = np.array(knife_rot_max)
        self.offset_tolerance = offset_tolerance
        self.angle_tolerance = angle_tolerance
        self.consider_rotation = consider_rotation
        self.reward_type = reward_type

        self.seed()
        self.init_joint_positions = [65, 0, 34, -90, 0, 0, -78]
        self.init_eef_euler = [180, 0, 0]
        self.eef_euler = self.init_eef_euler.copy()
        self.goal = self._sample_goal()
        self.t = 0
        self._init_env_()
        self.action_space = spaces.Box(
            low=-1,
            high=1,
            shape=(4,) if self.consider_rotation else (3,),
            dtype=np.float32,
        )
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
        assert (action.shape == (4,) and self.consider_rotation) or (
            action.shape == (3,) and not self.consider_rotation
        )
        current_pos = self._get_eef_pos()
        delta_pos = action[:3] * 0.05
        target_pos = current_pos + delta_pos

        if self.consider_rotation:
            delta_euler = action[3] * 5
            target_euler = self.eef_euler + np.array([0, delta_euler, 0])
            self.eef_euler = np.clip(
                target_euler,
                a_min=np.array([140, -40, -40]),
                a_max=np.array([220, 40, 40]),
            )

        self._set_flexiv(target_pos, self.eef_euler)

        self.t += 1
        obs = self._get_obs()
        info = {
            "is_success": self._check_success(obs["achieved_goal"], obs["desired_goal"])
        }
        reward = self.compute_reward(obs["achieved_goal"], obs["desired_goal"], info)
        done = False

        if self.t == self.max_steps:
            done = True

        return obs, reward, done, info

    def reset(self):
        super().reset()
        self.t = 0
        self.eef_euler = self.init_eef_euler.copy()
        self._reload_scene()
        self._init_env_()
        self.goal = self._sample_goal()
        self._set_goal_knife()

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def compute_reward(self, achieved_goal, desired_goal, info):
        assert len(achieved_goal.shape) <= 2
        if len(achieved_goal.shape) < 2:
            offset = self._distance(achieved_goal[:3], desired_goal[:3])
            if self.consider_rotation:
                angle = self._angle_from_quaternions(
                    achieved_goal[3:7], desired_goal[3:7]
                )
            else:
                angle = np.array(0)
        else:
            offset = self._distance(achieved_goal[:, :3], desired_goal[:, :3])
            if self.consider_rotation:
                angle = self._angle_from_quaternions(
                    achieved_goal[:, 3:7], desired_goal[:, 3:7]
                )
            else:
                angle = np.array(0)

        if self.reward_type == "dense":
            return -(offset + angle).astype(np.float32)
        else:
            return -(offset > self.offset_tolerance).astype(np.float32) - (
                angle > self.angle_tolerance
            ).astype(np.float32)

    def heuristic(self):
        target_pos = self._get_target_pos()
        # print(target_pos, self._get_eef_pos())
        target_rot = self._get_target_rot()
        self._set_flexiv_directly(target_pos, target_rot)
        # self._step()

        achieved_quat = self._get_knife_quat()
        desired_rot = self._get_goal_knife_quat()
        # print(self.instance_channel.data[self.object2id['flexiv']]['joint_positions'])
        # print(self._angle_from_quaternions(achieved_quat, desired_rot))

    def naive_cutting(self, num_steps=100):
        curr_pos = self._get_eef_pos()
        delta_pos = np.array([0, 0.3, 0]) / self.scale
        for i in range(num_steps):
            target_pos = curr_pos - delta_pos / num_steps * (i + 1)
            self._set_flexiv_directly(target_pos, self.eef_euler)
            self._step()

    def naive_cutting_old_version(self, num_steps=100):
        eef_positions = []
        curr_target_pos = self._get_target_pos()
        delta_pos = np.array([0, 0.3, 0]) / self.scale
        for i in range(num_steps):
            target_pos = curr_target_pos - delta_pos / num_steps * (i + 1)
            self._set_target_pos(target_pos)
            self.heuristic()
            if i % 10 == 0:
                eef_positions.append(self.get_flexiv_eef_pos())

        eef_positions.append(self.get_flexiv_eef_pos())

        return eef_positions

    def replay(self, eef_positions, num_steps=20):
        times = [2.82, 3.62, 2.78, 3.15, 1.82]
        time_counter = 0
        for i in range(eef_positions.shape[0]):
            if 1 <= i <= 6:
                continue
            target_position = eef_positions[i]
            curr_position = self._get_eef_pos()
            if i > 1 and time_counter < len(times):
                this_num_steps = int(times[time_counter] / 0.05)
                time_counter += 1
            else:
                this_num_steps = num_steps
            trajectories = sine_interpolate(
                curr_position, target_position, this_num_steps
            )
            for traj in trajectories:
                self._set_flexiv(traj, self.eef_euler)

            print("Demo " + str(i + 1) + " Completed! " + str(this_num_steps))

    def move_to_joint_position(self):
        init_joint_positions = (
            np.array(
                [
                    2.56255080e-05,
                    -6.98036730e-01,
                    -2.39185442e-06,
                    -1.57088447e00,
                    2.91160322e-05,
                    6.98098183e-01,
                    -6.93326874e-05,
                ]
            )
            / math.pi
            * 180
        )
        # joint_positions[3] need to be inverse

        self.attrs[self.object2id["flexiv"]].EnabledNativeIK(False)
        self._step()
        self.attrs[self.object2id["flexiv"]].SetJointPositionDirectly(
            init_joint_positions
        )
        self._step()

    def _get_obs(self):
        eef_pos = self._get_eef_pos()
        eef_quat = self._get_eef_quat()
        eef_velocity = self._get_eef_velocity()
        robot_obs = np.concatenate((eef_pos, eef_quat, eef_velocity))

        knife_pos = self._get_knife_pos()
        knife_quat = self._get_knife_quat()
        knife_obs = np.concatenate((knife_pos, knife_quat))

        goal_knife_obs = np.concatenate((self.goal[:3], self._get_goal_knife_quat()))

        if self.consider_rotation:
            achieved_goal = knife_obs.copy()
            desired_goal = goal_knife_obs.copy()
        else:
            achieved_goal = knife_pos.copy()
            desired_goal = self.goal.copy()

        return {
            "observation": np.concatenate((robot_obs, knife_obs, goal_knife_obs)),
            "achieved_goal": achieved_goal.copy(),
            "desired_goal": desired_goal.copy(),
        }

    def _init_env_(self):
        self._init_flexiv()
        self._init_ag95()

    def _reload_scene(self):
        self.SwitchSceneAsync("FlexivSOFA", True)
        self._step()

    def _init_flexiv_target_offset(self, offset):
        self.attrs[self.object2id["flexiv"]].SetIKTargetOffset(position=offset)
        self._step()

    def _init_flexiv(self):
        self.attrs[self.object2id["flexiv"]].EnabledNativeIK(False)
        self._step()
        self.attrs[self.object2id["flexiv"]].SetJointPositionDirectly(
            joint_positions=self.init_joint_positions
        )
        self._step()
        self.attrs[self.object2id["flexiv"]].EnabledNativeIK(True)
        self._step()

        self._init_flexiv_target_offset([0, 0, 0])
        self._step()

        self._set_flexiv_directly(
            position=np.array([-0.5, 0.5, 0]), rotation=[180, 0, 0]
        )
        self._step()

        self._init_flexiv_target_offset([0, 0.26, 0])
        self._step()

    def _init_ag95(self):
        self.attrs[self.object2id["ag95"]].SetJointPositionDirectly(
            joint_positions=[55, 55]
        )
        self._step()

    def _set_flexiv(self, position, rotation=None):
        self.attrs[self.object2id["flexiv"]].IKTargetDoMove(
            position=position * self.scale,
            duration=self.fixed_delta_time,
            speed_based=False,
        )
        if rotation is not None:
            self.attrs[self.object2id["flexiv"]].IKTargetDoRotate(
                rotation=rotation, duration=self.fixed_delta_time, speed_based=False
            )
        self._step()

    def _set_target_pos(self, position):
        self.attrs[self.object2id["target"]].SetTransform(
            position=list(np.array(position) * self.scale)
        )
        self._step()

    def _get_eef_pos(self):
        return (
            np.array(self.attrs[self.object2id["ag95"]].data["positions"][9])
            / self.scale
        )

    def _get_eef_rot(self):
        return (
            np.array(self.attrs[self.object2id["ag95"]].data["rotations"][9])
            / 180
            * math.pi
        )

    def _get_eef_quat(self):
        return np.array(self.attrs[self.object2id["ag95"]].data["quaternions"][9])

    def _get_eef_velocity(self):
        return (
            np.array(self.attrs[self.object2id["ag95"]].data["velocities"][9])
            / self.scale
        )

    def get_flexiv_eef_pos(self):
        return (
            np.array(self.attrs[self.object2id["ag95"]].data["positions"][0])
            / self.scale
        )

    def _get_target_pos(self):
        return (
            np.array(self.attrs[self.object2id["target"]].data["position"]) / self.scale
        )

    def _get_target_rot(self):
        return np.array(self.attrs[self.object2id["target"]].data["rotation"])

    def _get_knife_pos(self):
        return np.array(
            np.array(self.attrs[self.object2id["knife"]].data["position"]) / self.scale
        )

    def _get_knife_quat(self):
        return np.array(self.attrs[self.object2id["knife"]].data["quaternion"])

    def _get_goal_knife_quat(self):
        return np.array(self.attrs[self.object2id["goal_knife"]].data["quaternion"])

    def _set_flexiv_directly(self, position, rotation=None):
        self.attrs[self.object2id["flexiv"]].IKTargetDoMove(
            position=position * self.scale, duration=0, speed_based=False
        )
        if rotation is not None:
            self.attrs[self.object2id["flexiv"]].IKTargetDoRotate(
                rotation=rotation, duration=0, speed_based=False
            )
        self._step()

    def _set_goal_knife(self):
        if self.consider_rotation:
            self.attrs[self.object2id["goal_knife"]].SetTransform(
                position=list(self.goal[:3] * self.scale),
                rotation=list(self.goal[3:6] * 180 / math.pi),
            )
        else:
            self.attrs[self.object2id["goal_knife"]].SetTransform(
                position=list(self.goal[:3] * self.scale)
            )
        self._step()

    def _sample_goal(self):
        sampled_pos = self.np_random.uniform(self.knife_pos_min, self.knife_pos_max)
        if self.consider_rotation:
            sampled_rot = (
                self.np_random.uniform(self.knife_rot_min, self.knife_rot_max)
                / 180
                * math.pi
            )
        else:
            sampled_rot = np.array([])

        # Fixed Demo
        # sampled_pos = np.array([-0.54, 0.05, 0.01])
        # sampled_rot = np.array([90, 0, 0]) / 180 * math.pi

        return np.concatenate((sampled_pos, sampled_rot))

    def _distance(self, pos1, pos2):
        assert pos2.shape == pos1.shape
        return np.linalg.norm(pos1 - pos2, axis=-1)

    def _angle_from_quaternions(self, quat1, quat2):
        num = np.clip(abs(np.sum(quat1 * quat2, axis=-1)), a_max=1, a_min=-1)
        return np.arccos(num) * 2 * 180 / math.pi

    def _check_success(self, achieved_goal, desired_goal):
        offset = self._distance(achieved_goal[:3], desired_goal[:3])

        if self.consider_rotation:
            angle = self._angle_from_quaternions(achieved_goal[3:7], desired_goal[3:7])
        else:
            angle = 0

        return float(offset < self.offset_tolerance and angle < self.angle_tolerance)
