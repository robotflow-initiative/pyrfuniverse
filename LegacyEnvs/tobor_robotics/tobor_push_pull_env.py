from pyrfuniverse.envs import RFUniverseGymGoalWrapper
from pyrfuniverse.utils import RFUniverseToborController
import numpy as np
from gym import spaces
from gym.utils import seeding
import math
import pybullet as p


class ToborPushPullEnv(RFUniverseGymGoalWrapper):
    metadata = {"render.modes": ["human"]}

    def __init__(
        self,
        max_steps,
        asset_bundle_file,
        pull=True,
        executable_file=None,
        threshold_angle=None,
    ):
        super().__init__(
            executable_file=executable_file,
            articulation_channel=True,
        )
        self.max_steps = max_steps
        self.pull = pull
        self.asset_bundle_file = asset_bundle_file

        self.object_name_prefix = "Microwave_"
        self.objects = [self.object_name_prefix + str(i + 1) for i in range(15)]
        self.object_position_range_low = np.array([-0.3, 0.4, 1.3])
        self.object_position_range_high = np.array([-0.1, 0.6, 1.5])
        self.tobor_action_space_low = np.array([-0.2, 0, -0.6])
        self.tobor_action_space_high = np.array([0.2, 0.4, -0.3])
        self.eef_orn = np.array([0, 0, math.pi / 2])
        self.episode_object_position = self.object_position_range_high.copy()

        self.threshold_angle = threshold_angle
        if self.threshold_angle is None:
            if self.pull:
                self.threshold_angle = 45.0
            else:
                self.threshold_angle = 15.0

        self.ik_controller = RFUniverseToborController(
            urdf_folder="/home/haoyuan/workspace/tobor",
            left_hand="robotiq85",
            right_hand="robotiq85",
            left_init_joint_positions=[0] * 7,
            right_init_joint_positions=[0] * 7,
        )
        self.seed()
        self.t = 0
        self.goal = self._sample_goal()
        self._reset_object()

        self.action_space = spaces.Box(low=-1, high=1, shape=(5,), dtype=np.float32)
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
        # Position control
        action_ctrl = action.copy()
        pos_ctrl = action_ctrl[:3] * 0.05
        curr_pos = self._get_tobor_eef_position("left")
        pos_ctrl = pos_ctrl + curr_pos
        pos_ctrl = np.clip(
            pos_ctrl,
            self.episode_object_position + self.tobor_action_space_low,
            self.episode_object_position + self.tobor_action_space_high,
        )

        # Rotation control
        rot_ctrl_x = action_ctrl[3] * 5 / math.pi
        curr_rot_x = float(self.eef_orn[0])
        rot_ctrl_x = np.clip(rot_ctrl_x + curr_rot_x, -math.pi / 3, 0)
        self.eef_orn = np.array([rot_ctrl_x, 0, math.pi / 2])

        self._set_tobor_arm(
            mode="left", position=pos_ctrl, eef_euler_angles=self.eef_orn
        )

        # Gripper width control
        gripper_width = self._get_gripper_width("left")
        gripper_width_ctrl = np.clip(gripper_width + action_ctrl[4] * 0.2, 0, 0.085)
        gripper_angle = self._compute_gripper_angle(gripper_width_ctrl)
        self._set_tobor_gripper(mode="left", gripper_angle=gripper_angle)

        self.t += 1
        obs = self._get_obs()
        done = False
        is_success = self._check_success(obs)
        info = {"is_success": is_success}
        reward = self.compute_reward(obs["achieved_goal"], obs["desired_goal"], info)
        if is_success > 0 or self.t == self.max_steps:
            done = True
            obs = self.reset()

        return obs, reward, done, info

    def reset(self):
        super().reset()
        self.env.reset()
        self.ik_controller.reset()

        self.t = 0
        self.goal = self._sample_goal()
        self._destroy_object()
        self._reset_object()

        # Set Tobor arm directly to handle to reduce exploring space
        handle_position = self._get_handle_position()
        if self.pull:
            self.eef_orn = np.array([0, 0, math.pi / 2])
        else:
            self.eef_orn = np.array([-math.pi / 3, 0, math.pi / 2])
        self._set_tobor_arm_directly("left", handle_position, list(self.eef_orn))

        return self._get_obs()

    def seed(self, seed=None):
        self.np_random, seed = seeding.np_random(seed)
        return [seed]

    def render(self, mode="human"):
        self._step()

    def compute_reward(
        self, achieved_goal: np.ndarray, desired_goal: np.ndarray, info: dict
    ):
        d = self._compute_distance(achieved_goal / 10, desired_goal / 10)
        return -d

    def _get_obs(self):
        tobor_eef_position = self._get_tobor_eef_position("left")
        tobor_eef_orientation = self.eef_orn.copy()
        tobor_eef_width = [self._get_gripper_width("left")]
        tobor_obs = np.concatenate(
            (tobor_eef_position, tobor_eef_orientation, tobor_eef_width)
        )

        handle_position = self._get_handle_position()
        door_axis_position = self._get_door_axis_position()
        open_angle = [self._get_door_open_angle()]
        object_obs = np.concatenate((handle_position, door_axis_position, open_angle))

        obs = np.concatenate((tobor_obs, object_obs))

        return {
            "observation": obs.copy(),
            "achieved_goal": np.array(open_angle),
            "desired_goal": self.goal.copy(),
        }

    def _sample_goal(self):
        return np.array([self.threshold_angle])

    def _destroy_object(self):
        self.articulation_channel.set_action(
            "Destroy",
            index=4,
        )
        self._step()

    def _reset_object(self):
        # object_idx = self.np_random.randint(0, len(self.objects))
        object_idx = 0
        object_position = self.np_random.uniform(
            self.object_position_range_low, self.object_position_range_high
        )
        self.episode_object_position = object_position.copy()
        self.asset_channel.set_action(
            "LoadArticulationBody",
            filename=self.asset_bundle_file,
            name=self.objects[object_idx],
            position=list(object_position),
            rotation=[0, 180, 0],
        )
        self._step()

        # Set init joint position
        if not self.pull:
            self.articulation_channel.set_action(
                "SetJointPositionDirectly", index=4, joint_positions=[60]
            )
        self._step()

    def _get_handle_position(self):
        handle_position = self.articulation_channel.data[4]["positions"][2]
        return np.array(handle_position)

    def _get_door_axis_position(self):
        door_axis_position = self.articulation_channel.data[4]["positions"][1]
        return np.array(door_axis_position)

    def _get_door_open_angle(self):
        return self.articulation_channel.data[4]["joint_positions"][0]

    def _get_tobor_eef_position(self, mode):
        assert mode in ["left", "right"], "Mode is either 'left' or 'right'"
        if mode == "left":
            left_eef_position = self.articulation_channel.data[1]["positions"][11]
            return np.array(left_eef_position)
        else:
            right_eef_position = self.articulation_channel.data[3]["positions"][11]
            return np.array(right_eef_position)

    def _get_gripper_width(self, mode):
        assert mode in ["left", "right"], "Mode is either 'left' or 'right'"
        idx = 1
        if mode == "right":
            idx = 3
        right_inner_finger_pos = np.array(
            self.articulation_channel.data[idx]["positions"][5]
        )
        left_inner_finger_pos = np.array(
            self.articulation_channel.data[idx]["positions"][10]
        )
        width = self._compute_distance(right_inner_finger_pos, left_inner_finger_pos)

        # The position is at the center of inner_finger, so we must get rid of the width of inner finger,
        # to get accurate gripper width
        width = width - 0.00635

        return width

    def _calculate_tobor_arm_joint_positions(self, mode, position, eef_euler_angles):
        assert mode in ["left", "right"], "Mode is either 'left' or 'right'"
        arm_index = 0
        if mode == "left":
            arm_index = 0
        elif mode == "right":
            arm_index = 2

        eef_orn = p.getQuaternionFromEuler(eef_euler_angles)
        joint_positions = self.ik_controller.calculate_ik(mode, position, eef_orn)

        return arm_index, joint_positions

    def _set_tobor_arm(self, mode, position, eef_euler_angles):
        arm_index, joint_positions = self._calculate_tobor_arm_joint_positions(
            mode, position, eef_euler_angles
        )
        self.articulation_channel.set_action(
            "SetJointPosition", index=arm_index, joint_positions=joint_positions
        )
        self._step()

    def _set_tobor_arm_directly(self, mode, position, eef_euler_angles):
        arm_index, joint_positions = self._calculate_tobor_arm_joint_positions(
            mode, position, eef_euler_angles
        )
        self.articulation_channel.set_action(
            "SetJointPositionDirectly", index=arm_index, joint_positions=joint_positions
        )
        self._step()

    def _set_tobor_gripper(self, mode, gripper_angle):
        assert mode in ["left", "right"], "Mode is either 'left' or 'right'"
        gripper_index = 0
        if mode == "left":
            gripper_index = 1
        elif mode == "right":
            gripper_index = 3

        self.articulation_channel.set_action(
            "SetJointPosition",
            index=gripper_index,
            joint_positions=[gripper_angle, gripper_angle],
        )
        self._step()

    def _compute_distance(self, point_a, point_b):
        return np.linalg.norm(point_a - point_b, axis=-1)

    def _compute_gripper_angle(self, width):
        angle_rad = 0.715 - math.asin((width - 0.01) / 0.1143)
        angle_deg = angle_rad * 180 / math.pi

        return angle_deg

    def _check_success(self, obs):
        achieved_goal = obs["achieved_goal"][0]
        desired_goal = obs["desired_goal"][0]

        if self.pull:
            success = (desired_goal < achieved_goal).astype(np.float32)
        else:
            success = (desired_goal > achieved_goal).astype(np.float32)

        return success
