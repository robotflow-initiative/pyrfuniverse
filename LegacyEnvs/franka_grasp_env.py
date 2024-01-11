from pyrfuniverse.envs import RFUniverseBaseEnv
import numpy as np


class FrankaGraspEnv(RFUniverseBaseEnv):
    def __init__(self, executable_file=None):
        super().__init__(
            executable_file,
            camera_channel=True,
            rigidbody_channel=True,
            articulation_channel=True,
        )
        self.camera_channel.set_action(
            "AddCamera", position=[0, 0, 0], rotation=[0, 0, 0], parent_camera_idx=0
        )
        self._step()

        self.camera_channel.set_action(
            "AddCamera", position=[0, 0, 0], rotation=[0, 0, 0], parent_camera_idx=0
        )
        self._step()

        self.camera_channel.set_action("ResetCamera", index=2, position=[0, 0.1, 0])
        self._step()

        # self.asset_channel.set_action(
        #     'LoadRigidbody',
        #     filename='D:\\GitHub\\rfuniverse\\RFUniverse\\Assets\\AssetBundles\\rigidbody',
        #     name='011_banana',
        #     position=[-0.612053275,0.628000021,-0.356999993]
        # )
        # self._step()

        self.robot_joint_positions = np.array([0.0 for i in range(8)], dtype=float)

    def step(self, a: np.ndarray):
        """
        Params:
            a: 8-d or 16-d numpy array. The first 7 dimensions are for each joint's position of Franka (-180~180),
               while the 8th dimension is for gripper's width. If the total dimension is 16, the last
               8 dimensions are the speed scales. Otherwise, the speed scale will be default to 1.0
        """
        assert a.shape == (16,) or a.shape == (
            8,
        ), "The shape of action must be (14,) or (7,), but got {}".format(a.shape)
        if a.shape == (8,):
            a = np.concatenate((a, np.array([1.0 for i in range(8)], dtype=float)))

        self._set_franka_joints(a)
        self._wait_for_moving()
        self.t += 1

        return self._get_obs()

    def _get_obs(self):
        # Read message from channels
        return self.rigidbody_channel.data

    def reset(self):
        self.t = 0
        self.r = 0
        self.env.reset()

        return self._get_obs()

    def seed(self):
        pass

    def _set_franka_joints(self, a: np.ndarray):
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=0,
            joint_positions=list(a[0:7]),
            speed_scales=list(a[8:15]),
        )
        self._step()

        # Since a[7] is gripper's width, convert it to joint position value
        a[7] = -1 * a[7] / 2

        self.articulation_channel.set_action(
            "SetJointPosition",
            index=1,
            joint_positions=[a[7], a[7]],
            speed_scales=[a[15], a[15]],
        )
        self._step()

    def _control_gripper(self, joint_positions: np.ndarray, speed: np.ndarray):
        self.articulation_channel.set_action(
            "SetJointPosition",
            index=1,
            joint_positions=list(joint_positions),
            speed_scales=list(speed),
        )

    def _open_gripper(self, speed: float = 1.0):
        self._control_gripper(
            joint_positions=np.array([-0.04, -0.04], dtype=float),
            speed=np.array([speed, speed], dtype=float),
        )

    def _close_gripper(self, speed: float = 1.0):
        self._control_gripper(
            joint_positions=np.array([0, 0], dtype=float),
            speed=np.array([speed, speed], dtype=float),
        )

    def _wait_for_moving(self):
        self.articulation_channel.data.clear()
        while not 0 in self.articulation_channel.data.keys():
            self._step()
        while not (
            self.articulation_channel.data[0]["all_stable"]
            and self.articulation_channel.data[1]["all_stable"]
        ):
            self._step()
