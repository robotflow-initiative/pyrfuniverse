import time
from abc import ABC
from pyrfuniverse.environment import UnityEnvironment
from pyrfuniverse.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from pyrfuniverse.rfuniverse_channel import RigidbodyChannel
from pyrfuniverse.rfuniverse_channel import CameraChannel
from pyrfuniverse.rfuniverse_channel import ArticulationChannel
from pyrfuniverse.rfuniverse_channel import GameObjectChannel
from pyrfuniverse.rfuniverse_channel import ObiClothChannel
from pyrfuniverse.rfuniverse_channel import ObiClothWithGraspingChannel
from pyrfuniverse.rfuniverse_channel import ObiSoftbodyChannel
from pyrfuniverse.rfuniverse_channel import HumanDressingChannel
from pyrfuniverse.rfuniverse_channel import AssetChannel
from pyrfuniverse.rfuniverse_channel import InstanceChannel
from pyrfuniverse.rfuniverse_channel import DebugChannel
import gym
import os
import platform
import json
from typing import Union


def get_rfuniverse_log_dir():
    platform_name = platform.platform()
    assert 'Linux' in platform_name or 'Windows' in platform_name, \
        'Currently, we only support Linux and Windows.'

    rfuniverse_log_dir = ''
    curr_path = os.getcwd()

    if 'Linux' in platform_name:
        paths = curr_path.split('/')
        assert len(paths) >= 3 and paths[1] == 'home', \
            'Invalid path. Please set your path to /home/USER_NAME/xxx'

        rfuniverse_log_dir = '/home/{}/.rfuniverse'.format(paths[2])
    else:
        rfuniverse_log_dir = os.path.join(curr_path, '.rfuniverse')

    return rfuniverse_log_dir


def select_available_worker_id():
    worker_id_log_dir = get_rfuniverse_log_dir()
    if not os.path.exists(worker_id_log_dir):
        os.makedirs(worker_id_log_dir)
    log_file = os.path.join(worker_id_log_dir, 'worker_id_log')

    worker_id = 1
    worker_id_in_use = []
    if os.path.exists(log_file):
        with open(log_file, 'r') as f:
            worker_ids = f.readlines()
            for line in worker_ids:
                worker_id_in_use.append(int(line))
        while worker_id in worker_id_in_use:
            worker_id += 1

    worker_id_in_use.append(worker_id)
    with open(log_file, 'w') as f:
        for id in worker_id_in_use:
            f.write(str(id) + '\n')

    return worker_id


def delete_worker_id(worker_id):
    worker_id_log_dir = get_rfuniverse_log_dir()
    log_file = os.path.join(worker_id_log_dir, 'worker_id_log')

    worker_id_in_use = []
    if os.path.exists(log_file):
        with open(log_file, 'r') as f:
            worker_ids = f.readlines()
            for line in worker_ids:
                worker_id_in_use.append(int(line))

    worker_id_in_use.remove(worker_id)
    with open(log_file, 'w') as f:
        for id in worker_id_in_use:
            f.write(str(id) + '\n')


class RFUniverseBaseEnv(ABC):
    """
    This class is the base class for RFUniverse environments. In RFUniverse, every environment will be
    packaged in the Gym-like environment class. For custom environments, users will have to implement
    step(), reset(), seed(), _get_obs().
    """

    metadata = {'render.modes': ['human', 'rgb_array']}
    rfuniverse_channel_ids = {
        'instance_channel':                     '09bfcf57-9120-43dc-99f8-abeeec59df0f',
        'asset_channel':                        'd587efc8-9eb7-11ec-802a-18c04d443e7d',
        'debug_channel':                        '02ac5776-6a7c-54e4-011d-b4c4723831c9',
    }

    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        custom_channels: list = [],
        assets: list = [],
        **kwargs
    ):
        # time step
        self.t = 0
        self.worker_id = select_available_worker_id()
        # initialize rfuniverse channels
        self.channels = custom_channels.copy()
        self._init_channels(kwargs)
        self.assets = assets
        # initialize environment
        self.executable_file = executable_file
        self.scene_file = scene_file
        self._init_env()

    def _init_env(self):
        if self.executable_file is not None:
            self.env = UnityEnvironment(
                worker_id=self.worker_id,
                file_name=self.executable_file,
                side_channels=self.channels,
            )
        else:
            self.env = UnityEnvironment(
                worker_id=0,
                side_channels=self.channels
            )

        if self.scene_file is not None:
            self.asset_channel.LoadSceneAsync(self.scene_file)
            self.asset_channel.done = False
            while not self.asset_channel.done:
                self._step()
        if len(self.assets) > 0:
            self.asset_channel.PreLoadAssetsAsync(self.assets)
            self.asset_channel.done = False
            while not self.asset_channel.done:
                self._step()
        self.env.reset()

    def _init_channels(self, kwargs: dict):
        # Compulsory channels
        # Environment parameters channel
        self.env_param_channel = EnvironmentParametersChannel()
        self.channels.append(self.env_param_channel)
        # Asset channel
        self.asset_channel = AssetChannel(self.rfuniverse_channel_ids['asset_channel'])
        self.instance_channel = InstanceChannel(self.rfuniverse_channel_ids['instance_channel'], self)
        self.debug_channel = DebugChannel(self.rfuniverse_channel_ids['debug_channel'])
        self.channels.append(self.asset_channel)
        self.channels.append(self.instance_channel)
        self.channels.append(self.debug_channel)

        if 'camera_channel' in kwargs.keys():
            if kwargs['camera_channel'] is True or type(kwargs['camera_channel']) is str:
                self.camera_channel = CameraChannel(self.instance_channel)
            else:
                self.camera_channel = None
        else:
            self.camera_channel = None

        if 'rigidbody_channel' in kwargs.keys():
            if kwargs['rigidbody_channel'] is True or type(kwargs['rigidbody_channel']) is str:
                self.rigidbody_channel = RigidbodyChannel(self.instance_channel)
            else:
                self.rigidbody_channel = None
        else:
            self.rigidbody_channel = None

        if 'articulation_channel' in kwargs.keys():
            if kwargs['articulation_channel'] is True or type(kwargs['articulation_channel']) is str:
                self.articulation_channel = ArticulationChannel(self.instance_channel)
            else:
                self.articulation_channel = None
        else:
            self.articulation_channel = None

        if 'game_object_channel' in kwargs.keys():
            if kwargs['game_object_channel'] is True or type(kwargs['game_object_channel']) is str:
                self.game_object_channel = GameObjectChannel(self.instance_channel)
            else:
                self.game_object_channel = None
        else:
            self.game_object_channel = None

        if 'obi_cloth_channel' in kwargs.keys():
            if kwargs['obi_cloth_channel'] is True or type(kwargs['obi_cloth_channel']) is str:
                self.obi_cloth_channel = ObiClothChannel(self.instance_channel)
            else:
                self.obi_cloth_channel = None
        else:
            self.obi_cloth_channel = None

        if 'obi_cloth_with_grasping_channel' in kwargs.keys():
            if kwargs['obi_cloth_with_grasping_channel'] is True or type(kwargs['obi_cloth_with_grasping_channel']) is str:
                self.obi_cloth_with_grasping_channel = ObiClothWithGraspingChannel(self.instance_channel)
            else:
                self.obi_cloth_with_grasping_channel = None
        else:
            self.obi_cloth_with_grasping_channel = None

        if 'obi_softbody_channel' in kwargs.keys():
            if kwargs['obi_softbody_channel'] is True or type(kwargs['obi_softbody_channel']) is str:
                self.obi_softbody_channel = ObiSoftbodyChannel(self.instance_channel)
            else:
                self.obi_softbody_channel = None
        else:
            self.obi_softbody_channel = None

        if 'human_dressing_channel' in kwargs.keys():
            if kwargs['human_dressing_channel'] is True or type(kwargs['human_dressing_channel']) is str:
                self.human_dressing_channel = HumanDressingChannel(self.instance_channel)
            else:
                self.human_dressing_channel = None
        else:
            self.human_dressing_channel = None

        if 'debug_channel' in kwargs.keys():
            if kwargs['debug_channel'] is True or type(kwargs['debug_channel']) is str:
                self.debug_channel = DebugChannel(
                    self.rfuniverse_channel_ids['debug_channel']
                )
                self.channels.append(self.debug_channel)
            else:
                self.debug_channel = None
        else:
            self.debug_channel = None

    def _step(self):
        self.env.step()

    def render(
            self,
            id,
            mode='human',
            width=512,
            height=512,
            target_position=None,
            target_euler_angles=None
    ):
        """
        Render an image with given resolution, target position and target euler angles.
        TODO: Current version only support RoboTube, which only needs RGB image. For depth, normal, ins_seg, optical
            flow, etc., please refer to `camera_channel.py` for more actions.

        Args:
            id: Int. Camera ID.
            mode: Str. OpenAI-Gym style mode.
            width: Int. Optional. The width of target image.
            height: Int. Optional. The height of target image.
            target_position: List. Optional. The target position of this camera, in [X, Y, Z] order.
            target_euler_angles: List. Optional. The target euler angles of this camera, in [X, Y, Z] order.
                Each element is in degree, not radius.

        Returns:
            A numpy array with size (width, height, 3). Each pixel is in [R, G, B] order.
        """
        assert self.camera_channel is not None, \
            'There is no camera available in this scene. Please check.'

        target_position = list(target_position) if target_position is not None else None
        target_euler_angles = list(target_euler_angles) if target_euler_angles is not None else None
        '''
        self.camera_channel.set_action(
            'SetTransform',
            id=id,
            position=target_position,
            rotation=target_euler_angles,
        )'''
        #self._step()

        self.camera_channel.set_action(
            'GetImages',
            rendering_params=[[id, width, height]]
        )
        self._step()

        img = self.camera_channel.images.pop(0)
        return img

    def close(self):
        delete_worker_id(self.worker_id)
        self.env.close()


class RFUniverseGymWrapper(RFUniverseBaseEnv, gym.Env):

    def __init__(
            self,
            executable_file: str = None,
            scene_file: str = None,
            custom_channels: list = [],
            assets: list = [],
            **kwargs
    ):
        RFUniverseBaseEnv.__init__(
            self,
            executable_file=executable_file,
            scene_file=scene_file,
            custom_channels=custom_channels,
            assets=assets,
            **kwargs,
        )

    def close(self):
        RFUniverseBaseEnv.close(self)


class RFUniverseGymGoalWrapper(gym.GoalEnv, RFUniverseBaseEnv):

    def __init__(
            self,
            executable_file: str = None,
            scene_file: str = None,
            custom_channels: list = [],
            assets: list = [],
            **kwargs

    ):
        RFUniverseBaseEnv.__init__(
            self,
            executable_file=executable_file,
            scene_file=scene_file,
            custom_channels=custom_channels,
            assets=assets,
            **kwargs,
        )

    def reset(self):
        gym.GoalEnv.reset(self)

    def close(self):
        RFUniverseBaseEnv.close(self)
