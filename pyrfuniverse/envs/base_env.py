from abc import ABC
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse
from pyrfuniverse.environment import UnityEnvironment
from pyrfuniverse.side_channel.environment_parameters_channel import EnvironmentParametersChannel
from pyrfuniverse.rfuniverse_channel import AssetChannel
from pyrfuniverse.rfuniverse_channel import AssetChannelExt
from pyrfuniverse.rfuniverse_channel import InstanceChannel
from pyrfuniverse.rfuniverse_channel import DebugChannel
import pyrfuniverse.attributes as attr
import gym
import os


def select_available_worker_id():
    if not os.path.exists(pyrfuniverse.user_path):
        os.makedirs(pyrfuniverse.user_path)
    log_file = os.path.join(pyrfuniverse.user_path, 'worker_id_log')

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
    log_file = os.path.join(pyrfuniverse.user_path, 'worker_id_log')

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

    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        custom_channels: list = [],
        assets: list = [],
        graphics: bool = True,
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
        self.graphics = graphics
        self.attrs = {}
        self.data = {}
        self.ext = AssetChannelExt(self)
        self._init_env()

    def _init_env(self):
        if str(self.executable_file).lower() == '@editor':
            self.env = UnityEnvironment(
                worker_id=0,
                side_channels=self.channels,
                no_graphics=not self.graphics,
            )
        elif self.executable_file is not None:
            self.env = UnityEnvironment(
                worker_id=self.worker_id,
                file_name=self.executable_file,
                side_channels=self.channels,
                no_graphics=not self.graphics,
            )
        elif os.path.exists(pyrfuniverse.executable_file):
            self.env = UnityEnvironment(
                worker_id=self.worker_id,
                file_name=pyrfuniverse.executable_file,
                side_channels=self.channels,
                no_graphics=not self.graphics,
            )
        else:
            self.env = UnityEnvironment(
                worker_id=0,
                side_channels=self.channels,
                no_graphics=not self.graphics,
            )

        if self.scene_file is not None:
            self.LoadSceneAsync(self.scene_file)
            self.data['load_done'] = False
            while not self.data['load_done']:
                self.env.step()
        if len(self.assets) > 0:
            self.PreLoadAssetsAsync(self.assets)
            self.data['load_done'] = False
            while not self.data['load_done']:
                self.env.step()
        self.env.reset()

    def _init_channels(self, kwargs: dict):
        # Compulsory channels
        # Environment parameters channel
        self.env_param_channel = EnvironmentParametersChannel()
        self.channels.append(self.env_param_channel)
        # RFUniverse channel
        self.asset_channel = AssetChannel(self, 'd587efc8-9eb7-11ec-802a-18c04d443e7d')
        self.instance_channel = InstanceChannel(self, '09bfcf57-9120-43dc-99f8-abeeec59df0f')
        self.debug_channel = DebugChannel(self, '02ac5776-6a7c-54e4-011d-b4c4723831c9')
        self.channels.append(self.asset_channel)
        self.channels.append(self.instance_channel)
        self.channels.append(self.debug_channel)

    def _step(self):
        self.env.step()

    def step(self, count: int = 1):
        for _ in range(count):
            self.env.step()

    def PreLoadAssetsAsync(self, names: list) -> None:
        msg = OutgoingMessage()
        msg.write_string('PreLoadAssetsAsync')
        count = len(names)
        msg.write_int32(count)
        for i in range(count):
            msg.write_string(names[i])
        self.asset_channel.send_message(msg)

    def LoadSceneAsync(self, file: str) -> None:
        msg = OutgoingMessage()
        msg.write_string('LoadSceneAsync')
        msg.write_string(file)
        self.asset_channel.send_message(msg)

    def SendMessage(self, message: str, *args) -> None:
        msg = OutgoingMessage()
        msg.write_string('SendMessage')
        msg.write_string(message)
        for i in args:
            if type(i) == str:
                msg.write_string(i)
            elif type(i) == bool:
                msg.write_bool(i)
            elif type(i) == int:
                msg.write_int32(i)
            elif type(i) == float:
                msg.write_float32(i)
            elif type(i) == list and type(i[0]) == float:
                msg.write_float32_list(i)
            else:
                print(f'dont support this data type:{type(i)}')
        self.asset_channel.send_message(msg)

    def AddListener(self, message: str, fun):
        if message in self.asset_channel.messages:
            if fun in self.asset_channel.messages[message]:
                self.asset_channel.messages[message].append(fun)
        else:
            self.asset_channel.messages[message] = [fun]

    def RemoveListener(self, message: str, fun):
        if message in self.asset_channel.messages:
            if fun in self.asset_channel.messages[message]:
                self.asset_channel.messages[message].remove(fun)
            if len(self.asset_channel.messages[message]) == 0:
                self.asset_channel.messages[message].pop(message)

    def InstanceObject(self, name: str, id: int, attr_type: type = attr.BaseAttr):
        assert id not in self.attrs, \
            'this ID exists'

        msg = OutgoingMessage()

        msg.write_string('InstanceObject')
        msg.write_string(name)
        msg.write_int32(id)

        self.asset_channel.send_message(msg)

        self.attrs[id] = attr_type(self, id)
        return self.attrs[id]

    def LoadURDF(self, id: int, path: str, native_ik: bool) -> attr.ControllerAttr:
        msg = OutgoingMessage()

        msg.write_string('LoadURDF')
        msg.write_int32(id)
        msg.write_string(path)
        msg.write_bool(native_ik)

        self.asset_channel.send_message(msg)

        self.attrs[id] = attr.ControllerAttr(self, id)
        return self.attrs[id]

    def LoadMesh(self, id: int, path: str) -> attr.RigidbodyAttr:
        msg = OutgoingMessage()

        msg.write_string('LoadMesh')
        msg.write_int32(id)
        msg.write_string(path)

        self.asset_channel.send_message(msg)

        self.attrs[id] = attr.RigidbodyAttr(self, id)
        return self.attrs[id]

    def IgnoreLayerCollision(self, layer1: int, layer2: int, ignore: bool) -> None:
        msg = OutgoingMessage()

        msg.write_string('IgnoreLayerCollision')
        msg.write_int32(layer1)
        msg.write_int32(layer2)
        msg.write_bool(ignore)

        self.asset_channel.send_message(msg)

    def GetCurrentCollisionPairs(self) -> None:
        msg = OutgoingMessage()
        msg.write_string('GetCurrentCollisionPairs')
        self.asset_channel.send_message(msg)

    def GetRFMoveColliders(self) -> None:
        msg = OutgoingMessage()
        msg.write_string('GetRFMoveColliders')
        self.asset_channel.send_message(msg)

    def SetGravity(self, x: float, y: float, z: float) -> None:
        msg = OutgoingMessage()

        msg.write_string('SetGravity')
        msg.write_float32(x)
        msg.write_float32(y)
        msg.write_float32(z)

        self.asset_channel.send_message(msg)

    def SetGroundPhysicMaterial(self, bounciness: float, dynamic_friction: float, static_friction: float, friction_combine: int, bounce_combine: int) -> None:
        msg = OutgoingMessage()

        msg.write_string('SetGroundPhysicMaterial')
        msg.write_float32(bounciness)
        msg.write_float32(dynamic_friction)
        msg.write_float32(static_friction)
        msg.write_int32(friction_combine)
        msg.write_int32(bounce_combine)

        self.asset_channel.send_message(msg)

    def SetTimeStep(self, delta_time: float) -> None:
        msg = OutgoingMessage()

        msg.write_string('SetTimeStep')
        msg.write_float32(delta_time)

        self.asset_channel.send_message(msg)

    def SetTimeScale(self, time_scale: float) -> None:
        msg = OutgoingMessage()

        msg.write_string('SetTimeScale')
        msg.write_float32(time_scale)

        self.asset_channel.send_message(msg)

    def SetResolution(self, resolution_x: int, resolution_y: int) -> None:
        msg = OutgoingMessage()

        msg.write_string('SetResolution')
        msg.write_int32(resolution_x)
        msg.write_int32(resolution_y)

        self.asset_channel.send_message(msg)

    def GetAttr(self, id: int):
        if id not in self.attrs:
            self.attrs[id] = attr.BaseAttr(self, id)
        return self.attrs[id]

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
