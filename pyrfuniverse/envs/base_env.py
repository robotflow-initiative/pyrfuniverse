import random
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
    RFUniverse base environment class.

    Args:
        executable_file: Str, the absolute path of Unity executable file. None for last used executable file; "@editor" for using Unity Editor.
        scene_file: Str, the absolute path of Unity scene JSON file. All JSON files locate at `StraemingAssets/SceneData` by default.
        assets: List, the list of pre-load assets. All assets in the list will be pre-loaded in Unity when the environment is initialized, which will save time during instanciating.
        graphics: Bool, True for showing GUI and False for headless mode.
    """
    metadata = {'render.modes': ['human', 'rgb_array']}

    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        custom_channels=None,
        assets=None,
        graphics: bool = True,
        **kwargs
    ):
        if custom_channels is None:
            custom_channels = []
        if assets is None:
            assets = []
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
        self._SendVersion()
        if len(self.assets) > 0:
            self._PreLoadAssetsAsync(self.assets, True)
        if self.scene_file is not None:
            self.LoadSceneAsync(self.scene_file, True)
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
        """
        Send the messages of called functions to Unity and simulate for a step, then accept the data from Unity.

        Args:
            count: the number of steps for executing Unity simulation.
        """
        if count < 1:
            count = 1
        for _ in range(count):
            self.env.step()

    def close(self):
        """
        Close the environment
        """
        delete_worker_id(self.worker_id)
        self.env.close()

    def GetAttr(self, id: int):
        """
        Get the attribute instance by object id.

        Args:
            id: Int, object id.

        Returns:
            pyrfuniverse.attributes.BaseAttr: An instance of attribute.
        """
        if id not in self.attrs:
            self.attrs[id] = attr.BaseAttr(self, id)
        return self.attrs[id]

    #Env API
    def _PreLoadAssetsAsync(self, names: list, auto_wait: bool = False) -> None:
        msg = OutgoingMessage()

        msg.write_string('PreLoadAssetsAsync')
        count = len(names)
        msg.write_int32(count)
        for i in range(count):
            msg.write_string(names[i])

        self.asset_channel.send_message(msg)

        if auto_wait:
            self.WaitLoadDone()


    def LoadSceneAsync(self, file: str, auto_wait: bool = False) -> None:
        """
        Load the scene asynchronisely.

        Args:
            file: Str, the scene JSON file. If it's a relative path, it will load from `StraemingAssets`.
            auto_wait: Bool, if True, this function will not return until the loading is done.
        """
        msg = OutgoingMessage()

        msg.write_string('LoadSceneAsync')
        msg.write_string(file)

        self.asset_channel.send_message(msg)

        if auto_wait:
            self.WaitLoadDone()

    def SwitchSceneAsync(self, name: str, auto_wait: bool = False) -> None:
        """
        Switch the scene asynchronisely.

        Args:
            name: Str, the scene name.
            auto_wait: Bool, if True, this function will not return until the loading is done.
        """
        msg = OutgoingMessage()

        msg.write_string('SwitchSceneAsync')
        msg.write_string(name)

        self.asset_channel.send_message(msg)

        if auto_wait:
            self.WaitLoadDone()

    def WaitLoadDone(self) -> None:
        """
        Wait for the loading is done.
        """
        self.asset_channel.data['load_done'] = False
        while not self.asset_channel.data['load_done']:
            self.env.step()

    def Pend(self) -> None:
        """
        Pend the program until the `EndPend` button in `UnityPlayer` is clicked.
        """
        msg = OutgoingMessage()

        msg.write_string('Pend')

        self.asset_channel.send_message(msg)

        self.asset_channel.data['pend_done'] = False
        while not self.asset_channel.data['pend_done']:
            self.env.step()

    def SendMessage(self, message: str, *args) -> None:
        """
        Send message to Unity.

        Args:
            message: Str, the message head.
            args: List, the list of parameters. We support str, bool, int, float and List[float] types.
        """
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
        """
        Add listener.

        Args:
            message: Str, the message head.
            fun: Callable, the callback function.
        """
        if message in self.asset_channel.messages:
            if fun in self.asset_channel.messages[message]:
                self.asset_channel.messages[message].append(fun)
        else:
            self.asset_channel.messages[message] = [fun]

    def RemoveListener(self, message: str, fun):
        """
        Remove listener.

        Args:
            message: Str, the message head.
            fun: Callable, the callback function.
        """
        if message in self.asset_channel.messages:
            if fun in self.asset_channel.messages[message]:
                self.asset_channel.messages[message].remove(fun)
            if len(self.asset_channel.messages[message]) == 0:
                self.asset_channel.messages[message].pop(message)

    def InstanceObject(self, name: str, id: int = None, attr_type: type = attr.BaseAttr):
        """
        Instanciate an object.

        Built-in assets:
    
        GameObjectAttr:
            Basic Objects:
                "GameObject_Box",
                "GameObject_Capsule",
                "GameObject_Cylinder",
                "GameObject_Sphere",
                "GameObject_Quad",
            IGbison Meshes:
                "Hainesburg_mesh_texture",
                "Halfway_mesh_texture",
                "Hallettsville_mesh_texture",
                "Hambleton_mesh_texture",
                "Hammon_mesh_texture",
                "Hatfield_mesh_texture",
                "Haxtun_mesh_texture",
                "Haymarket_mesh_texture",
                "Hendrix_mesh_texture",
                "Hercules_mesh_texture",
                "Highspire_mesh_texture",
                "Hitchland_mesh_texture",

        ColliderAttr:
            "Collider_Box",
            "Collider_ObiBox",
            "Collider_Capsule",
            "Collider_Cylinder",
            "Collider_Sphere",
            "Collider_Quad",

        RigidbodyAttr:
            Basic Objects:
                "Rigidbody_Box",
                "GameObject_Capsule",
                "Rigidbody_Cylinder",
                "Rigidbody_Sphere",
            YCB dataset: 
                77 models in YCB dataset. See YCB Object and Model Set for detail: https://rse-lab.cs.washington.edu/projects/posecnn/

        ControllerAttr:
            gripper:
                "allegro_hand_right",
                "bhand",
                "svh",
                "robotiq_arg2f_85_model",
                "dh_robotics_ag95_gripper",
            robot:
                "kinova_gen3",
                "kinova_gen3_robotiq85",
                "ur5",
                "ur5_robotiq85",
                "franka_panda",
                "franka_hand",
                "tobor_robotiq85_robotiq85",
                "flexivArm",
                "flexivArm_ag95",
                "yumi",

        CameraAttr:
            "Camera",

        LightAttr:
            "Light",

        Args:
            name: Str, object name. Please check the above `built-in assets` list for names.
            id: Int, object id.
            attr_type: type(pyrfuniverse.attributes.BaseAttr), the attribute type.

        Returns:
            type(`attr_type`): The object attribute instance.
        """
        assert id not in self.attrs, \
            'this ID exists'

        while id is None or id in self.attrs:
            id = random.randint(100000, 999999)

        msg = OutgoingMessage()

        msg.write_string('InstanceObject')
        msg.write_string(name)
        msg.write_int32(id)

        self.asset_channel.send_message(msg)

        self.attrs[id] = attr_type(self, id)
        return self.attrs[id]

    def LoadURDF(self, path: str, id: int = None, native_ik: bool = True) -> attr.ControllerAttr:
        """
        Load a model from URDF file.

        Args:
            path: Str, the URDF file path.
            id: Int, object id.
            native_ik: Bool, True for enabling native IK; False for using custom IK.

        Returns:
            pyrfuniverse.attributes.ControllerAttr: The object attribute intance.
        """
        assert id not in self.attrs, \
            'this ID exists'

        while id is None or id in self.attrs:
            id = random.randint(100000, 999999)

        msg = OutgoingMessage()

        msg.write_string('LoadURDF')
        msg.write_int32(id)
        msg.write_string(path)
        msg.write_bool(native_ik)

        self.asset_channel.send_message(msg)

        self.attrs[id] = attr.ControllerAttr(self, id)
        return self.attrs[id]

    def LoadMesh(self, path: str, id: int = None) -> attr.RigidbodyAttr:
        """
        Load a model from Mesh file.

        Args:
            path: Str, the Mesh file path.
            id: Int, object id.

        Returns:
            pyrfuniverse.attributes.RigidbodyAttr: The object attribute intance.
        """
        assert id not in self.attrs, \
            'this ID exists'

        while id is None or id in self.attrs:
            id = random.randint(100000, 999999)

        msg = OutgoingMessage()

        msg.write_string('LoadMesh')
        msg.write_int32(id)
        msg.write_string(path)

        self.asset_channel.send_message(msg)

        self.attrs[id] = attr.RigidbodyAttr(self, id)
        return self.attrs[id]

    def IgnoreLayerCollision(self, layer1: int, layer2: int, ignore: bool) -> None:
        """
        Ignore or enable the collision between two layers.

        Args:
            layer1: Int, the layer number of the first layer.
            layer2: Int, the layer number of the second layer.
            ignore: Bool, True for ignoring collision between two layers; False for enabling collision between two layers.
        """
        msg = OutgoingMessage()

        msg.write_string('IgnoreLayerCollision')
        msg.write_int32(layer1)
        msg.write_int32(layer2)
        msg.write_bool(ignore)

        self.asset_channel.send_message(msg)

    def GetCurrentCollisionPairs(self) -> None:
        """
        Get the collision pairs of current collision.

        Returns:
            Call this function and `step()`, the collision pairs can be got from env.data['CurrentCollisionPairs'].
        """
        msg = OutgoingMessage()

        msg.write_string('GetCurrentCollisionPairs')

        self.asset_channel.send_message(msg)

    def GetRFMoveColliders(self) -> None:
        """
        Get the RFMove colliders.

        Returns:
            Call this function and `step()`, the collision pairs can be got from env.data['RFMoveColliders'].
        """
        msg = OutgoingMessage()

        msg.write_string('GetRFMoveColliders')

        self.asset_channel.send_message(msg)

    def SetGravity(self, x: float, y: float, z: float) -> None:
        """
        Set the gravity of environment.

        Args:
            x: Float, gravity on global x-axis (right).
            y: Float, gravity on global y-axis (up).
            z: Float, gravity on global z-axis (forward).
        """
        msg = OutgoingMessage()

        msg.write_string('SetGravity')
        msg.write_float32(x)
        msg.write_float32(y)
        msg.write_float32(z)

        self.asset_channel.send_message(msg)

    def SetGroundPhysicMaterial(self, bounciness: float, dynamic_friction: float, static_friction: float, friction_combine: int, bounce_combine: int) -> None:
        """
        Set the physics material of ground in environment.

        Args:
            bounciness: Float, the bounciness.
            dynamic_friction: Float, the dynamic friction coefficient (0-1).
            static_friction: Float, the static friction coefficient (0-1).
            friction_combine: Int, how friction of two colliding objects is combined. 0 for Average, 1 for Minimum, 2 for Maximum and 3 for Multiply. See https://docs.unity3d.com/Manual/class-PhysicMaterial.html for more details.
            bounce_combine: Int, how bounciness of two colliding objects is combined. The value representation is the same with `friction_combine`.
        """
        msg = OutgoingMessage()

        msg.write_string('SetGroundPhysicMaterial')
        msg.write_float32(bounciness)
        msg.write_float32(dynamic_friction)
        msg.write_float32(static_friction)
        msg.write_int32(friction_combine)
        msg.write_int32(bounce_combine)

        self.asset_channel.send_message(msg)

    def SetTimeStep(self, delta_time: float) -> None:
        """
        Set the time for a step in Unity.

        Args:
            delta_time: Float, the time for a step in Unity.
        """
        msg = OutgoingMessage()

        msg.write_string('SetTimeStep')
        msg.write_float32(delta_time)

        self.asset_channel.send_message(msg)

    def SetTimeScale(self, time_scale: float) -> None:
        """
        Set the time scale in Unity.

        Args:
            time_scale: Float, the time scale in Unity.
        """
        msg = OutgoingMessage()

        msg.write_string('SetTimeScale')
        msg.write_float32(time_scale)

        self.asset_channel.send_message(msg)

    def SetResolution(self, resolution_x: int, resolution_y: int) -> None:
        """
        Set the resolution of windowed GUI.

        Args:
            resolution_x: Int, window width.
            resolution_y: Int, window height.
        """
        msg = OutgoingMessage()

        msg.write_string('SetResolution')
        msg.write_int32(resolution_x)
        msg.write_int32(resolution_y)

        self.asset_channel.send_message(msg)

    def ExportOBJ(self, items_id: list, save_path: str) -> None:
        """
        Export the specified object list to OBJ file. For native bundle models, the `Read/Write` must be checked in Unity Editor.

        Args:
            items_id: List, the object ids.
            save_path: Str, the path to save the OBJ files.
        """
        msg = OutgoingMessage()

        msg.write_string('ExportOBJ')
        msg.write_int32(len(items_id))
        for i in items_id:
            msg.write_int32(i)
        msg.write_string(save_path)

        self.asset_channel.send_message(msg)

    def SetShadowDistance(self, distance: float) -> None:
        """
        Set the shadow distance for rendering in environment.

        Args:
            distance: Float, the shadow distance measured in meter.
        """
        msg = OutgoingMessage()

        msg.write_string('SetShadowDistance')
        msg.write_float32(distance)

        self.asset_channel.send_message(msg)

    def SaveScene(self, file: str) -> None:
        """
        Save current scene.

        Args:
            file: Str, the file path to save current scene. Default saving to `StreamingAssets` folder.
        """
        msg = OutgoingMessage()

        msg.write_string('SaveScene')
        msg.write_string(file)

        self.asset_channel.send_message(msg)

    def ClearScene(self) -> None:
        """
        Clear current scene.
        """
        msg = OutgoingMessage()

        msg.write_string('ClearScene')

        self.asset_channel.send_message(msg)

    def AlignCamera(self, camera_id: int) -> None:
        """
        Align current GUI view to a given camera.

        Args:
            camera_id: Int, camera id.
        """
        msg = OutgoingMessage()

        msg.write_string('AlignCamera')
        msg.write_int32(camera_id)

        self.asset_channel.send_message(msg)

    def SetViewTransform(self, position: list = None, rotation: list = None) -> None:
        """
        Set the GUI view.

        Args:
            position: A list of length 3, representing the position of GUI view.
            rotation: A list of length 3, representing the rotation of GUI view.
        """
        msg = OutgoingMessage()

        msg.write_string('SetViewTransform')
        msg.write_bool(position is not None)
        msg.write_bool(rotation is not None)
        if position is not None:
            assert type(position) == list and len(position) == 3, \
                'Argument position must be a 3-d list.'
            for i in range(3):
                msg.write_float32(position[i])
        if rotation is not None:
            assert type(rotation) == list and len(rotation) == 3, \
                'Argument rotation must be a 3-d list.'
            for i in range(3):
                msg.write_float32(rotation[i])

        self.asset_channel.send_message(msg)


    #Dubug API
    def DebugGraspPoint(self, enabled: bool = True) -> None:
        """
        Show or hide end effector of robot arm for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('DebugGraspPoint')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugObjectPose(self, enabled: bool = True) -> None:
        """
        Show or hide object base point for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('DebugObjectPose')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugCollisionPair(self, enabled: bool = True) -> None:
        """
        Show or hide collision pairs for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('DebugCollisionPair')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)
    
    def DebugColliderBound(self, enabled: bool = True) -> None:
        """
        Show or hide collider bounding box for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('DebugColliderBound')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugObjectID(self, enabled: bool = True) -> None:
        """
        Show or hide object id for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('DebugObjectID')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def Debug3DBBox(self, enabled: bool = True) -> None:
        """
        Show or hide 3d bounding box of objects for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('Debug3DBBox')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def Debug2DBBox(self, enabled: bool = True) -> None:
        """
        Show or hide 2d bounding box of objects for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('Debug2DBBox')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugJointLink(self, enabled: bool = True) -> None:
        """
        Show or hide joint information of articulation for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        msg = OutgoingMessage()
        msg.write_string('DebugJointLink')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def SendLog(self, log: str) -> None:
        """
        Send log messange and show it on Unity GUI window.

        Args:
            log: Str, log message.
        """
        msg = OutgoingMessage()
        msg.write_string('SendLog')
        msg.write_string(log)
        self.debug_channel.send_message(msg)

    def _SendVersion(self) -> None:
        msg = OutgoingMessage()
        msg.write_string('SendVersion')
        msg.write_string(pyrfuniverse.__version__)
        self.debug_channel.send_message(msg)



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
