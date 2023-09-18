import random
import subprocess
from abc import ABC
import socket
import threading
import time
import numpy as np
import pyrfuniverse
import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel import IncomingMessage, OutgoingMessage
from pyrfuniverse.utils.locker import Locker
from pyrfuniverse.utils.rfuniverse_communicator import RFUniverseCommunicator
import os
import warnings


class RFUniverseBaseEnv(ABC):
    """
    RFUniverse base environment class.

    Args:
        executable_file: Str, the absolute path of Unity executable file. None for last used executable file; "@editor" for using Unity Editor.
        scene_file: Str, the absolute path of Unity scene JSON file. All JSON files locate at `StraemingAssets/SceneData` by default.
        assets: List, the list of pre-load assets. All assets in the list will be pre-loaded in Unity when the environment is initialized, which will save time during instanciating.
        graphics: Bool, True for showing GUI and False for headless mode.
    """

    metadata = {"render.modes": ["human", "rgb_array"]}

    def __init__(
        self,
        executable_file: str = None,
        scene_file: str = None,
        assets: list = [],
        graphics: bool = True,
        port: int = 5004,
        proc_id=0,
        log_level=1,
    ):
        # time step
        self.t = 0
        self.executable_file = executable_file
        self.scene_file = scene_file
        self.pre_load_assets = assets
        self.graphics = graphics
        self.process = None
        self.attrs = {}
        self.data = {}
        self.listen_messages = {}
        self.listen_object = {}
        self.port = port

        self.log_level = log_level

        self.log_map = {"Log": 3, "Warning": 2, "Error": 1, "Exception": 1, "Assert": 1}

        if self.executable_file is None:
            self.executable_file = pyrfuniverse.executable_file

        if self.executable_file == "" or self.executable_file == "@editor":  # editor
            assert proc_id == 0, "proc_id must be 0 when using editor"
            print("Waiting for UnityEditor play...")
            PROC_TYPE = "editor"
        elif os.path.exists(self.executable_file):  # release
            PROC_TYPE = "release"
            self.port = self.port + 1 + proc_id  # default release port
        else:  # error
            raise ValueError(f"Executable file {self.executable_file} not exists")

        self.communicator = RFUniverseCommunicator(
            port=self.port,
            receive_data_callback=self._receive_data,
            proc_type=PROC_TYPE,
        )
        self.port = self.communicator.port  # update port
        if PROC_TYPE == "release":
            self.process = self._start_unity_env(self.executable_file, self.port)
        self.communicator.online()

        self._send_debug_data("SetPythonVersion", pyrfuniverse.__version__)
        if len(self.pre_load_assets) > 0:
            self.PreLoadAssetsAsync(assets, True)
        if self.scene_file is not None:
            self.LoadSceneAsync(self.scene_file, True)

    def _get_port(self) -> int:
        executable_port = self.port + 1
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        while True:
            try:
                s.bind(("localhost", executable_port))
                s.close()
                return executable_port
            except OSError:
                executable_port += 1
                continue

    def _start_unity_env(self, executable_file: str, port: int) -> subprocess.Popen:
        arg = [executable_file]
        if not self.graphics:
            arg.extend(["-nographics", "-batchmode"])
        if self.log_level == 0:
            proc_out = subprocess.DEVNULL
        else:
            proc_out = None
        arg.append(f"-port:{port}")
        return subprocess.Popen(arg, stdout=proc_out, stderr=proc_out)

    def _receive_data(self, objs: list) -> None:
        msg = objs[0]
        objs = objs[1:]
        if msg == "Env":
            self._parse_env_data(objs)
        elif msg == "Instance":
            self._parse_instence_data(objs)
        elif msg == "Debug":
            self._parse_debug_data(objs)
        elif msg == "Message":
            self._parse_message_data(objs)
        elif msg == "Object":
            self._parse_object_data(objs)
        return

    def _parse_env_data(self, objs: list) -> None:
        msg = objs[0]
        objs = objs[1:]
        if msg == "Close":
            self.close()
        elif msg == "LoadDone":
            self.data["load_done"] = True
        elif msg == "PendDone":
            self.data["pend_done"] = True
        elif msg == "RFMoveColliders":
            self.data["colliders"] = objs[0]
        elif msg == "CurrentCollisionPairs":
            self.data["collision_pairs"] = objs[0]
        else:
            print(f"unknown env data type: {msg}")

    def _parse_instence_data(self, objs: list) -> None:
        this_object_id = objs[0]
        this_object_type = objs[1]
        this_object_data = objs[2]

        attr_type = eval("attr." + this_object_type)
        if this_object_id not in self.attrs:
            self.attrs[this_object_id] = attr_type(self, this_object_id)
        elif type(self.attrs[this_object_id]) != attr_type:
            self.attrs[this_object_id] = attr_type(
                self, this_object_id, self.attrs[this_object_id].data
            )

        self.data[this_object_id] = self.attrs[this_object_id].parse_message(
            this_object_data
        )

    def _parse_debug_data(self, objs: list) -> None:
        msg = objs[0]
        objs = objs[1:]
        if msg == "Log":
            if self.log_map[objs[0]] <= self.log_level:
                print(
                    f"Unity Env Log Type:{objs[0]}\nCondition:{objs[1]}\nStackTrace:{objs[2]}"
                )
        else:
            print(f"unknown debug data type: {msg}")
        return

    def _parse_message_data(self, objs: list) -> None:
        msg = objs[0]
        objs = objs[1:]
        if msg in self.listen_messages:
            self.listen_messages[msg](IncomingMessage(objs[0]))

    def _parse_object_data(self, objs: list) -> None:
        head = objs[0]
        objs = objs[1:]
        if head in self.listen_object:
            self.listen_object[head](objs)
        # else:
        # warnings.warn(f"unknown object data type: {head}")

    def _send_env_data(self, *args) -> None:
        self.communicator.send_object("Env", *args)

    def _send_instance_data(self, *args) -> None:
        self.communicator.send_object("Instance", *args)

    def _send_debug_data(self, *args) -> None:
        self.communicator.send_object("Debug", *args)

    def _send_message_data(self, *args) -> None:
        self.communicator.send_object("Message", *args)

    def _send_object_data(self, *args) -> None:
        self.communicator.send_object("Object", *args)

    def _step(self):
        """
        Send the messages of called functions to Unity and simulate for a step, then accept the data from Unity.
        """
        if not self.communicator.connected:
            raise Exception("Unity Env not connected")
        self.communicator.sync_step()

    def step(self, count: int = 1):
        """
        Send the messages of called functions to Unity and simulate for a step, then accept the data from Unity.

        Args:
            count: the number of steps for executing Unity simulation.
        """
        if count < 1:
            count = 1
        for _ in range(count):
            self._step()

    def close(self):
        """
        Close the environment
        """
        if self.process is not None:
            self.process.kill()
        if self.communicator is not None:
            self.communicator.close()

    def GetAttr(self, id: int):
        """
        Get the attribute instance by object id.

        Args:
            id: Int, object id.

        Returns:
            pyrfuniverse.attributes.BaseAttr: An instance of attribute.
        """
        assert id in self.attrs, "this ID not exists"
        return self.attrs[id]

    # Env API
    def PreLoadAssetsAsync(self, names: list, auto_wait: bool = False) -> None:
        """
        PreLoad the asset.

        Args:
            names: list, the name of assets.
            auto_wait: Bool, if True, this function will not return until the loading is done.
        """
        self._send_env_data("PreLoadAssetsAsync", names)

        if auto_wait:
            self.WaitLoadDone()

    def LoadSceneAsync(self, file: str, auto_wait: bool = False) -> None:
        """
        Load the scene asynchronisely.

        Args:
            file: Str, the scene JSON file. If it's a relative path, it will load from `StraemingAssets`.
            auto_wait: Bool, if True, this function will not return until the loading is done.
        """
        self._send_env_data("LoadSceneAsync", file)

        if auto_wait:
            self.WaitLoadDone()

    def SwitchSceneAsync(self, name: str, auto_wait: bool = False) -> None:
        """
        Switch the scene asynchronisely.

        Args:
            name: Str, the scene name.
            auto_wait: Bool, if True, this function will not return until the loading is done.
        """
        self._send_env_data("SwitchSceneAsync", name)

        if auto_wait:
            self.WaitLoadDone()

    def WaitLoadDone(self) -> None:
        """
        Wait for the loading is done.
        """
        self.data["load_done"] = False
        while not self.data["load_done"]:
            self._step()

    def Pend(self) -> None:
        """
        Pend the program until the `EndPend` button in `UnityPlayer` is clicked.
        """
        self._send_env_data("Pend")

        self.data["pend_done"] = False
        while not self.data["pend_done"]:
            self._step()

    def SendMessage(self, message: str, *args) -> None:
        """
        Send message to Unity.

        Args:
            message: Str, the message head.
            *args: List, the list of parameters. We support str, bool, int, float and List[float] types.
        """
        msg = OutgoingMessage()
        for i in args:
            if type(i) == str:
                msg.write_string(i)
            elif type(i) == bool:
                msg.write_bool(i)
            elif type(i) == int:
                msg.write_int32(i)
            elif type(i) == float or type(i) == np.float32 or type(i) == np.float64:
                msg.write_float32(float(i))
            elif type(i) == list and type(i[0]) == float:
                msg.write_float32_list(i)
            else:
                print(f"dont support this data type:{type(i)}")
        self._send_message_data(message, msg.buffer)

    def SendObject(self, head: str, *args) -> None:
        """
        Send object to Unity.

        Args:
            head: Str, the message head.
            *args: List, the list of parameters. We support str, bool, int, float and List[float] types.
        """
        self._send_object_data(head, *args)

    def AddListener(self, message: str, fun):
        """
        Add listener.

        Args:
            message: Str, the message head.
            fun: Callable, the callback function.
        """
        self.listen_messages[message] = fun

    def RemoveListener(self, message: str, fun):
        """
        Remove listener.

        Args:
            message: Str, the message head.
            fun: Callable, the callback function.
        """
        self.listen_messages.pop(message)

    def AddListenerObject(self, type: str, fun):
        """
        Add object listener.

        Args:
            type: Str, the message head.
            fun: Callable, the callback function.
        """
        self.listen_object[type] = fun

    def RemoveListenerObject(self, type: str):
        """
        Remove object listener.

        Args:
            type: Str, the message head.
        """
        self.listen_object.pop(type)

    def InstanceObject(
        self, name: str, id: int = None, attr_type: type = attr.BaseAttr
    ):
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
        assert id not in self.attrs, "this ID exists"

        while id is None or id in self.attrs:
            id = random.randint(100000, 999999)

        self._send_env_data("InstanceObject", name, id)

        self.attrs[id] = attr_type(self, id)
        return self.attrs[id]

    def LoadURDF(
        self, path: str, id: int = None, native_ik: bool = False, axis: str = "y"
    ) -> attr.ControllerAttr:
        """
        Load a model from URDF file.

        Args:
            path: Str, the URDF file path.
            id: Int, object id.
            native_ik: Bool, True for enabling native IK; False for using custom IK.When it is True, through the IKTargetDo*** interface, according to the end pose.When it is False, through the SetJoint*** interface, according to the joint movement.
            axis: Str, import axis, This can be 'z' or 'y', depending on the URDF file

        Returns:
            pyrfuniverse.attributes.ControllerAttr: The object attribute intance.
        """
        assert id not in self.attrs, "this ID exists"

        while id is None or id in self.attrs:
            id = random.randint(100000, 999999)

        self._send_env_data("LoadURDF", id, path, native_ik, axis)

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
        assert id not in self.attrs, "this ID exists"

        while id is None or id in self.attrs:
            id = random.randint(100000, 999999)

        self._send_env_data("LoadMesh", id, path)

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
        self._send_env_data("IgnoreLayerCollision", layer1, layer2, ignore)

    def GetCurrentCollisionPairs(self) -> None:
        """
        Get the collision pairs of current collision. After calling this method and stepping once, the result will be saved in env.data['CurrentCollisionPairs']
        """
        self._send_env_data("GetCurrentCollisionPairs")

    def GetRFMoveColliders(self) -> None:
        """
        Get the RFMove colliders. After calling this method and stepping once, the result will be saved in env.data['RFMoveColliders']
        """
        self._send_env_data("GetRFMoveColliders")

    def SetGravity(self, x: float, y: float, z: float) -> None:
        """
        Set the gravity of environment.

        Args:
            x: Float, gravity on global x-axis (right).
            y: Float, gravity on global y-axis (up).
            z: Float, gravity on global z-axis (forward).
        """
        self._send_env_data("SetGravity", [float(x), float(y), float(z)])

    def SetGroundActive(self, active: bool) -> None:
        """
        Set the ground active or inactive.

        Args:
            active: Bool, active or inactive the ground.
        """
        self._send_env_data("GetRFMoveColliders", active)

    def SetGroundPhysicMaterial(
        self,
        bounciness: float,
        dynamic_friction: float,
        static_friction: float,
        friction_combine: int,
        bounce_combine: int,
    ) -> None:
        """
        Set the physics material of ground in environment.

        Args:
            bounciness: Float, the bounciness.
            dynamic_friction: Float, the dynamic friction coefficient (0-1).
            static_friction: Float, the static friction coefficient (0-1).
            friction_combine: Int, how friction of two colliding objects is combined. 0 for Average, 1 for Minimum, 2 for Maximum and 3 for Multiply. See https://docs.unity3d.com/Manual/class-PhysicMaterial.html for more details.
            bounce_combine: Int, how bounciness of two colliding objects is combined. The value representation is the same with `friction_combine`.
        """
        self._send_env_data(
            "SetGroundPhysicMaterial",
            float(bounciness),
            float(dynamic_friction),
            float(static_friction),
            friction_combine,
            bounce_combine,
        )

    def SetTimeStep(self, delta_time: float) -> None:
        """
        Set the time for a step in Unity.

        Args:
            delta_time: Float, the time for a step in Unity.
        """
        self._send_env_data("SetTimeStep", float(delta_time))

    def SetTimeScale(self, time_scale: float) -> None:
        """
        Set the time scale in Unity.

        Args:
            time_scale: Float, the time scale in Unity.
        """
        self._send_env_data("SetTimeScale", float(time_scale))

    def SetResolution(self, resolution_x: int, resolution_y: int) -> None:
        """
        Set the resolution of windowed GUI.

        Args:
            resolution_x: Int, window width.
            resolution_y: Int, window height.
        """
        self._send_env_data("SetResolution", resolution_x, resolution_y)

    def ExportOBJ(self, items_id: list, save_path: str) -> None:
        """
        Export the specified object list to OBJ file. For native bundle models, the `Read/Write` must be checked in Unity Editor.

        Args:
            items_id: List, the object ids.
            save_path: Str, the path to save the OBJ files.
        """
        self._send_env_data("ExportOBJ", items_id, save_path)

    def SetShadowDistance(self, distance: float) -> None:
        """
        Set the shadow distance for rendering in environment.

        Args:
            distance: Float, the shadow distance measured in meter.
        """
        self._send_env_data("SetShadowDistance", float(distance))

    def SaveScene(self, file: str) -> None:
        """
        Save current scene.

        Args:
            file: Str, the file path to save current scene. Default saving to `StreamingAssets` folder.
        """
        self._send_env_data("SaveScene", file)

    def ClearScene(self) -> None:
        """
        Clear current scene.
        """
        self._send_env_data("ClearScene")

    def AlignCamera(self, camera_id: int) -> None:
        """
        Align current GUI view to a given camera.

        Args:
            camera_id: Int, camera id.
        """
        self._send_env_data("AlignCamera", camera_id)

    def SetViewTransform(self, position: list = None, rotation: list = None) -> None:
        """
        Set the GUI view.

        Args:
            position: A list of length 3, representing the position of GUI view.
            rotation: A list of length 3, representing the rotation of GUI view.
        """
        if position is not None:
            assert (
                type(position) == list and len(position) == 3
            ), "Argument position must be a 3-d list."
            position = [float(i) for i in position]
        if rotation is not None:
            assert (
                type(rotation) == list and len(rotation) == 3
            ), "Argument rotation must be a 3-d list."
            rotation = [float(i) for i in rotation]

        self._send_env_data("SetViewTransform", position, rotation)

    def ViewLookAt(self, target: list, world_up: list = None) -> None:
        """
        Rotates the transform so the forward vector points at target's current position.

        Args:
            target: A list of length 3, target to point towards.
            world_up: A list of length 3, vector specifying the upward direction.
        """
        if world_up is None:
            world_up = [0.0, 1.0, 0.0]
        assert len(target) == 3, "target length must be 3"
        target = [float(i) for i in target]
        assert len(world_up) == 3, "world_up length must be 3"
        world_up = [float(i) for i in world_up]

        self._send_env_data("ViewLookAt", target, world_up)

    def SetViewBackGround(self, color: list = None) -> None:
        """
        Set the GUI view.

        Args:
            color: A list of length 3, background color of GUI view. None : default skybox.
        """
        if color is not None:
            assert type(color) == list and len(color) == 3, "color length must be 3"
            color = [float(i) for i in color]

        self._send_env_data("SetViewBackGround", color)

    # Dubug API
    def DebugGraspPoint(self, enabled: bool = True) -> None:
        """
        Show or hide end effector of robot arm for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("DebugGraspPoint", enabled)

    def DebugObjectPose(self, enabled: bool = True) -> None:
        """
        Show or hide object base point for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("DebugObjectPose", enabled)

    def DebugCollisionPair(self, enabled: bool = True) -> None:
        """
        Show or hide collision pairs for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("DebugCollisionPair", enabled)

    def DebugColliderBound(self, enabled: bool = True) -> None:
        """
        Show or hide collider bounding box for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("DebugColliderBound", enabled)

    def DebugObjectID(self, enabled: bool = True) -> None:
        """
        Show or hide object id for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("DebugObjectID", enabled)

    def Debug3DBBox(self, enabled: bool = True) -> None:
        """
        Show or hide 3d bounding box of objects for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("Debug3DBBox", enabled)

    def Debug2DBBox(self, enabled: bool = True) -> None:
        """
        Show or hide 2d bounding box of objects for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("Debug2DBBox", enabled)

    def DebugJointLink(self, enabled: bool = True) -> None:
        """
        Show or hide joint information of articulation for debug.

        Args:
            enabled: Bool, True for showing and False for hiding.
        """
        self._send_debug_data("DebugJointLink", enabled)

    def SendLog(self, log: str) -> None:
        """
        Send log messange and show it on Unity GUI window.

        Args:
            log: Str, log message.
        """
        self._send_debug_data("SendLog", log)

    def ShowArticulationParameter(self, controller_id: int) -> None:
        """
        Show Articulation Parameter on Unity GUI window.

        Args:
            controller_id: int, controller_attr id.
        """
        self._send_debug_data("ShowArticulationParameter", int(controller_id))
