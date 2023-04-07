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
    RFUnivers基础环境类
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
        self._SendVersion()
        if self.scene_file is not None:
            self.LoadSceneAsync(self.scene_file, True)
        if len(self.assets) > 0:
            self._PreLoadAssetsAsync(self.assets, True)
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
        将已经调用的接口消息发送给Unity，执行一步仿真，然后接收Unity返回的数据

        Args:
            count: 执行的步数
        """
        if count < 1:
            count = 1
        for _ in range(count):
            self.env.step()

    def close(self):
        """
        关闭环境
        """
        delete_worker_id(self.worker_id)
        self.env.close()

    def GetAttr(self, id: int):
        """
        根据ID获取物体

        Args:
            id: 物体ID
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
        异步加载场景

        Args:
            file: 场景Json文件，当该值为路径时，从路径加载场景，否则从StreamingAssets加载场景
            auto_wait: 是否等待加载完成，如果为True，则在加载完成后才返回
        """
        msg = OutgoingMessage()

        msg.write_string('LoadSceneAsync')
        msg.write_string(file)

        self.asset_channel.send_message(msg)

        if auto_wait:
            self.WaitLoadDone()

    def WaitLoadDone(self) -> None:
        """
        等待加载完成，使用LoadSceneAsync接口后，调用该接口可以等待加载完成
        """
        self.asset_channel.data['load_done'] = False
        while not self.asset_channel.data['load_done']:
            self.env.step()

    def Pend(self) -> None:
        """
        挂起，直到UnityPlayer中点击EndPend按钮
        """
        msg = OutgoingMessage()

        msg.write_string('Pend')

        self.asset_channel.send_message(msg)

        self.asset_channel.data['pend_done'] = False
        while not self.asset_channel.data['pend_done']:
            self.env.step()

    def SendMessage(self, message: str, *args) -> None:
        """
        发送动态消息给Unity

        Args:
            message: 消息头
            *args: 参数列表，支持的参数类型有：str, bool, int, float, list[float]
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
        添加动态消息监听

        Args:
            message: 消息头
            fun: 回调函数
        """
        if message in self.asset_channel.messages:
            if fun in self.asset_channel.messages[message]:
                self.asset_channel.messages[message].append(fun)
        else:
            self.asset_channel.messages[message] = [fun]

    def RemoveListener(self, message: str, fun):
        """
        移除动态消息监听

        Args:
            message: 消息头
            fun: 回调函数
        """
        if message in self.asset_channel.messages:
            if fun in self.asset_channel.messages[message]:
                self.asset_channel.messages[message].remove(fun)
            if len(self.asset_channel.messages[message]) == 0:
                self.asset_channel.messages[message].pop(message)

    def InstanceObject(self, name: str, id: int = None, attr_type: type = attr.BaseAttr):
        """
        实例化物体

        Args:
            name: 物体名
                RfUniverseRelease中已有的资源类型及名称:
                    GameObjcetAttr 静态物体:
                        GameObject_Box,
                        GameObject_Capsule,
                        GameObject_Cylinder,
                        GameObject_Sphere,
                        GameObject_Quad,

                        IGbison 环境:
                            Hainesburg_mesh_texture,
                            Halfway_mesh_texture,
                            Hallettsville_mesh_texture,
                            Hambleton_mesh_texture,
                            Hammon_mesh_texture,
                            Hatfield_mesh_texture,
                            Haxtun_mesh_texture,
                            Haymarket_mesh_texture,
                            Hendrix_mesh_texture,
                            Hercules_mesh_texture,
                            Highspire_mesh_texture,
                            Hitchland_mesh_texture,

                    ColliderAttr 带有碰撞体的静态物体:
                        Collider_Box,
                        Collider_ObiBox,
                        Collider_Capsule,
                        Collider_Cylinder,
                        Collider_Sphere,
                        Collider_Quad,

                    RigidbodyAttr 刚体:
                        Rigidbody_Box,
                        GameObject_Capsule,
                        Rigidbody_Cylinder,
                        Rigidbody_Sphere,

                        77个YCB数据集模型: 详见The YCB Object and Model Set: https://rse-lab.cs.washington.edu/projects/posecnn/

                    ControllerAttr 机械臂及关节体:
                        gripper:
                            allegro_hand_right,
                            bhand,
                            svh,
                            robotiq_arg2f_85_model,
                            dh_robotics_ag95_gripper,
                        robot:
                            kinova_gen3,
                            kinova_gen3_robotiq85,
                            ur5,
                            ur5_robotiq85,
                            franka_panda,
                            franka_hand,
                            tobor_robotiq85_robotiq85,
                            flexivArm,
                            flexivArm_ag95,
                            yumi,

                    CameraAttr 相机:
                        Camera,

                    LightAttr 灯光:
                        Light,

            id: 物体ID
            attr_type: 物体类型

        Returns:
            物体实例
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
        加载URDF模型

        Args:
            path: URDF路径
            id: 物体ID
            native_ik: 是否启用内置IK

        Returns:
            ControllerAttr机械臂实例
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
        加载Mesh模型

        Args:
            path: Mesh路径
            id: 物体ID

        Returns:
            RigidbodyAttr刚体实例
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
        忽略或启用指定两个层的碰撞

        Args:
            layer1: 层1
            layer2: 层1
            ignore: 是否忽略
        """
        msg = OutgoingMessage()

        msg.write_string('IgnoreLayerCollision')
        msg.write_int32(layer1)
        msg.write_int32(layer2)
        msg.write_bool(ignore)

        self.asset_channel.send_message(msg)

    def GetCurrentCollisionPairs(self) -> None:
        """
        获取当前碰撞对

        Returns:
            调用此接口并step后，从env.data['CurrentCollisionPairs']中获取碰撞对
        """
        msg = OutgoingMessage()

        msg.write_string('GetCurrentCollisionPairs')

        self.asset_channel.send_message(msg)

    def GetRFMoveColliders(self) -> None:
        """
        获取RFMove碰撞体

        Returns:
            调用此接口并step后，从env.data['RFMoveColliders']中获取碰撞体
        """
        msg = OutgoingMessage()

        msg.write_string('GetRFMoveColliders')

        self.asset_channel.send_message(msg)

    def SetGravity(self, x: float, y: float, z: float) -> None:
        """
        设置环境重力

        Args:
            x: 右方向
            y: 上方向
            z: 前方向
        """
        msg = OutgoingMessage()

        msg.write_string('SetGravity')
        msg.write_float32(x)
        msg.write_float32(y)
        msg.write_float32(z)

        self.asset_channel.send_message(msg)

    def SetGroundPhysicMaterial(self, bounciness: float, dynamic_friction: float, static_friction: float, friction_combine: int, bounce_combine: int) -> None:
        """
        设置环境地面物理材质

        Args:
            bounciness: 弹力
            dynamic_friction: 动摩擦力
            static_friction: 静摩擦力
            friction_combine: 摩擦力组合方式
            bounce_combine: 弹力组合方式
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
        设置环境step时间步长

        Args:
            delta_time: 时间步长(s)
        """
        msg = OutgoingMessage()

        msg.write_string('SetTimeStep')
        msg.write_float32(delta_time)

        self.asset_channel.send_message(msg)

    def SetTimeScale(self, time_scale: float) -> None:
        """
        设置环境时间缩放比例

        Args:
            time_scale: 时间缩放比例
        """
        msg = OutgoingMessage()

        msg.write_string('SetTimeScale')
        msg.write_float32(time_scale)

        self.asset_channel.send_message(msg)

    def SetResolution(self, resolution_x: int, resolution_y: int) -> None:
        """
        设置窗口分辨率

        Args:
            resolution_x: 宽度width分辨率
            resolution_y: 高度height分辨率
        """
        msg = OutgoingMessage()

        msg.write_string('SetResolution')
        msg.write_int32(resolution_x)
        msg.write_int32(resolution_y)

        self.asset_channel.send_message(msg)

    def ExportOBJ(self, items_id: list, save_path: str) -> None:
        """
        导出指定物体列表为OBJ文件

        Args:
            items_id: 物体ID列表
            save_path: 保存绝对路径
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
        设置环境阴影渲染距离

        Args:
            distance: 距离(m)
        """
        msg = OutgoingMessage()

        msg.write_string('SetShadowDistance')
        msg.write_float32(distance)

        self.asset_channel.send_message(msg)

    def SaveScene(self, file: str) -> None:
        """
        保存当前场景

        Args:
            file: 存储场景Json路径，当该值为路径时，保存到该路径，否则保存到StreamingAssets
        """
        msg = OutgoingMessage()

        msg.write_string('SaveScene')
        msg.write_string(file)

        self.asset_channel.send_message(msg)

    def ClearScene(self) -> None:
        """
        清理当前场景
        """
        msg = OutgoingMessage()

        msg.write_string('ClearScene')

        self.asset_channel.send_message(msg)

    def AlignCamera(self, camera_id: int) -> None:
        """
        对齐视口到相机

        Args:
            camera_id: 相机ID
        """
        msg = OutgoingMessage()

        msg.write_string('AlignCamera')
        msg.write_int32(camera_id)

        self.asset_channel.send_message(msg)

    def SetViewTransform(self, position: list = None, rotation: list = None) -> None:
        """
        设置视口位置和旋转

        Args:
            position: 位置
            rotation: 旋转
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
        Debug显示机械臂末端点
        """
        msg = OutgoingMessage()
        msg.write_string('DebugGraspPoint')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugObjectPose(self, enabled: bool = True) -> None:
        """
        Debug显示物体base点
        """
        msg = OutgoingMessage()
        msg.write_string('DebugObjectPose')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugCollisionPair(self, enabled: bool = True) -> None:
        """
        Debug显示物理碰撞对
        """
        msg = OutgoingMessage()
        msg.write_string('DebugCollisionPair')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)
    def DebugColliderBound(self, enabled: bool = True) -> None:
        """
        Debug显示碰撞包围盒
        """
        msg = OutgoingMessage()
        msg.write_string('DebugColliderBound')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugObjectID(self, enabled: bool = True) -> None:
        """
        Debug显示碰物体ID
        """
        msg = OutgoingMessage()
        msg.write_string('DebugObjectID')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def Debug3DBBox(self, enabled: bool = True) -> None:
        """
        Debug显示物体3DBoundingBox
        """
        msg = OutgoingMessage()
        msg.write_string('Debug3DBBox')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def Debug2DBBox(self, enabled: bool = True) -> None:
        """
        Debug显示物体2DBoundingBox
        """
        msg = OutgoingMessage()
        msg.write_string('Debug2DBBox')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def DebugJointLink(self, enabled: bool = True) -> None:
        """
        Debug显示关节体Joint信息
        """
        msg = OutgoingMessage()
        msg.write_string('DebugJointLink')
        msg.write_bool(enabled)
        self.debug_channel.send_message(msg)

    def SendLog(self, log: str) -> None:
        """
        发送Log消息并显示在Unity窗口

        Args:
            log: Log内容
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
