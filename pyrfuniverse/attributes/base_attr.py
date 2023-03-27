from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def SetTransform(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = ['position', 'rotation', 'scale', 'is_world']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetTransform')
    position = None
    set_position = False
    rotation = None
    set_rotation = False
    scale = None
    set_scale = False

    if 'position' in kwargs:  # position
        position = kwargs['position']
        set_position = True
        assert type(position) == list and len(position) == 3, \
            'Argument position must be a 3-d list.'

    if 'rotation' in kwargs:  # rotation
        rotation = kwargs['rotation']
        set_rotation = True
        assert type(rotation) == list and len(rotation) == 3, \
            'Argument rotation must be a 3-d list.'

    if 'scale' in kwargs:  # scale
        scale = kwargs['scale']
        set_scale = True
        assert type(scale) == list and len(scale) == 3, \
            'Argument rotation must be a 3-d list.'

    msg.write_bool(set_position)
    msg.write_bool(set_rotation)
    msg.write_bool(set_scale)

    if set_position:
        for i in range(3):
            msg.write_float32(position[i])

    if set_rotation:
        for i in range(3):
            msg.write_float32(rotation[i])

    if set_scale:
        for i in range(3):
            msg.write_float32(scale[i])

    if 'is_world' in kwargs.keys():
        msg.write_bool(kwargs['is_world'])
    else:
        msg.write_bool(True)
    return msg


def SetRotationQuaternion(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'quaternion']
    optional_params = ['is_world']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('SetRotationQuaternion')
    for i in range(4):
        msg.write_float32(kwargs['quaternion'][i])
    if 'is_world' in kwargs.keys():
        msg.write_bool(kwargs['is_world'])
    else:
        msg.write_bool(True)
    return msg


def SetActive(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'active']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetActive')
    msg.write_bool(kwargs['active'])

    return msg


def SetParent(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'parent_id', 'parent_name']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetParent')
    msg.write_int32(kwargs['parent_id'])
    msg.write_string(kwargs['parent_name'])

    return msg


def SetLayer(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'layer']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetLayer')
    msg.write_int32(kwargs['layer'])

    return msg


def Copy(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'copy_id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('Copy')
    msg.write_int32(kwargs['copy_id'])
    return msg


def Destroy(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('Destroy')

    return msg


def SetRFMoveColliderActive(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'active']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetRFMoveColliderActive')
    msg.write_bool(kwargs['active'])

    return msg


def GetLoaclPointFromWorld(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'point']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetLoaclPointFromWorld')
    msg.write_float32(kwargs['point'][0])
    msg.write_float32(kwargs['point'][1])
    msg.write_float32(kwargs['point'][2])
    return msg


def GetWorldPointFromLocal(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'point']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('GetWorldPointFromLocal')
    msg.write_float32(kwargs['point'][0])
    msg.write_float32(kwargs['point'][1])
    msg.write_float32(kwargs['point'][2])
    return msg


class BaseAttr:
    """
    基础Attr类，包含物体加载删除移动等通用功能
    """
    def __init__(self, env, id: int, data=None):
        if data is None:
            data = {}
        self.env = env
        self.id = id
        self.data = data

    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:
            self.data['name'] 物体名称

            self.data['position'] 物体世界坐标

            self.data['rotation'] 物体世界欧拉角

            self.data['quaternion'] 物体世界四元数

            self.data['local_position'] 物体局部坐标

            self.data['local_rotation'] 物体局部欧拉角

            self.data['local_quaternion'] 物体局部四元数

            self.data['local_to_world_matrix'] 物体局部坐标转世界坐标矩阵

            self.data['result_local_point'] 物体局部坐标转世界坐标结果

            self.data['result_world_point'] 物体世界坐标转局部坐标结果
        """
        self.data['name'] = msg.read_string()
        self.data['position'] = [msg.read_float32() for _ in range(3)]
        self.data['rotation'] = [msg.read_float32() for _ in range(3)]
        self.data['quaternion'] = [msg.read_float32() for _ in range(4)]
        self.data['local_position'] = [msg.read_float32() for _ in range(3)]
        self.data['local_rotation'] = [msg.read_float32() for _ in range(3)]
        self.data['local_quaternion'] = [msg.read_float32() for _ in range(4)]

        self.data['local_to_world_matrix'] = msg.read_float32_list()

        if msg.read_bool() is True:
            self.data['result_local_point'] = msg.read_float32_list()
        if msg.read_bool() is True:
            self.data['result_world_point'] = msg.read_float32_list()
        return self.data

    def SetType(self, attr_type: type):
        """
        设置物体Attr类型

        Args:
            attr_type:类型参数

        Returns:目标类型实例
        """
        self.env.attrs[self.id] = attr_type(self.env, self.id, self.data)
        return self.env.attrs[self.id]

    def SetTransform(self, position: list = None, rotation: list = None, scale: list = None, is_world: bool = True):
        """
        使用Vector3设置物体pose

        Args:
            position: 位置
            rotation: 旋转
            scale: 缩放
            is_world: 世界空间/局部空间
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetTransform')
        msg.write_bool(position is not None)
        msg.write_bool(rotation is not None)
        msg.write_bool(scale is not None)
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
        if scale is not None:
            assert type(scale) == list and len(scale) == 3, \
                'Argument rotation must be a 3-d list.'
            for i in range(3):
                msg.write_float32(scale[i])
        msg.write_bool(is_world)

        self.env.instance_channel.send_message(msg)

    def Translate(self, translation: list, is_world: bool = True):
        """
        物体平移

        Args:
            translation: 平移量
            is_world: 世界空间/局部空间
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Translate')
        for i in range(3):
            msg.write_float32(translation[i])
        msg.write_bool(is_world)

        self.env.instance_channel.send_message(msg)

    def Rotate(self, rotation: list, is_world: bool = True):
        """
        物体旋转

        Args:
            rotation: 旋转量
            is_world: 世界空间/局部空间
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Rotate')
        for i in range(3):
            msg.write_float32(rotation[i])
        msg.write_bool(is_world)

        self.env.instance_channel.send_message(msg)

    def SetRotationQuaternion(self, quaternion: list = None, is_world: bool = True):
        """
        使用四元数设置物体旋转

        Args:
            quaternion: 四元数
            is_world: 世界空间or局部空间
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRotationQuaternion')
        assert type(quaternion) == list and len(quaternion) == 3, \
            'Argument quaternion must be a 4-d list.'
        for i in range(4):
            msg.write_float32(quaternion[i])
        msg.write_bool(is_world)

        self.env.instance_channel.send_message(msg)

    def SetActive(self, active: bool):
        """
        设置物体激活状态

        Args:
            active: 激活/非激活
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetActive')
        msg.write_bool(active)

        self.env.instance_channel.send_message(msg)

    def SetParent(self, parent_id: int, parent_name: str = ''):
        """
        设置父物体

        Args:
            parent_id: 父物体ID
            parent_name: 父物体内节点名
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetParent')
        msg.write_int32(parent_id)
        msg.write_string(parent_name)

        self.env.instance_channel.send_message(msg)

    def SetLayer(self, layer: int):
        """
        设置物体层

        Args:
            layer: 层编号
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetLayer')
        msg.write_int32(layer)

        self.env.instance_channel.send_message(msg)

    def Copy(self, new_id: int):
        """
        复制物体

        Args:
            new_id: 新物体的ID
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Copy')
        msg.write_int32(new_id)

        self.env.instance_channel.send_message(msg)

        self.env.attrs[new_id] = type(self)(self.env, new_id, self.data)
        return self.env.attrs[new_id]

    def Destroy(self):
        """
        删除物体
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Destroy')

        self.env.instance_channel.send_message(msg)
        self.env.attrs.pop(self.id)

    def SetRFMoveColliderActive(self, active: bool):
        """
        设置物体在RFMove中的碰撞开关

        Args:
            active:
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRFMoveColliderActive')
        msg.write_bool(active)

        self.env.instance_channel.send_message(msg)

    def GetLoaclPointFromWorld(self, point: list):
        """
        转换局部坐标到世界坐标

        Args:
            point:局部坐标
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetLoaclPointFromWorld')
        msg.write_float32(point[0])
        msg.write_float32(point[1])
        msg.write_float32(point[2])

        self.env.instance_channel.send_message(msg)

    def GetWorldPointFromLocal(self, point: list):
        """
        转换世界坐标到局部坐标

        Args:
            point:世界坐标
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetWorldPointFromLocal')
        msg.write_float32(point[0])
        msg.write_float32(point[1])
        msg.write_float32(point[2])

        self.env.instance_channel.send_message(msg)
