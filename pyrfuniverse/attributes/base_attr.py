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
    Base attribute class, which includes general functions such as 
    object loading, deleting and transforming.
    """
    def __init__(self, env, id: int, data=None):
        if data is None:
            data = {}
        self.env = env
        self.id = id
        self.data = data

    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['name']: The name of the object.

            self.data['position']: The position of the object in world coordinate.

            self.data['rotation']: The euler angle of the object in world coordinate.

            self.data['quaternion']: The quaternion of the object in world coordinate.

            self.data['local_position']: The position of the object in its parent's local coordinate.

            self.data['local_rotation']: The euler angle of the object in its parent's local coordinate.

            self.data['local_quaternion']: The quaternion of the object in its parent's local coordinate.

            self.data['local_to_world_matrix']: The transformation matrix from local to world coordinate.

            self.data['result_local_point']: The result of transforming object from local to world coordinate.

            self.data['result_world_point']: The result of transforming object from world to local coordinate.
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
        Set the attribute type of this object

        Args:
            attr_type: Any attribute in pyrfuniverse.attributes.

        Returns:
            The target attribute.
        """
        self.env.attrs[self.id] = attr_type(self.env, self.id, self.data)
        return self.env.attrs[self.id]

    def SetTransform(self, position: list = None, rotation: list = None, scale: list = None, is_world: bool = True):
        """
        Set the transform of this object, including position, rotation, scale and coordinate.

        Args:
            position: A list of length 3, representing the target position value of object.
            rotation: A list of length 3, representing the target euler angle value of object.
            scale: A list of length 3, representing the target scale value of object.
            is_world: Bool, True for world coordinate, False for local coordinate.
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
        Translate this object.

        Args:
            translation: A list of length 3, representing the translation from current position.
            is_world: Bool, True for world coordinate, False for local coordinate.
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
        Rotate this object.

        Args:
            rotation: A list of length 3, representing the euler-angle-format rotation from current euler angle.
            is_world: Bool, True for world coordinate, False for local coordinate.
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
        Rotate this object using quaternion.

        Args:
            quaternion: A list of length 4, representing the quaternion from current pose.
            is_world: Bool, True for world coordinate, False for local coordinate.
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
        Set the activeness of this obeject.

        Args:
            active: Bool, True for active, False for inactive.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetActive')
        msg.write_bool(active)

        self.env.instance_channel.send_message(msg)

    def SetParent(self, parent_id: int, parent_name: str = ''):
        """
        Set the parent of this object.

        Args:
            parent_id: Int, the id of parent object.
            parent_name: Str, the name of parent object.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetParent')
        msg.write_int32(parent_id)
        msg.write_string(parent_name)

        self.env.instance_channel.send_message(msg)

    def SetLayer(self, layer: int):
        """
        Set the layer in Unity of this object.

        Args:
            layer: Int, the number of layer.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetLayer')
        msg.write_int32(layer)

        self.env.instance_channel.send_message(msg)

    def Copy(self, new_id: int):
        """
        Duplicate an object.

        Args:
            new_id: Int, the id of new object.
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
        Destroy this object in Unity.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Destroy')

        self.env.instance_channel.send_message(msg)
        self.env.attrs.pop(self.id)

    def SetRFMoveColliderActive(self, active: bool):
        """
        Set the collider active or inactive in RFMove.

        Args:
            active: Bool, True for active and False for inactive.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRFMoveColliderActive')
        msg.write_bool(active)

        self.env.instance_channel.send_message(msg)

    def GetLoaclPointFromWorld(self, point: list):
        """
        Transform a point from local coordinate to world coordinate.

        Args:
            point: A list of length 3, representing the position of a point.
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
        Transform a point from world coordinate to local coordinate.

        Args:
            point: A list of length 3, representing the position of a point.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetWorldPointFromLocal')
        msg.write_float32(point[0])
        msg.write_float32(point[1])
        msg.write_float32(point[2])

        self.env.instance_channel.send_message(msg)
