import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility
import base64


def AlignView(kwargs: dict):
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('AlignView')

    return msg

def GetRGB(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetRGB')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetNormal(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetNormal')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetID(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetID')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetDepth(kwargs: dict):
    compulsory_params = ['id', 'zero_dis', 'one_dis']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetDepth')
    msg.write_float32(kwargs['zero_dis'])
    msg.write_float32(kwargs['one_dis'])
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetDepthEXR(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetDepthEXR')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg

def GetAmodalMask(kwargs: dict):
    compulsory_params = ['id']
    optional_params = ['width', 'height', 'fov', 'intrinsic_matrix']
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GetAmodalMask')
    if 'intrinsic_matrix' in kwargs:
        msg.write_bool(True)
        msg.write_float32_list(kwargs['intrinsic_matrix'])
    else:
        msg.write_bool(False)
        msg.write_int32(kwargs['width'])
        msg.write_int32(kwargs['height'])
        if 'fov' in kwargs:
            msg.write_float32(kwargs['fov'])
        else:
            msg.write_float32(60)
    return msg


class CameraAttr(attr.BaseAttr):
    """
    Camera attribute class, which can capture many kinds of screenshot
    of the scene in RFUniverse.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.
        
            self.data['width']: The width of image.

            self.data['height']: The height of image.

            self.data['fov']: The field of view of camera.

            self.data['rgb']: The bytes of rgb image.

            self.data['normal']: The bytes of normal image.

            self.data['id_map']: The bytes of instance segmentation mask image.

            self.data['depth']: The bytes of depth image.

            self.data['depth_exr']: The bytes of depth image in exr format.

            self.data['amodal_mask']: The bytes of amodal mask image.

            self.data['heat_map']: The bytes of heat map image.

            self.data['2d_bounding_box']: The 2d bouding box of objects in camera (image) coordinate.

            self.data['3d_bounding_box']: The 3d bounding box of objects in world coordinate.
        """
        super().parse_message(msg)
        self.data['width'] = msg.read_int32()
        self.data['height'] = msg.read_int32()
        self.data['fov'] = msg.read_float32()
        if msg.read_bool() is True:
            self.data['rgb'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['normal'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['id_map'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['depth'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['depth_exr'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['amodal_mask'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            self.data['heat_map'] = base64.b64decode(msg.read_string())
        if msg.read_bool() is True:
            ddbbox_count = msg.read_int32()
            self.data['2d_bounding_box'] = {}
            for i in range(ddbbox_count):
                item_id = msg.read_int32()
                self.data['2d_bounding_box'][item_id] = {}
                self.data['2d_bounding_box'][item_id]['position'] = [msg.read_float32() for _ in range(2)]
                self.data['2d_bounding_box'][item_id]['size'] = [msg.read_float32() for _ in range(2)]
        if msg.read_bool() is True:
            dddbbox_count = msg.read_int32()
            self.data['3d_bounding_box'] = {}
            for i in range(dddbbox_count):
                item_id = msg.read_int32()
                self.data['3d_bounding_box'][item_id] = {}
                self.data['3d_bounding_box'][item_id]['position'] = [msg.read_float32() for _ in range(3)]
                self.data['3d_bounding_box'][item_id]['rotation'] = [msg.read_float32() for _ in range(3)]
                self.data['3d_bounding_box'][item_id]['size'] = [msg.read_float32() for _ in range(3)]
        return self.data

    def AlignView(self):
        """
        Make the camera in RFUniverse align the current view in GUI.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AlignView')

        self.env.instance_channel.send_message(msg)

    def GetRGB(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the camera RGB image.

        Args:
            width: Int, the width of image.
            height: Int, the height of image.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetRGB')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetNormal(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the normal image in world coordinate.

        Args:
            width: Int, the width of image.
            height: Int, the height of image.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetNormal')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetID(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the instance segmentation mask image. The color for each pixel is computed from object ID, see `pyrfuniverse.utils.rfuniverse_utility.GetColorFromID` for more details.

        Args:
            width: Int, the width of image.
            height: Int, the height of image.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetID')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetDepth(self, zero_dis: float, one_dis: float, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the depth image from camera. Since eacg pixel of depth image returned from this function is 8-bit, user should limit the depth range (`zero_dis` and `one_dis`) for more accurate results.

        Args:
            zero_dis: The minimum distance in calculation.
            one_dis: The maximum distance in calculation.
            width: Int, the width of image.
            height: Int, the height of image.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetDepth')
        msg.write_float32(zero_dis)
        msg.write_float32(one_dis)
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetDepthEXR(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the depth image from camera. This function returns EXR format image bytes and each pixel is 32-bit.

        Args:
            width: Int, the width of image.
            height: Int, the height of image.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetDepthEXR')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def GetAmodalMask(self, target_id: int, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the amodal mask image for target object.

        Args:
            target_id: The target object ID.
            width: Int, the width of image.
            height: Int, the height of image.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetAmodalMask')
        msg.write_int32(target_id)
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def StartHeatMapRecord(self, targets_id: list):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('StartHeatMapRecord')
        msg.write_int32(len(targets_id))
        for target in targets_id:
            msg.write_int32(target)

        self.env.instance_channel.send_message(msg)

    def EndHeatMapRecord(self):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('EndHeatMapRecord')

        self.env.instance_channel.send_message(msg)

    def GetHeatMap(self, width: int = 512, height: int = 512, radius: int = 50, fov: float = 60., intrinsic_matrix=None):
        """
        Get the heat map image.

        Args:
            width: Int, the width of image.
            height: Int, the height of image.
            radius: The radius of heat map.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetHeatMap')
        msg.write_int32(radius)
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def Get2DBBox(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        Get the 2d bounding box of objects in current camera view.

        Args:
            width: Int, the width of image.
            height: Int, the height of image.
            radius: The radius of heat map.
            fov: Float, the field of view for camera.
            intrinsic_matrix: A list of length 9, representing the camera intrinsic matrix. When this parameter is passed, `width`, `height` and `fov` will be ignroed.
        """
        if intrinsic_matrix is None:
            intrinsic_matrix = []

        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Get2DBBox')
        if len(intrinsic_matrix) == 9:
            msg.write_bool(True)
            msg.write_float32_list(intrinsic_matrix)
        else:
            msg.write_bool(False)
            msg.write_int32(width)
            msg.write_int32(height)
            msg.write_float32(fov)

        self.env.instance_channel.send_message(msg)

    def Get3DBBox(self):
        """
        Get the 3d bounding box of objects in world coordinate.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Get3DBBox')

        self.env.instance_channel.send_message(msg)

