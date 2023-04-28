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
    相机类,可获取各种相机截图
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:
            self.data['width'] 图像宽度

            self.data['height'] 图像高度

            self.data['fov'] 相机FOV

            self.data['rgb'] RGB图像bytes

            self.data['normal'] 法线图像bytes

            self.data['id_map'] ID图像bytes

            self.data['depth'] 深度图像bytes

            self.data['depth_exr'] EXR深度图像bytes

            self.data['amodal_mask'] amodal_mask图bytes

            self.data['heat_map'] heat_map图bytes

            self.data['2d_bounding_box'] 相机屏幕坐标系下2d_bounding_box数据

            self.data['3d_bounding_box'] 世界空间3d_bounding_box数据
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
        使相机对准当前视口视角
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('AlignView')

        self.env.instance_channel.send_message(msg)

    def GetRGB(self, width: int = 512, height: int = 512, fov: float = 60., intrinsic_matrix=None):
        """
        获取相机的RGB图像

        Args:
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['rgb']获取结果
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
        获取相机的世界空间法线图像

        Args:
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['normal']获取结果
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
        获取相机的ID图像,每个像素的颜色值由物体ID计算而来,见rfuniverse_utility.GetColorFromID

        Args:
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['id_map']获取结果
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
        获取相机的深度图像,由于8位深度图精度低,因此需要限制范围以提高精度

        Args:
            zero_dis: 黑色像素表示深度
            one_dis: 白像素表示深度
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['depth']获取结果
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
        获取相机的世界空间深度图像,EXR格式图像支持存储32位信息,因此无需限制范围

        Args:
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['depth_exr']获取结果
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
        获取目标物体的Amodal Mask图像

        Args:
            target_id: 目标物体ID
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['amodal_mask']获取结果
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
        获取目标物体的HeatMap图像

        Args:
            width: 图像分辨率宽度
            height: 图像分辨率高度
            radius: 热力图半径
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['heat_map']获取结果
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
        获取相机中物体的2DBBox

        Args:
            width: 图像分辨率宽度
            height: 图像分辨率高度
            fov: 相机FOV
            intrinsic_matrix: List[9]相机内参,当传入时,width、height、fov参数无效

        Returns:
            调用此接口并step后,从self.data['2d_bounding_box']获取结果
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
        获取物体的3DBBox

        Returns:
            调用此接口并step后,从self.data['3d_bounding_box']获取结果
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('Get3DBBox')

        self.env.instance_channel.send_message(msg)

