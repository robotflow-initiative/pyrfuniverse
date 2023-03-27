import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility

def Translate(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'translation']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('Translate')
    for i in range(3):
        msg.write_float32(kwargs['translation'][i])

    return msg


def Rotate(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'rotation']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('Rotate')
    for i in range(3):
        msg.write_float32(kwargs['rotation'][i])

    return msg


def SetColor(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id', 'color']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)
    msg = OutgoingMessage()

    msg.write_int32(kwargs['id'])
    msg.write_string('SetColor')
    for i in range(4):
        msg.write_float32(kwargs['color'][i])

    return msg


class GameObjectAttr(attr.BaseAttr):
    """
    基本视觉物体类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:

        """
        super().parse_message(msg)
        return self.data

    def SetColor(self, color: list):
        """
        设置物体颜色

        Args:
            color: 颜色，长度为4，分别为RGBA[0-1]
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetColor')
        for i in range(4):
            msg.write_float32(color[i])

        self.env.instance_channel.send_message(msg)

    def EnabledRender(self, enabled: bool):
        """
        启用或禁用渲染

        Args:
            enabled: 是否启用渲染
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('EnabledRender')
        msg.write_bool(enabled)

        self.env.instance_channel.send_message(msg)

    def SetTexture(self, path: str):
        """
        设置物体纹理

        Args:
            path: 纹理贴图绝对路径
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetTexture')
        msg.write_string(path)

        self.env.instance_channel.send_message(msg)