from enum import Enum

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class LightType(Enum):
    """
    光源类型,与Unity中的LightType一致
    """
    Spot = 0
    Directional = 1
    Point = 2
    Area = 3  # 不可用
    Disc = 4  # 不可用


class LightShadow(Enum):
    """
    阴影类型,与Unity中的LightShadows一致
    """
    NoneShadow = 0
    Hard = 1
    Soft = 2


class LightAttr(attr.BaseAttr):
    """
    灯光类
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
        设置灯光颜色

        Args:
            color: 颜色,长度为3,分别为RGB三个通道的值,范围为0-1
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetColor')
        for i in range(3):
            msg.write_float32(color[i])

        self.env.instance_channel.send_message(msg)

    def SetType(self, light_type: LightType):
        """
        设置灯光类型

        Args:
            light_type: 灯光类型,参考LightType
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetType')
        msg.write_int32(light_type.value)

        self.env.instance_channel.send_message(msg)

    def SetShadow(self, light_shadow: LightShadow):
        """
        设置灯光阴影类型

        Args:
            light_shadow: 灯光阴影类型,参考LightShadow
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetShadow')
        msg.write_float32(light_shadow.value)

        self.env.instance_channel.send_message(msg)

    def SetIntensity(self, light_intensity: float):
        """
        设置灯光强度

        Args:
            light_intensity: 灯光强度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIntensity')
        msg.write_float32(light_intensity)

        self.env.instance_channel.send_message(msg)

    def SetRange(self, light_range: float):
        """
        设置灯光范围(仅灯光类型为Spot和Point时有效)

        Args:
            light_range: 灯光范围
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRange')
        msg.write_float32(light_range)

        self.env.instance_channel.send_message(msg)

    def SetSpotAngle(self, spot_angle: float):
        """
        设置灯光角度(仅灯光类型为Spot时有效)

        Args:
            spot_angle: 灯光角度
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetSpotAngle')
        msg.write_float32(spot_angle)

        self.env.instance_channel.send_message(msg)
