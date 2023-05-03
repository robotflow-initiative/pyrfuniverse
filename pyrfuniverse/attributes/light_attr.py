from enum import Enum

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class LightType(Enum):
    """
    The type of light, keeping same name with LightType (https://docs.unity3d.com/ScriptReference/LightType.html) in Unity.
    """
    Spot = 0
    Directional = 1
    Point = 2
    Area = 3  # 不可用
    Disc = 4  # 不可用


class LightShadow(Enum):
    """
    The type of shadow, keeping same name with LightShadows (https://docs.unity3d.com/ScriptReference/LightShadows.html) in Unity.
    """
    NoneShadow = 0
    Hard = 1
    Soft = 2


class LightAttr(attr.BaseAttr):
    """
    Light attribute class.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.
        """
        super().parse_message(msg)
        return self.data

    def SetColor(self, color: list):
        """
        Set the color of light.

        Args:
            color: A list of length 3, representing the R, G and B channel, in range [0, 1].
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetColor')
        for i in range(3):
            msg.write_float32(color[i])

        self.env.instance_channel.send_message(msg)

    def SetType(self, light_type: LightType):
        """
        Set the type of light.

        Args:
            light_type: LightType, the type of light.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetType')
        msg.write_int32(light_type.value)

        self.env.instance_channel.send_message(msg)

    def SetShadow(self, light_shadow: LightShadow):
        """
        Set the type of shadow.

        Args:
            light_shadow: LightShadow, the type of the shadow.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetShadow')
        msg.write_float32(light_shadow.value)

        self.env.instance_channel.send_message(msg)

    def SetIntensity(self, light_intensity: float):
        """
        Set the intensity of light.

        Args:
            light_intensity: Float, the intensity of light.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetIntensity')
        msg.write_float32(light_intensity)

        self.env.instance_channel.send_message(msg)

    def SetRange(self, light_range: float):
        """
        Set the range of light. (Only available when the LightType is `LightType.Spot` or `LightType.Point`)

        Args:
            light_range: Float, the range of light.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRange')
        msg.write_float32(light_range)

        self.env.instance_channel.send_message(msg)

    def SetSpotAngle(self, spot_angle: float):
        """
        Set the angle of light. (Only available when the LightType is `LightType.Spot`)

        Args:
            spot_angle: Float, the angle of light.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetSpotAngle')
        msg.write_float32(spot_angle)

        self.env.instance_channel.send_message(msg)
