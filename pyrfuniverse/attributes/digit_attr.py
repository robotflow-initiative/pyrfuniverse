import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import base64


class DigitAttr(attr.BaseAttr):
    """
    模拟Digit指尖触觉传感器类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        消息解析

        Returns:
            self.data['light'] RGB灯光图像bytes

            self.data['depth'] 深度图像bytes
        """
        super().parse_message(msg)
        if msg.read_bool() is True:
            self.data['light'] = base64.b64decode(msg.read_string())
            self.data['depth'] = base64.b64decode(msg.read_string())
        return self.data


    def GetData(self):
        """
        获取传感器数据

        Returns:
            调用此接口并step后,从
            self.data['light']: 获取RGB灯光图像,
            self.data['depth']: 获取深度图像
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GetData')

        self.env.instance_channel.send_message(msg)