import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
import pyrfuniverse.utils.rfuniverse_utility as utility


def GenerateVHACDColider(kwargs: dict) -> OutgoingMessage:
    compulsory_params = ['id']
    optional_params = []
    utility.CheckKwargs(kwargs, compulsory_params)

    msg = OutgoingMessage()
    msg.write_int32(kwargs['id'])
    msg.write_string('GenerateVHACDColider')
    return msg


class ColliderAttr(attr.GameObjectAttr):
    """
    具有碰撞体的物体类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:

        """
        super().parse_message(msg)
        return self.data

    def GenerateVHACDColider(self):
        """
        使用VHACD算法生成物体碰撞体
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GenerateVHACDColider')

        self.env.instance_channel.send_message(msg)