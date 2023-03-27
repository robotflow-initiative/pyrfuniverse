import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)

class SoftbodyAttr(attr.BaseAttr):
    """
    ObiSoftbody类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:

        """
        super().parse_message(msg)
        return self.data