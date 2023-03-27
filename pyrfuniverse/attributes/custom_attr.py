import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


#自定义Attr类示例
class CustomAttr(attr.BaseAttr):
    """
    此类为自定义Attr类的示例，不包含实际功能
    """
    # 消息解析示例
    def parse_message(self, msg: IncomingMessage) -> dict:
        # 先完成所继承的基类的数据读取
        super().parse_message(msg)
        # 按顺序读取数据
        # 此处读取顺序对应Unity的CustomAttr脚本CollectData函数中的写入顺序
        self.data['custom_message'] = msg.read_string()
        return self.data

    # 新增接口示例
    def CustomMessage(self, message: str):
        msg = OutgoingMessage()

        # 第一个写入的数据必须是ID
        msg.write_int32(self.id)
        # 第二个写入的数据必须是消息类型 此处CustomMessage对应Unity新增Attr脚本AnalysisMsg函数switch的一个分支
        msg.write_string('CustomMessage')

        # 写入数据
        msg.write_string(message)

        self.env.instance_channel.send_message(msg)
