from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)

class AssetChannelExt:
    def __init__(self, env):
        self.env = env
        self.data = {}

    # 消息解析
    def parse_message(self, msg: IncomingMessage, msg_type: str) -> dict:
        self.data = {}
        # 根据头字符串添加自己的分支
        # 此处CustomMessage对应Unity中AssetManagerExt脚本CustomMessage接口函数中写入的第一个头字符串
        if msg_type == 'CustomMessage':
            # 按顺序读取数据
            # 此处读取顺序对应Unity中使用SendMetaDataToPython前写入顺序
            data = msg.read_string()
            self.data['custom_message'] = data
        # 将数据写入到data中返回
        return self.data

    # 新增接口示例
    def CustomMessage(self, message: str):
        msg = OutgoingMessage()
        # 第一个写入的数据必须是消息类型
        # 此处CustomMessage对应Unity中AssetManagerExt脚本AnalysisMsg函数switch的CustomMessage分支
        msg.write_string('CustomMessage')
        # 按顺序写入自己想要发送的数据
        msg.write_string(message)

        self.env.asset_channel.send_message(msg)

