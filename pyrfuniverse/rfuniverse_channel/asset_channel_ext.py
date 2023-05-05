from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class AssetChannelExt:
    """
    This is an example of extend asset channel, without actual functions.
    """
    def __init__(self, env):
        self.env = env
        self.data = {}

    def parse_message(self, msg: IncomingMessage, msg_type: str) -> dict:
        self.data = {}
        # 1.Switch data read based on message type
        if msg_type == 'CustomMessage':
            # 2. Read the message from Unity in order.
            # Note that the reading order here should align with
            # the writing order in CustomMessage() of AssetManagerExt.cs.
            data = msg.read_string()
            self.data['custom_message'] = data
        # 3. Return the data
        return self.data

    def CustomMessage(self, message: str):
        # 1. Define an out-going message
        msg = OutgoingMessage()

        # 2. The first data written must be the function name.
        # The keyword `CustomMessage` here should also be added
        # as a new branch in AnalysisMsg() in AssetManagerExt.cs.
        msg.write_string('CustomMessage')
        # 3. Write in the data.
        msg.write_string(message)

        self.env.asset_channel.send_message(msg)

