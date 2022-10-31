from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
from pyrfuniverse.rfuniverse_channel import RFUniverseChannel


class DebugChannel(RFUniverseChannel):

    def __init__(self, channel_id: str) -> None:
        super().__init__(channel_id)
        self.count = 0
        self.data = {}

    def _parse_message(self, msg: IncomingMessage) -> None:
        return



