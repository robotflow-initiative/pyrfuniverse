import numpy as np

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)
class PointCloudAttr(attr.BaseAttr):
    def parse_message(self, msg: IncomingMessage) -> dict:
        super().parse_message(msg)
        return self.data

    def ShowPointCloud(self, positions: list[list[float]] = [], colors: list[list[float]] = [], ply_path: str = None, radius: float = 0.01):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('ShowPointCloud')
        msg.write_bool(ply_path is not None)
        if ply_path is not None:
            msg.write_string(ply_path)
        else:
            msg.write_float32_list(np.array(positions).reshape(-1).tolist())
            msg.write_float32_list(np.array(colors).reshape(-1).tolist())
        msg.write_float32(radius)

        self.env.instance_channel.send_message(msg)

    def SetRadius(self, radius: float):
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRadius')
        msg.write_float32(radius)

        self.env.instance_channel.send_message(msg)