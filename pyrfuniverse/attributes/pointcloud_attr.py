import numpy as np

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class PointCloudAttr(attr.BaseAttr):
    """
    点云渲染类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        解析消息

        Returns:

        """
        super().parse_message(msg)
        return self.data

    def ShowPointCloud(self, positions: list = [], colors: list = [], ply_path: str = None, radius: float = 0.01):
        """
        显示点云

        Args:
            positions: 点位置列表
            colors: 点颜色列表
            ply_path: .ply文件列表,如果不为空,则positions和colors参数无效
            radius: 点半径
        """
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
        """
        设置点半径

        Args:
            radius: 半径
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRadius')
        msg.write_float32(radius)

        self.env.instance_channel.send_message(msg)