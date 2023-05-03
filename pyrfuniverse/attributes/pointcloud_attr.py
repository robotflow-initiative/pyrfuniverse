import numpy as np

import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class PointCloudAttr(attr.BaseAttr):
    """
    Point cloud rendering class.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.
        """
        super().parse_message(msg)
        return self.data

    def ShowPointCloud(self, positions: list = [], colors: list = [], ply_path: str = None, radius: float = 0.01):
        """
        Display point cloud in Unity.

        Args:
            positions: A list of positions of points in a point cloud.
            colors: A list of colors of points (range [0, 1]) in a point cloud.
            ply_path: Str, the absolute path of `.ply` file. If this parameter is specified, `positions`
                and `colors` will be ignored.
            radius: Float, the radius of the point cloud.
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
        Set the radius for points in a point cloud.

        Args:
            radius: Float, the radius.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('SetRadius')
        msg.write_float32(radius)

        self.env.instance_channel.send_message(msg)