import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class GraspSimAttr(attr.BaseAttr):
    """
    抓取测试农场仿真类
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        消息解析

        Returns:
            self.data['done'] 模拟是否完成

            self.data['points'] 抓点列表

            self.data['quaternions'] 抓点对应的四元数列表

            self.data['width'] 抓点对应的抓型宽度列表

            self.data['success'] 抓点对应的抓取结果列表
        """
        super().parse_message(msg)
        self.data['done'] = msg.read_bool()
        if self.data['done']:
            mode = msg.read_int32()
            if mode == 0:
                self.data['points'] = msg.read_float32_list()
                self.data['quaternions'] = msg.read_float32_list()
                self.data['width'] = msg.read_float32_list()
            if mode == 1:
                self.data['success'] = msg.read_float32_list()
        return self.data

    def StartGraspSim(self, mesh: str, gripper: str, points: list, normals: list, depth_range_min: float, depth_range_max: float, depth_lerp_count: int, angle_lerp_count: int, parallel_count: int = 100):
        """
        开始模拟抓取

        Args:
            mesh: 物体网格路径
            gripper: 夹爪
            points: 抓点列表
            normals: 法线列表
            depth_range_min: 抓型深度范围最小值
            depth_range_max: 抓型深度范围最大值
            depth_lerp_count: 抓型深度插值数量
            angle_lerp_count: 抓型角度插值数量
            parallel_count: 并行抓取数

        Returns:
            当模拟完成时,self.data['done']会被置为True
            self.data['points']为抓点列表
            self.data['quaternions']为抓点对应的四元数列表
            self.data['width']为抓点对应的抓型宽度列表
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('StartGraspSim')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(points)
        msg.write_float32_list(normals)
        msg.write_float32(depth_range_min)
        msg.write_float32(depth_range_max)
        msg.write_int32(depth_lerp_count)
        msg.write_int32(angle_lerp_count)
        msg.write_int32(parallel_count)

        self.env.instance_channel.send_message(msg)

    def GenerateGraspPose(self, mesh: str, gripper: str, points: list, normals: list, depth_range_min: float, depth_range_max: float, depth_lerp_count: int, angle_lerp_count: int):
        """
        生成抓取姿态

        Args:
            mesh: 物体网格路径
            gripper: 夹爪
            points: 抓点列表
            normals: 法线列表
            depth_range_min: 抓型深度范围最小值
            depth_range_max: 抓型深度范围最大值
            depth_lerp_count: 抓型深度插值数量
            angle_lerp_count: 抓型角度插值数量

        Returns:
            生成并显示抓取姿态
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('GenerateGraspPose')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(points)
        msg.write_float32_list(normals)
        msg.write_float32(depth_range_min)
        msg.write_float32(depth_range_max)
        msg.write_int32(depth_lerp_count)
        msg.write_int32(angle_lerp_count)

        self.env.instance_channel.send_message(msg)

    def StartGraspTest(self, mesh: str, gripper: str, points: list, quaternions: list, parallel_count: int = 100):
        """
        对现有抓型进行抓取测试

        Args:
            mesh: 物体网格路径
            gripper: 夹爪
            points: 抓点列表
            quaternions: 四元数列表
            parallel_count: 并行抓取数

        Returns:
            当模拟完成时,self.data['done']会被置为True
            self.data['success']为抓取结果列表
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('StartGraspTest')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(points)
        msg.write_float32_list(quaternions)
        msg.write_int32(parallel_count)

        self.env.instance_channel.send_message(msg)

    def ShowGraspPose(self, mesh: str, gripper: str, positions: list, quaternions: list):
        """
        显示抓取姿态

        Args:
            mesh: 物体网格路径
            gripper: 夹爪
            positions: 抓点列表
            quaternions: 四元数列表
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('ShowGraspPose')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(positions)
        msg.write_float32_list(quaternions)

        self.env.instance_channel.send_message(msg)
