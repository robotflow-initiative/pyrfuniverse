import pyrfuniverse.attributes as attr
from pyrfuniverse.side_channel.side_channel import (
    IncomingMessage,
    OutgoingMessage,
)


class GraspSimAttr(attr.BaseAttr):
    """
    Grasp pose simulation class.
    """
    def parse_message(self, msg: IncomingMessage) -> dict:
        """
        Parse messages. This function is called by internal function.

        Returns:
            Dict: A dict containing useful information of this class.

            self.data['done']: Whether the simulation is done

            self.data['points']: The list of grasp points.

            self.data['quaternions']: The list of grasping pose quaternions.

            self.data['width']: The list of gripper width of grasping pose.

            self.data['success']: The list of success or failure of the grasing pose.
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
        Start simulating grasping.

        Args:
            mesh: Str, the absolute path to .obj file.
            gripper: Str, the name of the gripper.
            points: A list of float, representing the grasping points.
            normals: A list of float, representing the normals.
            depth_range_min: Float, the minimum depth of grasp pose.
            depth_range_max: Float, the maximum depth of grasp pose.
            depth_lerp_count: Int, the interpolation count of depth.
            angle_lerp_count: Int, the interpolation count of angle.
            parallel_count: Int, the count of parallel grasping.
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
        Generate grasp poses and visualize grasp results.

        Args:
            mesh: Str, the absolute path to .obj file.
            gripper: Str, the name of the gripper.
            points: A list of float, representing the grasping points.
            normals: A list of float, representing the normals.
            depth_range_min: Float, the minimum depth of grasp pose.
            depth_range_max: Float, the maximum depth of grasp pose.
            depth_lerp_count: Int, the interpolation count of depth.
            angle_lerp_count: Int, the interpolation count of angle.
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
        Start testing the grasp based on current grasp poses.

        Args:
            mesh: Str, the absolute path to .obj file.
            gripper: Str, the name of the gripper.
            points: A list of float, representing the grasping points.
            quaternions: A list of float, representing the quaternions.
            parallel_count: Int, the interpolation count of angle.
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
        Display grasp poses.

        Args:
            mesh: Str, the absolute path to .obj file.
            gripper: Str, the name of the gripper.
            points: A list of float, representing the grasping points.
            quaternions: A list of float, representing the quaternions.
        """
        msg = OutgoingMessage()

        msg.write_int32(self.id)
        msg.write_string('ShowGraspPose')
        msg.write_string(mesh)
        msg.write_string(gripper)
        msg.write_float32_list(positions)
        msg.write_float32_list(quaternions)

        self.env.instance_channel.send_message(msg)
