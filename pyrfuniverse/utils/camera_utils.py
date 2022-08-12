import math
import numpy as np
import pybullet as p
import cv2


def save_image(img: np.ndarray, path: str):
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    cv2.imwrite(path, img)


class RFUniverseCamera:
    """Summary of Camera Class here.

        This class is a camera class.
        Github: https://github.com/ElectronicElephant/pybullet_ur5_robotiq/blob/main/utilities.py
                https://github.com/bulletphysics/bullet3/issues/1616

    Attributes:
        * @param near   Number Distance to the near clipping plane along the -Z axis
        * @param far    Number Distance to the far clipping plane along the -Z axis
    """
    def __init__(
            self,
            width,
            height,
            near_plane,
            far_plane,
            fov=90
    ):
        self.width, self.height = width, height
        self.aspect = self.width / self.height
        self.near, self.far = near_plane, far_plane
        self.fov = fov
        self.projection_matrix = p.computeProjectionMatrixFOV(self.fov, self.aspect, self.near, self.far)
        # pybullet requires the fov here to be in degrees.

        # rot_matrix = p.getMatrixFromQuaternion(cam_orn)
        # self.cam_rot_matrix = np.array(rot_matrix).reshape(3, 3)
        # Initial vectors
        # init_camera_vector = (1, 0, 0)  # z-axis
        # init_up_vector = (0, -1, 0)  # y-axis
        # Rotated vectors

        # self.camera_vector = self.cam_rot_matrix.dot(init_camera_vector)
        # self.up_vector = self.cam_rot_matrix.dot(init_up_vector)

        # self.view_matrix = p.computeViewMatrix(cam_pos, cam_pos + 0.1 * self.camera_vector, self.up_vector)

        # self._view_matrix = np.array(self.view_matrix).reshape((4, 4), order='F')
        self._projection_matrix = np.array(self.projection_matrix).reshape((4, 4), order='F')
        # order='F' means column first for pybullet magical configuration
        # self.tran_pix_world = np.linalg.inv(self._projection_matrix @ self._view_matrix)

        # https://stackoverflow.com/questions/60430958/understanding-the-view-and-projection-matrix-from-pybullet
        h = self.width
        self.f = h / (2 * math.tan(math.radians(self.fov / 2)))    # the equation need (fov / 2) in radius
        k = np.zeros((3, 3))
        self.intrinsic_matrix = np.array([[self.f, 0, self.width / 2],
                                          [0, self.f, self.height / 2],
                                          [0, 0, 1]])

    # def get_matrix_from_camera_2_world(self):
    #     """
    #         m_rotation, m_translation = camera.get_matrix_from_camera_2_world()
    #         point_in_camera_frame = np.array([1, 5, 8])
    #         point_in_world_frame = m_rotation @ point + m_translation
    #         calculated_point_in_camera_frame = np.linalg.inv(m_rotation) @ (point - m_translation)
    #         assert(np.array_equal(point_in_camera_frame, calculated_point_in_camera_frame) == True)
    #     """
    #     b_rotation = np.array(p.getMatrixFromQuaternion(self.cam_orn)).reshape(3, 3)
    #     m_rotation = np.zeros(b_rotation.shape)
    #     m_rotation[:, 0] = b_rotation[:, 2]
    #     m_rotation[:, 1] = b_rotation[:, 1]
    #     m_rotation[:, 2] = b_rotation[:, 0]
    #     m_rotation[0, 0] = -1 * m_rotation[0, 0]
    #     m_rotation[1, 0] = -1 * m_rotation[1, 0]
    #     m_rotation[0, 2] = -1 * m_rotation[0, 2]
    #     m_translation = self.cam_pos
    #     # I don't know why, but it works
    #     return m_rotation.T, m_translation

    # def get_a_point_in_camera_frame(self, point_in_world_frame):
    #     m_rotation, m_translation = self.get_matrix_from_camera_2_world()
    #     calculated_point_in_camera_frame = np.linalg.inv(m_rotation) @ (point_in_world_frame - m_translation)
    #     return calculated_point_in_camera_frame
    #
    # def get_a_point_in_world_frame(self, point_in_camera_frame):
    #     m_rotation, m_translation = self.get_matrix_from_camera_2_world()
    #     calculated_point_in_world_frame = m_rotation @ point_in_camera_frame + m_translation
    #     return calculated_point_in_world_frame
    #
    # def rgbd_2_world(self, w, h, d):  # get the pointCloud of one single point.
    #     x = (2 * w - self.width) / self.width
    #     y = -(2 * h - self.height) / self.height
    #     z = 2 * d - 1
    #     pix_pos = np.array((x, y, z, 1))
    #     position = self.tran_pix_world @ pix_pos
    #     position /= position[3]
    #     return position[:3]
    #
    # def shot(self):
    #     # Get depth values using the OpenGL renderer
    #     _w, _h, rgb, depth, seg = p.getCameraImage(self.width, self.height,
    #                                                self.view_matrix, self.projection_matrix)
    #     depth_buffer_opengl = np.reshape(depth, [self.width, self.height])
    #     real_depth_buffer = self.far * self.near / (self.far - (self.far - self.near) * depth_buffer_opengl)
    #     return rgb, depth, seg, real_depth_buffer
    #
    # def rgbd_2_world_pointcloud(self, depth):
    #     x = (2 * np.arange(0, self.width) - self.width) / self.width
    #     x = np.repeat(x[None, :], self.height, axis=0)
    #     y = -(2 * np.arange(0, self.height) - self.height) / self.height
    #     y = np.repeat(y[:, None], self.width, axis=1)
    #     z = 2 * depth - 1
    #
    #     pix_pos = np.array([x.flatten(), y.flatten(), z.flatten(), np.ones_like(z.flatten())]).T
    #     position = self.tran_pix_world @ pix_pos.T
    #     position = position.T
    #     # print(position)
    #
    #     position[:, :] /= position[:, 3:4]
    #
    #     return position[:, :3]  # .reshape(*x.shape, -1)

    def depth_image_2_depth(self, depth_img: np.ndarray):
        """ Convert a 3-channel depth image to a 1-channel depth matrix
            Input:
                depth_img: numpy.ndarray
                    Depth image in shape (H, W, 3)
                    Height and width must match the initialization of this camera.

            Output:
                depth: numpy.ndarray
                    Converted depth matrix in shape (H, W)
        """
        assert depth_img.shape[0] == self.height and \
            depth_img.shape[1] == self.width and \
            depth_img.shape[2] == 3

        image_depth_out = (
                depth_img[:, :, 0]
                + depth_img[:, :, 1] / np.float32(256)
                + depth_img[:, :, 2] / np.float32(256 ** 2)
        )
        depth = image_depth_out * (self.far - self.near) / 255.0

        return depth

    def depth_2_camera_pointcloud(self, depth):
        """ Generate point cloud using depth image only.
            author: GraspNet baseline

            Input:
                depth: [numpy.ndarray, (H,W), numpy.float32]
                    depth image
                camera: [CameraInfo]
                    camera intrinsics
                organized: bool
                    whether to keep the cloud in image shape (H,W,3)

            Output:
                cloud: [numpy.ndarray, (H,W,3)/(H*W,3), numpy.float32]
                    generated cloud, (H,W,3) for organized=True, (H*W,3) for organized=False
        """
        xmap = np.arange(self.width)
        ymap = np.arange(self.height)
        xmap, ymap = np.meshgrid(xmap, ymap)  # 0~999

        fx = fy = self.f
        cx = self.width / 2
        cy = self.height / 2
        points_z = depth
        points_x = (xmap - cx) * points_z / fx
        points_y = (ymap - cy) * points_z / fy
        cloud = np.stack([points_x, points_y, points_z], axis=-1)
        cloud = cloud.reshape([-1, 3])
        return cloud

    def get_pixel_position(self, depth):
        x = (2 * np.arange(0, self.width) - self.width) / self.width
        x = np.repeat(x[None, :], self.height, axis=0)
        y = -(2 * np.arange(0, self.height) - self.height) / self.height
        y = np.repeat(y[:, None], self.width, axis=1)
        z = 2 * depth - 1

        pix_pos = np.array([x.flatten(), y.flatten(), z.flatten(), np.ones_like(z.flatten())]).T
        return pix_pos

    def cut_pc_with_seg(self, pc, seg, index):
        pc = pc.reshape(self.width, self.height, 3)
        result = list()
        for i in range(self.width):
            for j in range(self.height):
                if seg[i][j] == index:
                    result.append(pc[i][j])
                    # print(i, j, " ", pc[i][j])
        return np.array(result)
