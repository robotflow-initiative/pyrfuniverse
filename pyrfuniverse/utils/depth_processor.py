import math
import os
import os.path as osp
import numpy as np
import open3d as o3d
import cv2


def image_bytes_to_point_cloud(rgb_bytes: bytes, depth_bytes: bytes, fov: float, extrinsic_matrix: np.ndarray):
    """
    Convert bytes to images, then convert them into point cloud
    """
    image_rgb = np.frombuffer(rgb_bytes, dtype=np.uint8)
    image_rgb = cv2.imdecode(image_rgb, cv2.IMREAD_COLOR)
    image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
    image_rgb = np.transpose(image_rgb, [1, 0, 2])

    # image_depth = np.frombuffer(depth_bytes, dtype=np.float16)
    # image_depth = cv2.imdecode(image_depth, cv2.IMREAD_UNCHANGED)

    # image_depth = np.frombuffer(depth_bytes, dtype=np.uint8)
    # image_depth = cv2.imdecode(image_depth, cv2.IMREAD_GRAYSCALE)
    # image_depth = image_depth * 5 / 255

    temp_file_path = osp.join('/tmp', 'temp_img.exr')
    with open(temp_file_path, 'wb') as f:
        f.write(depth_bytes)
    image_depth = cv2.imread(temp_file_path, cv2.IMREAD_UNCHANGED)
    os.remove(temp_file_path)

    image_depth = np.transpose(image_depth, [1, 0])

    points = depth_to_point_cloud(image_depth, fov=fov, organized=False)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    colors = image_rgb.reshape(-1, 3)
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32) / 255.0)

    # matrix[2, :] = - matrix[2, :]
    # matrix[:, 2] = - matrix[:, 2]
    pcd.transform(extrinsic_matrix)

    # change from Unity coordinates to Open3D coordniates
    # foreground_pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, -1]])


    # coorninate = o3d.geometry.TriangleMesh.create_coordinate_frame(size=1, origin=(0., 0., 0.))
    # o3d.visualization.draw_geometries([foreground_pcd, coorninate])

    pc_sim = np.asarray(pcd.points)
    pc_sim_rgb = (np.asarray(pcd.colors) * 255).astype(np.uint8)
    # return pc_sim, pc_sim_rgb
    return pcd


def image_array_to_point_cloud(image_rgb: np.ndarray, image_depth: np.ndarray, fov: float, extrinsic_matrix: np.ndarray):
    points = depth_to_point_cloud(image_depth, fov=fov, organized=False)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    colors = image_rgb.reshape(-1, 3)
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32) / 255.0)

    pcd.transform(extrinsic_matrix)

    return pcd

def depth_to_point_cloud(depth: np.ndarray, fov: float, organized=False):
    """
    Generate point cloud using depth image only.
        author: GraspNet baseline

        Args:
            depth: [numpy.ndarray, (W,H), numpy.float32] Depth image
            fov: [float] Field Of View for camera
            organized: [bool] Whether to keep the cloud in image shape (W,H,3)

        Return:
            cloud: [numpy.ndarray, (W,H,3)/(W*H,3), numpy.float32]
                generated cloud, (W,H,3) for organized=True, (W*H,3) for organized=False
    """
    width = depth.shape[0]
    height = depth.shape[1]
    cx = width / 2
    cy = height / 2
    xmap = np.arange(width)
    ymap = np.arange(height)
    xmap, ymap = np.meshgrid(xmap, ymap)
    xmap = xmap.T
    ymap = ymap.T

    fx = fy = height / (2 * math.tan(math.radians(fov / 2)))

    points_z = depth
    points_x = (xmap - cx) * points_z / fx
    points_y = (ymap - cy) * points_z / fy

    # radian_per_pixel = math.radians(fov / height)
    # points_x = np.sin(radian_per_pixel * (xmap - cx)) * depth
    # points_y = np.sin(radian_per_pixel * (ymap - cy)) * depth
    # points_z = depth ** 2 - points_x ** 2 - points_y ** 2
    # points_z[points_z < 0] = 0
    # points_z = np.sqrt(points_z)

    cloud = np.stack([points_x, -points_y, points_z], axis=-1)
    if not organized:
        cloud = cloud.reshape([-1, 3])

    return cloud

def image_bytes_to_point_cloud_intrinsic_matrix(rgb_bytes: bytes, depth_bytes: bytes, intrinsic_matrix: np.ndarray, extrinsic_matrix: np.ndarray):
    temp_file_path = osp.join('/tmp', 'temp_img.png')
    with open(temp_file_path, 'wb') as f:
        f.write(rgb_bytes)
    color = o3d.io.read_image(temp_file_path)
    os.remove(temp_file_path)

    temp_file_path = osp.join('/tmp', 'temp_img.exr')
    with open(temp_file_path, 'wb') as f:
        f.write(depth_bytes)
    # change .exr format to .png format
    depth_exr = cv2.imread(temp_file_path, cv2.IMREAD_UNCHANGED)
    os.remove(temp_file_path)
    # mask = np.asarray(o3d.io.read_image(osp.join(mask_path, video_name, view_name, base_name + '.png')))[:, :, 0]
    # foregound_mask = mask == 11
    depth_png = (depth_exr * 1000).astype(np.uint16)[:, :]
    # foreground_depth_png[~foregound_mask] = 0  # filter the background, only need foreground
    temp_file_path = osp.join('/tmp', 'temp_img.png')
    cv2.imwrite(temp_file_path, depth_png)
    depth = o3d.io.read_image(temp_file_path)
    os.remove(temp_file_path)

    pcd = image_open3d_to_point_cloud_intrinsic_matrix(color, depth, intrinsic_matrix, extrinsic_matrix)

    return pcd


def image_array_to_point_cloud_intrinsic_matrix(image_rgb: np.ndarray, image_depth: np.ndarray, intrinsic_matrix: np.ndarray, extrinsic_matrix: np.ndarray):
    temp_file_path = osp.join('/tmp', 'temp_img.png')

    image_rgb = np.transpose(image_rgb, [1, 0, 2])
    cv2.imwrite(temp_file_path, image_rgb)
    color = o3d.io.read_image(temp_file_path)

    # change .exr format to .png format
    depth_png = (image_depth * 1000).astype(np.uint16)[:, :]
    depth_png = np.transpose(depth_png, [1, 0])
    cv2.imwrite(temp_file_path, depth_png)
    depth = o3d.io.read_image(temp_file_path)

    os.remove(temp_file_path)

    pcd = image_open3d_to_point_cloud_intrinsic_matrix(color, depth, intrinsic_matrix, extrinsic_matrix)

    return pcd

def image_open3d_to_point_cloud_intrinsic_matrix(color: o3d.geometry.Image, depth: o3d.geometry.Image, intrinsic_matrix: np.ndarray, extrinsic_matrix: np.ndarray):
    rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
        color, depth, depth_trunc=20, convert_rgb_to_intensity=False)

    intrinsic_matrix = o3d.camera.PinholeCameraIntrinsic(int(intrinsic_matrix[0, 2] * 2),
                                                         int(intrinsic_matrix[1, 2] * 2), intrinsic_matrix[0, 0],
                                                         intrinsic_matrix[1, 1], intrinsic_matrix[0, 2],
                                                         intrinsic_matrix[1, 2])

    pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
        rgbd_image, intrinsic_matrix, project_valid_depth_only=False)

    # convter to unity space
    pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    # camera to world in unity space
    pcd.transform(extrinsic_matrix)

    return pcd

def mask_point_cloud_with_id_gray_color(pcd: o3d.geometry.PointCloud, image_mask: np.ndarray, color:int):
    pcd = o3d.geometry.PointCloud(pcd)
    image_mask = image_mask.reshape(-1)
    index = np.argwhere(image_mask == color).reshape(-1)
    d = np.array(pcd.points)[index]
    pcd.points = o3d.utility.Vector3dVector(d)
    c = np.array(pcd.colors)[index]
    pcd.colors = o3d.utility.Vector3dVector(c)
    return pcd