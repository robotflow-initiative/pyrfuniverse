import math
import os
import os.path as osp
import random

import numpy as np
import open3d as o3d
import cv2
import tempfile

def image_bytes_to_point_cloud(rgb_bytes: bytes, depth_bytes: bytes, fov: float, extrinsic_matrix: np.ndarray):
    """
    使用rgb与深度图bytes以及相机fov生成世界空间点云

    Args:
        rgb_bytes: rgb图bytes
        depth_bytes: 深度图bytes
        fov: 摄像机fov
        extrinsic_matrix: 相机外参矩阵
    """
    image_rgb = np.frombuffer(rgb_bytes, dtype=np.uint8)
    image_rgb = cv2.imdecode(image_rgb, cv2.IMREAD_COLOR)
    image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
    image_rgb = np.transpose(image_rgb, [1, 0, 2])

    temp_file_path = osp.join(tempfile.gettempdir(), 'temp_img.exr')
    with open(temp_file_path, 'wb') as f:
        f.write(depth_bytes)
    image_depth = cv2.imread(temp_file_path, cv2.IMREAD_UNCHANGED)
    os.remove(temp_file_path)

    image_depth = np.transpose(image_depth, [1, 0])

    pcd = image_array_to_point_cloud(image_rgb, image_depth, fov, extrinsic_matrix)
    return pcd


def image_array_to_point_cloud(image_rgb: np.ndarray, image_depth: np.ndarray, fov: float, extrinsic_matrix: np.ndarray):
    """
    使用rgb与深度图以及相机fov生成世界空间点云

    Args:
        image_rgb: rgb图像素颜色ndarray,shape为(H,W,3)
        image_depth: 深度图像素颜色ndarray,shape为(H,W,3)
        fov: 摄像机fov
        extrinsic_matrix: 相机外参矩阵
    """
    points = depth_to_point_cloud(image_depth, fov=fov, organized=False)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)

    colors = image_rgb.reshape(-1, 3)
    pcd.colors = o3d.utility.Vector3dVector(colors.astype(np.float32) / 255.0)

    pcd.transform(extrinsic_matrix)

    return pcd

def depth_to_point_cloud(depth: np.ndarray, fov: float, organized=False):
    """
    使用深度图以及相机fov生成点云

    Args:
        depth: 深度图像素颜色ndarray,shape为(H,W,3)
        fov: 摄像机fov
        organized: 是否保持点云有序,若为True,则返回的点云shape为(H,W,3),否则为(H*W,3)
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
    """
    使用rgb与深度图bytes以及相机内参矩阵生成世界空间点云

    Args:
        rgb_bytes: rgb图bytes
        depth_bytes: 深度图bytes
        intrinsic_matrix: 相机内参矩阵
        extrinsic_matrix: 相机外参矩阵
    """
    temp_file_path = osp.join(tempfile.gettempdir(), f'temp_img_{int(random.uniform(10000000,99999999))}.png')
    with open(temp_file_path, 'wb') as f:
        f.write(rgb_bytes)
    color = o3d.io.read_image(temp_file_path)
    os.remove(temp_file_path)

    temp_file_path = osp.join(tempfile.gettempdir(), f'temp_img_{int(random.uniform(10000000,99999999))}.exr')
    with open(temp_file_path, 'wb') as f:
        f.write(depth_bytes)
    # change .exr format to .png format
    depth_exr = cv2.imread(temp_file_path, cv2.IMREAD_UNCHANGED)
    os.remove(temp_file_path)
    # mask = np.asarray(o3d.io.read_image(osp.join(mask_path, video_name, view_name, base_name + '.png')))[:, :, 0]
    # foregound_mask = mask == 11
    depth_png = (depth_exr * 1000).astype(np.uint16)[:, :]
    # foreground_depth_png[~foregound_mask] = 0  # filter the background, only need foreground
    temp_file_path = osp.join(tempfile.gettempdir(), f'temp_img_{int(random.uniform(10000000,99999999))}.png')
    cv2.imwrite(temp_file_path, depth_png)
    depth = o3d.io.read_image(temp_file_path)
    os.remove(temp_file_path)

    pcd = image_open3d_to_point_cloud_intrinsic_matrix(color, depth, intrinsic_matrix, extrinsic_matrix)

    return pcd


def image_array_to_point_cloud_intrinsic_matrix(image_rgb: np.ndarray, image_depth: np.ndarray, intrinsic_matrix: np.ndarray, extrinsic_matrix: np.ndarray):
    """
    使用rgb与深度图以及相机内参矩阵生成世界空间点云

    Args:
        image_rgb: rgb图像素颜色ndarray,shape为(H,W,3)
        image_depth: 深度图像素颜色ndarray,shape为(H,W,3)
        intrinsic_matrix: 相机内参矩阵
        extrinsic_matrix: 相机外参矩阵
    """
    temp_file_path = osp.join(tempfile.gettempdir(), f'temp_img_{int(random.uniform(10000000,99999999))}.png')

    image_rgb = np.transpose(image_rgb, [1, 0, 2])
    image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
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
    """
    使用open3d格式的rgb与深度图以及相机内参矩阵生成世界空间点云

    Args:
        color: open3d格式的rgb图
        depth: open3d格式的深度图
        intrinsic_matrix: 相机内参矩阵
        extrinsic_matrix: 相机外参矩阵
    """
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

def mask_point_cloud_with_id_color(pcd: o3d.geometry.PointCloud, image_mask: np.ndarray, color:list):
    """
    使用mask/id图对点云进行过滤

    Args:
        pcd: open3d点云
        image_mask: mask/id图,shape为(H,W,3)
        color: 需要筛选出的像素颜色
    """
    image_mask = image_mask.reshape(-1, 3)
    index = np.argwhere(image_mask == color)[:, 0]
    index = index[::3]
    d = np.array(pcd.points)[index]
    pcd.points = o3d.utility.Vector3dVector(d)
    c = np.array(pcd.colors)[index]
    pcd.colors = o3d.utility.Vector3dVector(c)
    return pcd

def mask_point_cloud_with_id_gray_color(pcd: o3d.geometry.PointCloud, image_mask: np.ndarray, color:int):
    """
    使用单通道mask/id图对点云进行过滤

    Args:
        pcd: open3d点云
        image_mask: 单通道mask/id图,shape为(H,W)
        color: 需要筛选出的像素灰度
    """
    image_mask = image_mask.reshape(-1)
    index = np.argwhere(image_mask == color).reshape(-1)
    id_pcd = o3d.geometry.PointCloud()
    d = np.array(pcd.points)[index]
    id_pcd.points = o3d.utility.Vector3dVector(d)
    c = np.array(pcd.colors)[index]
    id_pcd.colors = o3d.utility.Vector3dVector(c)
    return id_pcd

def filter_active_depth_point_cloud_with_exact_depth_point_cloud(active_pcd: o3d.geometry.PointCloud, exact_pcd: o3d.geometry.PointCloud, max_distance:float = 0.05):
    """
    使用精确的点云过滤红外点云,外出距离过远的点

    Args:
        active_pcd: 红外点云
        exact_pcd: 精确点云
        max_distance: 允许的最大距离
    """
    active_point = np.array(active_pcd.points)
    exact_point = np.array(exact_pcd.points)
    # m = exact_point.shape[0]
    # n = active_point.shape[0]
    # duplicated_active_point = np.repeat(active_point, m).reshape((n, m, 3))
    # distance = np.linalg.norm(duplicated_active_point - exact_point, axis=-1)
    distance = np.linalg.norm(active_point - exact_point, axis=-1)
    # min_distance = np.min(distance, axis=1)
    index = np.argwhere(distance < max_distance).reshape(-1)
    filter_pcd = o3d.geometry.PointCloud()
    d = np.array(active_pcd.points)[index]
    filter_pcd.points = o3d.utility.Vector3dVector(d)
    c = np.array(active_pcd.colors)[index]
    filter_pcd.colors = o3d.utility.Vector3dVector(c)
    return filter_pcd
