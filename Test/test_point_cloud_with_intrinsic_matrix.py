import os
os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.utils.depth_processor as dp
import numpy as np
try:
    import open3d as o3d
except ImportError:
    print('This feature requires open3d, please install with `pip install open3d`')
    raise

env = RFUniverseBaseEnv(scene_file='PointCloud.json')

intrinsic_matrix = [960, 0, 0, 0, 960, 0, 960, 540, 1]
nd_intrinsic_matrix = np.reshape(intrinsic_matrix, [3, 3]).T

camera1 = env.GetAttr(698548)
camera1.GetDepthEXR(intrinsic_matrix=intrinsic_matrix)
camera1.GetRGB(intrinsic_matrix=intrinsic_matrix)
camera1.GetID(intrinsic_matrix=intrinsic_matrix)
env.step()

image_rgb = camera1.data['rgb']
image_depth_exr = camera1.data['depth_exr']
local_to_world_matrix = camera1.data['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T
point1 = dp.image_bytes_to_point_cloud_intrinsic_matrix(image_rgb, image_depth_exr, nd_intrinsic_matrix, local_to_world_matrix)

camera2 = env.GetAttr(698550)
camera2.GetDepthEXR(intrinsic_matrix=intrinsic_matrix)
camera2.GetRGB(intrinsic_matrix=intrinsic_matrix)
camera2.GetID(intrinsic_matrix=intrinsic_matrix)
env.step()

image_rgb = camera2.data['rgb']
image_depth_exr = camera2.data['depth_exr']
local_to_world_matrix = camera2.data['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T
point2 = dp.image_bytes_to_point_cloud_intrinsic_matrix(image_rgb, image_depth_exr, nd_intrinsic_matrix, local_to_world_matrix)
env.close()

# unity space to open3d space and show
point1.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
point2.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
coorninate = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([point1, point2, coorninate])

