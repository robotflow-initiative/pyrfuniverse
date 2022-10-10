import cv2
import numpy as np
import open3d as o3d

import pyrfuniverse.utils.depth_processor as dp
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv(
    # executable_file='/home/yanbing/Project/rfuniverse/rfuniverse/Build/usr/local/RFUniverse/RFUniverse.x86_64',
    scene_file='ActiveDepth.json'
)

main_intrinsic_matrix = [1380, 0, 0, 0, 1380, 0, 960, 540, 1]

env.instance_channel.set_action(
    'GetRGB',
    id=789789,
    intrinsic_matrix=main_intrinsic_matrix
)
env._step()
image_rgb = env.instance_channel.data[789789]['rgb']
image_rgb = np.frombuffer(image_rgb, dtype=np.uint8)
image_rgb = cv2.imdecode(image_rgb, cv2.IMREAD_COLOR)
image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
image_rgb = np.transpose(image_rgb, [1, 0, 2])

env.instance_channel.set_action(
    'GetActiveDepth',
    id=789789,
    main_intrinsic_matrix=main_intrinsic_matrix
)
env._step()
image_active_depth = env.instance_channel.data[789789]['active_depth']
image_active_depth = np.transpose(image_active_depth, [1, 0])

local_to_world_matrix = env.instance_channel.data[789789]['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T

nd_main_intrinsic_matrix = np.reshape(main_intrinsic_matrix, [3, 3]).T
# point = dp.image_array_to_point_cloud(image_rgb, image_active_depth, 45, local_to_world_matrix)

point1 = dp.image_array_to_point_cloud_intrinsic_matrix(image_rgb, image_active_depth-0.03, nd_main_intrinsic_matrix, local_to_world_matrix)


env.instance_channel.set_action(
    'GetRGB',
    id=123123,
    intrinsic_matrix=main_intrinsic_matrix
)
env._step()
image_rgb = env.instance_channel.data[123123]['rgb']
image_rgb = np.frombuffer(image_rgb, dtype=np.uint8)
image_rgb = cv2.imdecode(image_rgb, cv2.IMREAD_COLOR)
image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
image_rgb = np.transpose(image_rgb, [1, 0, 2])

env.instance_channel.set_action(
    'GetActiveDepth',
    id=123123,
    main_intrinsic_matrix=main_intrinsic_matrix
)
env._step()
image_active_depth = env.instance_channel.data[123123]['active_depth']
image_active_depth = np.transpose(image_active_depth, [1, 0])

local_to_world_matrix = env.instance_channel.data[123123]['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T

nd_main_intrinsic_matrix = np.reshape(main_intrinsic_matrix, [3, 3]).T
# point = dp.image_array_to_point_cloud(image_rgb, image_active_depth, 45, local_to_world_matrix)
point2 = dp.image_array_to_point_cloud_intrinsic_matrix(image_rgb, image_active_depth-0.03, nd_main_intrinsic_matrix, local_to_world_matrix)

# unity space to open3d space and show
point1.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
point2.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
coorninate = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([point1, point2, coorninate])

while 1:
    env._step()
