import os

os.environ["OPENCV_IO_ENABLE_OPENEXR"] = "1"
import cv2
import numpy as np

try:
    import open3d as o3d
except ImportError:
    raise Exception(
        "This feature requires open3d, please install with `pip install open3d`"
    )
import pyrfuniverse.utils.rfuniverse_utility as utility
import pyrfuniverse.utils.depth_processor as dp
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

nd_main_intrinsic_matrix = np.array([[600, 0, 240], [0, 600, 240], [0, 0, 1]])
nd_ir_intrinsic_matrix = np.array([[480, 0, 240], [0, 480, 240], [0, 0, 1]])

env = RFUniverseBaseEnv(scene_file="ActiveDepth.json")
active_light_sensor_1 = env.GetAttr(789789)

active_light_sensor_1.GetRGB(intrinsic_matrix=nd_main_intrinsic_matrix)
env.step()
image_byte = active_light_sensor_1.data["rgb"]
image_rgb = np.frombuffer(image_byte, dtype=np.uint8)
image_rgb = cv2.imdecode(image_rgb, cv2.IMREAD_COLOR)
image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
image_rgb = np.transpose(image_rgb, [1, 0, 2])

active_light_sensor_1.GetID(intrinsic_matrix=nd_main_intrinsic_matrix)
env.step()
image_id = active_light_sensor_1.data["id_map"]
image_id = np.frombuffer(image_id, dtype=np.uint8)
image_id = cv2.imdecode(image_id, cv2.IMREAD_COLOR)
image_id = cv2.cvtColor(image_id, cv2.COLOR_BGR2RGB)

active_light_sensor_1.GetDepthEXR(intrinsic_matrix=nd_main_intrinsic_matrix)
env.step()
image_depth_exr = active_light_sensor_1.data["depth_exr"]

active_light_sensor_1.GetActiveDepth(
    main_intrinsic_matrix_local=nd_main_intrinsic_matrix,
    ir_intrinsic_matrix_local=nd_ir_intrinsic_matrix,
)
env.step()
image_active_depth = active_light_sensor_1.data["active_depth"]
image_active_depth = np.transpose(image_active_depth, [1, 0])

local_to_world_matrix = active_light_sensor_1.data["local_to_world_matrix"]
# local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T

# point = dp.image_array_to_point_cloud(image_rgb, image_active_depth, 45, local_to_world_matrix)

color = utility.EncodeIDAsColor(568451)[0:3]
real_point_cloud1 = dp.image_bytes_to_point_cloud_intrinsic_matrix(
    image_byte, image_depth_exr, nd_main_intrinsic_matrix, local_to_world_matrix
)
real_point_cloud1 = dp.mask_point_cloud_with_id_color(
    real_point_cloud1, image_id, color
)
active_point_cloud1 = dp.image_array_to_point_cloud_intrinsic_matrix(
    image_rgb, image_active_depth, nd_main_intrinsic_matrix, local_to_world_matrix
)
active_point_cloud1 = dp.mask_point_cloud_with_id_color(
    active_point_cloud1, image_id, color
)
filtered_point_cloud1 = dp.filter_active_depth_point_cloud_with_exact_depth_point_cloud(
    active_point_cloud1, real_point_cloud1
)

##################################################

active_light_sensor_2 = env.GetAttr(123123)

active_light_sensor_2.GetRGB(intrinsic_matrix=nd_main_intrinsic_matrix)
env.step()
image_byte = active_light_sensor_2.data["rgb"]
image_rgb = np.frombuffer(image_byte, dtype=np.uint8)
image_rgb = cv2.imdecode(image_rgb, cv2.IMREAD_COLOR)
image_rgb = cv2.cvtColor(image_rgb, cv2.COLOR_BGR2RGB)
image_rgb = np.transpose(image_rgb, [1, 0, 2])

active_light_sensor_2.GetID(intrinsic_matrix=nd_main_intrinsic_matrix)
env.step()
image_id = active_light_sensor_2.data["id_map"]
image_id = np.frombuffer(image_id, dtype=np.uint8)
image_id = cv2.imdecode(image_id, cv2.IMREAD_COLOR)
image_id = cv2.cvtColor(image_id, cv2.COLOR_BGR2RGB)

active_light_sensor_2.GetDepthEXR(intrinsic_matrix=nd_main_intrinsic_matrix)
env.step()
image_depth_exr = active_light_sensor_2.data["depth_exr"]

active_light_sensor_2.GetActiveDepth(
    main_intrinsic_matrix_local=nd_main_intrinsic_matrix,
    ir_intrinsic_matrix_local=nd_ir_intrinsic_matrix,
)
env.step()
image_active_depth = active_light_sensor_2.data["active_depth"]
image_active_depth = np.transpose(image_active_depth, [1, 0])

local_to_world_matrix = active_light_sensor_2.data["local_to_world_matrix"]

env.close()

# point = dp.image_array_to_point_cloud(image_rgb, image_active_depth, 45, local_to_world_matrix)
real_point_cloud2 = dp.image_bytes_to_point_cloud_intrinsic_matrix(
    image_byte, image_depth_exr, nd_main_intrinsic_matrix, local_to_world_matrix
)
real_point_cloud2 = dp.mask_point_cloud_with_id_color(
    real_point_cloud2, image_id, color
)
active_point_cloud2 = dp.image_array_to_point_cloud_intrinsic_matrix(
    image_rgb, image_active_depth, nd_main_intrinsic_matrix, local_to_world_matrix
)
active_point_cloud2 = dp.mask_point_cloud_with_id_color(
    active_point_cloud2, image_id, color
)
filtered_point_cloud2 = dp.filter_active_depth_point_cloud_with_exact_depth_point_cloud(
    active_point_cloud2, real_point_cloud2
)

# unity space to open3d space and show
real_point_cloud1.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
real_point_cloud2.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
active_point_cloud1.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
active_point_cloud2.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
filtered_point_cloud1.transform(
    [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
)
filtered_point_cloud2.transform(
    [[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]]
)
coorninate = o3d.geometry.TriangleMesh.create_coordinate_frame()

o3d.visualization.draw_geometries([real_point_cloud1, real_point_cloud2, coorninate])
o3d.visualization.draw_geometries(
    [active_point_cloud1, active_point_cloud2, coorninate]
)
o3d.visualization.draw_geometries(
    [filtered_point_cloud1, filtered_point_cloud2, coorninate]
)
