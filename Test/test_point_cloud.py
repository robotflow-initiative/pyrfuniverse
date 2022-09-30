from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.utils.depth_processor as dp
import numpy as np

env = RFUniverseBaseEnv(
    # executable_file='/home/yanbing/Project/rfuniverse/rfuniverse/Build/usr/local/RFUniverse/RFUniverse.x86_64',
    scene_file='PointCloud.json'
)
env.instance_channel.set_action(
    'GetDepthEXR',
    id=698548,
    width=1920,
    height=1080,
)
env.instance_channel.set_action(
    'GetRGB',
    id=698548,
    width=1920,
    height=1080
)
env._step()
image_rgb = env.instance_channel.data[698548]['rgb']
image_depth_exr = env.instance_channel.data[698548]['depth_exr']
fov = env.instance_channel.data[698548]['fov']
local_to_world_matrix = env.instance_channel.data[698548]['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T
dp.convert_bytes2pc(image_rgb, image_depth_exr, fov, local_to_world_matrix)

while 1:
    env._step()
