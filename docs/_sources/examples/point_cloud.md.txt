# Point Cloud from Images Examples

## Introduction

This example will guide you through [test_point_cloud.py](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_point_cloud.py) that captures images from cameras in RFUniverse and converts them into point clouds using `OpenCV` and `Open3D` libraries. The code further transforms the point clouds into an appropriate coordinate frame and visualizes them.

## Understanding the Code 

The consists of several sections. Below we will discuss each part of the code line by line.

1. Importing Libraries: 

The first line of code enable OpenEXR files in OpenCV. The last few lines import the Open3D library and raise an ImportError if it is not installed.

```python
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
```

(load_json)=
2. Creating a Simulation Environment Instance and Capturing Images: 

The next code creates an instance of the RFUniverse simulation environment by loading a predefined JSON file describing the scene and captures images from cameras in the environment.

```python
env = RFUniverseBaseEnv(scene_file='PointCloud.json')

camera1 = env.GetAttr(698548)
camera1.GetDepthEXR(width=1920, height=1080)
camera1.GetRGB(width=1920, height=1080)
camera1.GetID(width=1920, height=1080)
```

3. Transform the RGBD image into point cloud:

The next code set gets rdb image and depth image from camera, as well as corresponding transformation matrix to calculate the point cloud in the camera's coordinate.

```python
env.step()

image_rgb = camera1.data['rgb']
image_depth_exr = camera1.data['depth_exr']
fov = camera1.data['fov']
local_to_world_matrix = camera1.data['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T
point1 = dp.image_bytes_to_point_cloud(image_rgb, image_depth_exr, fov, local_to_world_matrix)
```

4. Calculate the point cloud from the second camera.

The next code set is the same process as the above. For the second camera in this scene, calculate the point cloud in its coordinate.

```python
camera2 = env.GetAttr(698550)
camera2.GetDepthEXR(width=1920, height=1080)
camera2.GetRGB(width=1920, height=1080)
camera2.GetID(width=1920, height=1080)

env.step()

image_rgb = camera2.data['rgb']
image_depth_exr = camera2.data['depth_exr']
fov = camera2.data['fov']
local_to_world_matrix = camera2.data['local_to_world_matrix']
local_to_world_matrix = np.reshape(local_to_world_matrix, [4, 4]).T
point2 = dp.image_bytes_to_point_cloud(image_rgb, image_depth_exr, fov, local_to_world_matrix)

env.close()
```

5. Transforming the points from Unity Coordinate to Open3d Coordinate: 

The next few lines of code transform the points. The "transform" method applies the necessary transformation to bring the point clouds into a suitable coordinate frame.

```python
point1.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
point2.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
```

6. Visualizing the Point Clouds: 

The final lines of code visualize the point clouds using Open3D's visualization module.

```python
coorninate = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([point1, point2, coorninate])
```
