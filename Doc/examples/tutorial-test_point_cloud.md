# test_point_cloud

## 1 Basic Features

- Use image width, height, and FOV to obtain a depth map and convert it to a point cloud.

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(scene_file="PointCloud.json")
```

- Import the scene from the pre-configured `PointCloud.json` file.

### 2.2 Obtain the Image

```python
camera1 = env.GetAttr(698548)
camera1.GetDepthEXR(width=1920, height=1080)
camera1.GetRGB(width=1920, height=1080)
camera1.GetID(width=1920, height=1080)
env.step()
```

### 2.3 Convert to Point Cloud

```python
image_rgb = camera1.data["rgb"]
image_depth_exr = camera1.data["depth_exr"]
fov = 60
local_to_world_matrix = camera1.data["local_to_world_matrix"]
point1 = dp.image_bytes_to_point_cloud(
    image_rgb, image_depth_exr, fov, local_to_world_matrix
)
```

- `fov`: Field of View, default is 60.
- `image_bytes_to_point_cloud`: Generates a scene's point cloud using the RGB image, depth map, `fov`, and `local_to_world_matrix`.

### 2.4 Visualize the Point Cloud

```python
# unity space to open3d space and show
point1.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
point2.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([point1, point2, coordinate])
```