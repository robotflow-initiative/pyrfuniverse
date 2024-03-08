# test_pointcloud_with_intrinsic_matrix

## 1 Basic Functionality

- Use camera intrinsic parameters to obtain a depth map and convert it into a point cloud.

## 2 Implementation Process

### 2.1 Environment Initialization

```python
env = RFUniverseBaseEnv(scene_file="PointCloud.json")
```

- Import the scene from the pre-configured `PointCloud.json` file.

### 2.2 Obtain Camera Intrinsic Parameters and Images

```python
nd_intrinsic_matrix = np.array([[960, 0, 960], [0, 960, 540], [0, 0, 1]])
camera1 = env.GetAttr(698548)
camera1.GetDepthEXR(intrinsic_matrix=nd_intrinsic_matrix)
camera1.GetRGB(intrinsic_matrix=nd_intrinsic_matrix)
camera1.GetID(intrinsic_matrix=nd_intrinsic_matrix)
env.step()
```

### 2.3 Convert to Point Cloud

```python
image_rgb = camera1.data["rgb"]
image_depth_exr = camera1.data["depth_exr"]
local_to_world_matrix = camera1.data["local_to_world_matrix"]
point1 = dp.image_bytes_to_point_cloud_intrinsic_matrix(
    image_rgb, image_depth_exr, nd_intrinsic_matrix, local_to_world_matrix
)
```

- `image_bytes_to_point_cloud_intrinsic_matrix`: Generates the scene's point cloud through the RGB image, depth map, intrinsic matrix, and the transformation matrix from local to world coordinates.

### 2.4 Visualize Point Cloud

```python
# unity space to open3d space and show
point1.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
point2.transform([[-1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
coordinate = o3d.geometry.TriangleMesh.create_coordinate_frame()
o3d.visualization.draw_geometries([point1, point2, coordinate])
```