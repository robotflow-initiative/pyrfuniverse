# test_ply_render

## 1 Basic Features

- Demonstrates the point cloud display feature

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv()
env.SetViewBackGround([0.0, 0.0, 0.0])
```

- `SetViewBackGround` sets the background to black

### 2.2 Display Point Cloud

```python
point_cloud = env.InstanceObject(
    name="PointCloud", id=123456, attr_type=attr.PointCloudAttr
)
point_cloud.ShowPointCloud(ply_path=os.path.abspath("../Mesh/000000_000673513312.ply"))
point_cloud.SetTransform(rotation=[-90, 0, 0])
point_cloud.SetRadius(radius=0.001)

env.Pend()
env.close()
```

- `ShowPointCloud` imports a point cloud file and displays it
- `SetRadius` sets the radius size for each point displayed in the point cloud