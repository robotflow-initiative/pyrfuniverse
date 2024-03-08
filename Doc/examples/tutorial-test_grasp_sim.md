# test_grasp_sim

## 1 Basic Functionality

![](../image/grasp_sim.gif)

- Demonstrates the Franka two-finger gripper grasp test
- Basic idea: Randomly initialize the position of the object. If there is penetration between the object and the gripper, do not grasp; if the object's generated position is reasonable, the gripper will grasp normally. Since penetration occurs most of the time, a large number of grippers and objects are initialized in the scene at once for parallel synchronous testing.

## 2 Implementation Process

### 2.1 Data Preprocessing

```python
mesh_path = "../Mesh/drink1/drink1.obj"

points, normals = get_grasp_pose(mesh_path, 100)
points = points.reshape(-1).tolist()
normals = normals.reshape(-1).tolist()
```

- Import the required mesh and pose data and preprocess it

### 2.2 Initialize the Environment

```python
env = RFUniverseBaseEnv(assets=["GraspSim"], ext_attr=[GraspSimAttr])
```

### 2.3 Demonstrate the Franka Two-Finger Gripper Grasp Test

```python
grasp_sim = env.InstanceObject(id=123123, name="GraspSim", attr_type=GraspSimAttr)
grasp_sim.StartGraspSim(
    mesh=os.path.abspath(mesh_path),
    gripper="franka_hand",
    points=points,
    normals=normals,
    depth_range_min=-0.05,
    depth_range_max=0,
    depth_lerp_count=5,
    angle_lerp_count=5,
    parallel_count=100,
)

env.step()
while not grasp_sim.data['done']:
    env.step()
```

- Call `StartGraspSim` to start the gripper test and wait for the test to complete

```python
points = grasp_sim.data["points"]
points = np.array(points).reshape([-1, 3])
quaternions = grasp_sim.data["quaternions"]
quaternions = np.array(quaternions).reshape([-1, 4])
width = grasp_sim.data["width"]
width = np.array(width).reshape([-1, 1])

env.close()

data = np.concatenate((points, quaternions, width), axis=1)
csv = pd.DataFrame(data, columns=["x", "y", "z", "qx", "qy", "qz", "qw", "width"])

csv_path = os.path.join(os.path.dirname(mesh_path), "grasps_rfu.csv")
csv.to_csv(csv_path, index=True, header=True)

env.Pend()
env.close()
```

- Record the test results and save