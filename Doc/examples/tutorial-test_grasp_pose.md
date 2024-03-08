# test_grasp_pose

## 1 Basic Features

- Display preview of Franka two-finger gripper's grasp pose

## 2 Implementation Process

### 2.1 Data Preprocessing

```python
mesh_path = "../Mesh/drink1/drink1.obj"
pose_path = "../Mesh/drink1/grasps_rfu.csv"

data = pd.read_csv(pose_path, usecols=["x", "y", "z", "qx", "qy", "qz", "qw"])
data = data.to_numpy()
positions = data[:, 0:3].reshape(-1).tolist()
quaternions = data[:, 3:7].reshape(-1).tolist()
```

- Import the required mesh and pose data, and preprocess it

### 2.2 Environment Initialization

```python
env = RFUniverseBaseEnv(ext_attr=[GraspSimAttr])
```

### 2.3 Display Preview of Franka Two-Finger Gripper Grasp Pose

```python
grasp_sim = env.InstanceObject(id=123123, name="GraspSim", attr_type=GraspSimAttr)
grasp_sim.ShowGraspPose(
    mesh=os.path.abspath(mesh_path),
    gripper="SimpleFrankaGripper",
    positions=positions,
    quaternions=quaternions,
)

env.Pend()
env.close()
```

- `InstanceObject` for instantiating a GraspSim object
- `ShowGraspPose` for displaying the grasp pose preview, where:
    - `mesh` parameter is the absolute path of the `.obj` file
    - `gripper` parameter is the name of the gripper
    - `positions` parameter is a list of coordinates for the grasp points
    - `quaternions` parameter is for representing rotation using quaternions