# test_urdf_parameter

## 1. Basic Features

![](../image/urdf_parameter.png)

- Display the joint target position setting panel

## 2. Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv()
```

### 2.2 Initialize the Robotic Arm

```python
robot = env.LoadURDF(path=os.path.abspath("../URDF/Franka/panda.urdf"), axis="z")
robot.SetTransform(position=[0, 0, 0])
robot.EnabledNativeIK(False)
```

- `LoadURDF`: Imports a model from a URDF file.
- `EnabledNativeIK`: `native_ik` is a plugin. When set to `false`, interfaces like `IKTargetDoMove` cannot be used, and each joint's position must be set manually (using `setJointPosition`). Conversely, when set to `true`, `setJointPosition` cannot be used to manually set positions.

### 2.3 Display the Joint Target Position Setting Panel

```python
env.ShowArticulationParameter(robot.id)
```