# test_load_urdf

## 1 Basic Functionality

- Importing URDF files
- Effects
  - Import 3 URDF files
  - Achieve movement and rotation of the first robot

## 2 Implementation Process

### 2.1 Initialize the environment and import URDF files

```python
env = RFUniverseBaseEnv()

ur5 = env.LoadURDF(id=639787, path=os.path.abspath('../URDF/UR5/ur5_robot.urdf'), native_ik=True)
ur5.SetTransform(position=[1, 0, 0])
yumi = env.LoadURDF(id=358136, path=os.path.abspath('../URDF/yumi_description/urdf/yumi.urdf'), native_ik=False)
yumi.SetTransform(position=[2, 0, 0])
kinova = env.LoadURDF(id=985135, path=os.path.abspath('../URDF/kinova_gen3/GEN3_URDF_V12.urdf'), native_ik=False)
kinova.SetTransform(position=[3, 0, 0])
```

- Import 3 URDF files in sequence and specify the initial positions of the three rigid bodies
- `native_ik` is a plugin. When set to `false`, you cannot use interfaces like `IKTargetDoMove` mentioned in [2.2](#2.2) and can only manually set the position of each joint (using `SetJointPosition`). Conversely, when set to `true`, you cannot manually set through `SetJointPosition`.

### 2.2 Setting the action of the first rigid body

```python
ur5.IKTargetDoMove(position=[0, 0.5, 0], duration=0.1, relative=True)
env.step()
ur5.WaitDo()
```

- The meaning of parameters in `IKTargetDoMove`:
  - `position`: The target position of the rigid body's end
  - `duration`: The duration from the current position to the target position
  - `relative`: When `true`, it is relative to the current position; when `false`, it moves to the absolute position in world coordinates

```python
ur5.IKTargetDoMove(position=[0, 0, -0.5], duration=0.1, relative=True)
env.step()
ur5.WaitDo()
ur5.IKTargetDoMove(position=[0, -0.2, 0.3], duration=0.1, relative=True)
ur5.IKTargetDoRotateQuaternion(quaternion=utility.UnityEularToQuaternion([0, 90, 0]), duration=30, relative=True)
env.step()
ur5.WaitDo()

while 1:
    env.step()
```

- Since the first rigid body has `native_ik` set to `true` initially, only the end position needs to be specified, and the ik algorithm inversely calculates the angles of each joint