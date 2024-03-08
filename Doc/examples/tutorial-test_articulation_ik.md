# test_articulation_ik

## 1. Basic Functionality

- Utilizes the native IK (Inverse Kinematics) algorithm to move and rotate the robotic arm.

## 2. Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(scene_file="ArticulationIK.json")
```

- Import the scene from a pre-configured `ArticulationIK.json` file.

### 2.2 Invoke the IK Algorithm to Move the Robotic Arm

```python
for id in ids:
    current_robot = env.GetAttr(id)
    current_robot.IKTargetDoMove(position=[0, 0, -0.5], duration=0.1, relative=True)
    env.step()
    while not current_robot.data["move_done"]:
        env.step()
    current_robot.IKTargetDoMove(position=[0, -0.5, 0], duration=0.1, relative=True)
    env.step()
    while not current_robot.data["move_done"]:
        env.step()
    current_robot.IKTargetDoMove(position=[0, 0.5, 0.5], duration=0.1, relative=True)
    current_robot.IKTargetDoRotateQuaternion(
        quaternion=utility.UnityEularToQuaternion([90, 0, 0]),
        duration=30,
        relative=True,
    )
    env.step()
    while not current_robot.data["move_done"] or not current_robot.data["rotate_done"]:
        env.step()
```

- Parameters in `IKTargetDoMove`:
  
  - `position`: The target position for the movement.
  - `duration`: The duration from the current position to the target position.
  - `relative`: `true` indicates the movement is relative to the current position, `false` indicates an absolute position in world coordinates.

- Parameters in `IKTargetDoRotateQuaternion`:
  
  - `quaternion`: The quaternion representing the rotation.
  - `duration`: The duration from the current orientation to the target orientation.
  - `relative`: `true` indicates the rotation is relative to the current orientation, `false` indicates an absolute orientation in world coordinates.

This tutorial segment demonstrates how to utilize the native IK capabilities of the pyrfuniverse simulation platform for precise control over the movements and rotations of a robotic arm.