# test_pick_and_place

## 1 Basic Functionality

![](../image/pick_place.gif)

- Demonstrates the basic interface of the robotic arm and native IK-driven grabbing

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(assets=["franka_panda"])
env.SetTimeStep(0.005)
robot = env.InstanceObject(
    name="franka_panda", id=123456, attr_type=attr.ControllerAttr
)
robot.SetIKTargetOffset(position=[0, 0.105, 0])
env.step()
gripper = env.GetAttr(1234560)
gripper.GripperOpen()
robot.IKTargetDoMove(position=[0, 0.5, 0.5], duration=0, speed_based=False)
robot.IKTargetDoRotate(rotation=[0, 45, 180], duration=0, speed_based=False)
robot.WaitDo()
```

- `SetTimeStep`: Sets the duration of a time step in the corresponding Unity environment
- `SetIKTargetOffset`: Sets the offset for the movement of the robotic arm; here, the offset is upwards, so later when moving the robotic arm, you just need to set the target to the position of the object, and the robotic arm will automatically move to the position above the object to grab it

### 2.2 Loop to Grab Objects

```python
    box1 = env.InstanceObject(
        name="Rigidbody_Box", id=111111, attr_type=attr.RigidbodyAttr
    )
    box1.SetTransform(
        position=[random.uniform(-0.5, -0.3), 0.03, random.uniform(0.3, 0.5)],
        scale=[0.06, 0.06, 0.06],
    )
    box2 = env.InstanceObject(
        name="Rigidbody_Box", id=222222, attr_type=attr.RigidbodyAttr
    )
    box2.SetTransform(
        position=[random.uniform(0.3, 0.5), 0.03, random.uniform(0.3, 0.5)],
        scale=[0.06, 0.06, 0.06],
    )
    env.step(100)

    position1 = box1.data["position"]
    position2 = box2.data["position"]
```

- `InstanceObject`: Initializes two boxes in the scene for grabbing, note that the id parameter must be globally unique
- `SetTransform`: Sets the position, orientation, and size of the boxes

```python
    robot.IKTargetDoMove(
        position=[position1[0], position1[1] + 0.5, position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
    robot.IKTargetDoMove(
        position=[position1[0], position1[1], position1[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
```

- In `IKTargetDoMove`, the parameters mean:
  - `position`: Represents the quaternion for rotation
  - `duration`: The duration from the current position to the target position
  - `relative`: `true` means relative to the current position, `false` means the absolute position in world coordinates
- `WaitDo`: Since the movement of the robot in the scene takes time, this function is called to wait for the robot to complete its movement before executing subsequent code

```python
    gripper.GripperClose()
    env.step(50)
    robot.IKTargetDoMove(
        position=[0, 0.5, 0], duration=2, speed_based=False, relative=True
    )
    robot.WaitDo()
```

- `GripperClose`: Closes the gripper

```python
    robot.IKTargetDoMove(
        position=[position2[0], position2[1] + 0.5, position2[2]],
        duration=4,
        speed_based=False,
    )
    robot.WaitDo()
    robot.IKTargetDoMove(
        position=[position2[0], position2[1] + 0.06, position2[2]],
        duration=2,
        speed_based=False,
    )
    robot.WaitDo()
```

```python
    gripper.GripperOpen()
    env.step(50)
    robot.IKTargetDoMove(
        position=[0, 0.5, 0], duration=2, speed_based=False, relative=True
    )
    robot.WaitDo()
    robot.IKTargetDoMove(position=[0, 0.5, 0.5], duration=2, speed_based=False)
    robot.WaitDo()
```

- `GripperOpen`: Opens the gripper

```python
    box1.Destroy()
    box2.Destroy()
    env.step()
```

- `Destroy`: Destroys the object