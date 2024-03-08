# test_save_gripper

## 1 Basic Functionality

- Save the gripper after driving it as a model in OBJ format.

## 2 Implementation Process

### 2.1 Initialize Environment

```python
env = RFUniverseBaseEnv(assets=["allegro_hand_right"])
```

### 2.2 Set Gripper State

```python
bhand = env.InstanceObject("allegro_hand_right", attr_type=attr.ControllerAttr)
env.step(5)
moveable_joint_count = bhand.data["number_of_moveable_joints"]
print(f"moveable_joint_count:{moveable_joint_count}")
bhand.SetJointPositionDirectly([30 for _ in range(moveable_joint_count)])
env.step(5)
```

- `SetJointPositionDirectly`: Sets the position for each joint and moves it directly.

### 2.3 Save Gripper State

```python
env.ExportOBJ([bhand.id], os.path.abspath("../Mesh/gripper_mesh.obj"))
```

- `ExportOBJ`: Saves the specified object as an OBJ format file.