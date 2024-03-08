# test_humanbody_ik

## 1. Basic Features

- Demonstrate the human body IK (Inverse Kinematics) interface.

## 2. Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(scene_file="HumanBodyIK.json", ext_attr=[HumanbodyAttr])
env.step()
```

### 2.2 Demonstrate the Human Body IK Interface

```python
human = env.GetAttr(168242)
for index in range(5):
    human.HumanIKTargetDoMove(
        index=index, position=[0, 0, 0.5], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()
    human.HumanIKTargetDoMove(
        index=index, position=[0, 0, 0.5], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()
    human.HumanIKTargetDoMove(
        index=index, position=[0, 0, -0.5], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()
    human.HumanIKTargetDoMove(
        index=index, position=[0, -0.5, 0], duration=1, speed_based=False, relative=True
    )
    human.WaitDo()

env.Pend()
env.close()
```

- `HumanIKTargetDoMove` can control the movement of the human body's limbs to a specified position.
    - `index`: The target of the movement.
        - 0: Left hand
        - 1: Right hand
        - 2: Left foot
        - 3: Right foot
        - 4: Head
    - `position`: The endpoint position of the movement.
    - `duration`: If the `speed_based` parameter is set to True, this parameter represents the time required for the movement; if the `speed_based` parameter is set to False, it represents the speed of the movement.
    - `speed_based`: As described above, defaults to True.
    - `relative`: If True, it is a relative coordinate; otherwise, it is a world absolute coordinate.