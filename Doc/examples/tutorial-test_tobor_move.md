# test_tobor_move

## 1 Basic Features

- Demonstrate the wheel-driven movement of tobor

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(assets=["tobor_r300_ag95_ag95"])
```

### 2.2 Initialize Tobor

```python
torbor = env.InstanceObject(name="tobor_r300_ag95_ag95", attr_type=attr.ControllerAttr)
torbor.SetTransform(position=[0, 0.05, 0])
torbor.SetImmovable(False)
env.step()
```

- `SetImmovable`: Sets the movability of the base joint

### 2.3 Drive Tobor to Move

```python
while 1:
    torbor.MoveForward(1, 0.2)
    env.step(300)
    torbor.TurnLeft(90, 30)
    env.step(300)
    torbor.MoveForward(1, 0.2)
    env.step(300)
    torbor.TurnLeft(90, 30)
    env.step(300)
    torbor.MoveForward(1, 0.2)
    env.step(300)
    torbor.TurnRight(90, 30)
    env.step(300)
    torbor.MoveBack(1, 0.2)
    env.step(300)
    torbor.TurnRight(90, 30)
    env.step(300)

```

- `MoveForward`: Move forward, the first parameter represents distance, the second parameter represents speed
- `MoveBack`: Move backward, the first parameter represents distance, the second parameter represents speed
- `TurnLeft`: Turn left, the first parameter represents angle, the second parameter represents speed
- `TurnRight`: Turn right, the first parameter represents angle, the second parameter represents speed