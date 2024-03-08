# test_digit

## 1 Basic Functions

![](../image/digit.png)

- Display interactive Digit tactile sensor simulation

## 2 Implementation Process

### 2.1 Initialize Environment

```python
env = RFUniverseBaseEnv(ext_attr=[DigitAttr])
```

### 2.2 Display Interactive Digit Tactile Sensor Simulation

```python
digit = env.InstanceObject(name="Digit", attr_type=DigitAttr)
digit.SetTransform(position=[0, 0.015, 0])
target = env.InstanceObject(name="DigitTarget")
target.SetTransform(position=[0, 0.05, 0.015])
env.SetViewTransform(position=[-0.1, 0.033, 0.014], rotation=[0, 90, 0])
env.Pend()
env.close()
```

- `InstanceObject` instantiates the Digit sensor
- `SetTransform` sets the sensor and object at appropriate positions and orientations
- `SetViewTransform` sets the viewpoint at appropriate positions and orientations