# test_light

## 1 Basic Features

- Demonstrate lighting functionality

## 2 Implementation Process

### 2.1 Initialize Environment

```python
env = RFUniverseBaseEnv(scene_file="LightScene.json")
```

### 2.2 Demonstrate Lighting Functionality

```python
light = env.GetAttr(885275)
env.SetShadowDistance(50)
while 1:
    env.step(50)
    light.SetColor(color=[1.0, 0.0, 0.0])
    env.step(50)
    light.SetRange(30.0)
    env.step(50)
    light.SetType(LightType.Directional)
    env.step(50)
    light.SetIntensity(5.0)
    env.step(50)
    light.SetType(LightType.Spot)
    env.step(50)
    light.SetSpotAngle(60.0)
    env.step(50)
    light.SetType(LightType.Point)
    env.step(50)
    light.SetRange(10.0)
    light.SetIntensity(1.0)
    light.SetSpotAngle(30.0)
```

- `SetShadowDistance`: Set the distance of the shadow, in meters
- `SetColor`: Set the color of the light
- `SetRange`: Set the range of the light (only effective when the light type is Spot or Point)
- `SetType`: Set the type of the light, similar to Unity, includes five types: Spot, Directional, Point, Rectangle, Disc
- `SetSpotAngle`: Set the angle of the light (only effective when the light type is Spot)
- `SetIntensity`: Set the intensity of the light