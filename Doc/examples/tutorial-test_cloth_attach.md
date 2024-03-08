# test_cloth_attach

## 1 Basic Functionality

- Demonstrate the behavior of cloth in a virtual environment

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv()
env.DebugObjectPose()
env.EnabledGroundObiCollider(True)
```

- `DebugObjectPose` enables the function to display the base coordinates of objects.
- `EnabledGroundObiCollider` activates the Obi collider plugin in Unity.

### 2.2 Import Cloth Object

```python
mesh = env.LoadCloth(
    path=os.path.abspath("../Mesh/Tshirt.obj")
)
mesh.SetTransform(position=[0, 1, 0])
env.step(200)
mesh.GetParticles()
env.step()
```

- By meshing the cloth, a cloth mesh composed of cloth particles is constructed to handle collisions between cloth and general objects.

### 2.3 Simulate Grabbing and Moving the Cloth

```python
position1 = mesh.data['particles'][500]
position2 = mesh.data['particles'][200]
point1 = env.InstanceObject("Empty")
point1.SetTransform(position=position1)
mesh.AddAttach(point1.id)
point2 = env.InstanceObject("Empty")
point2.SetTransform(position=position2)
mesh.AddAttach(point2.id)
env.step()

point1.DoMove([-0.25, 1, 0], 2, speed_based=False)
point2.DoMove([0.25, 1, 0], 2, speed_based=False)
point2.WaitDo()

while True:
    point1.DoMove([-0.25, 1, -0.5], 1)
    point2.DoMove([0.25, 1, -0.5], 1)
    point2.WaitDo()

    point1.DoMove([-0.25, 1, 0.5], 1)
    point2.DoMove([0.25, 1, 0.5], 1)
    point2.WaitDo()

env.Pend()
```