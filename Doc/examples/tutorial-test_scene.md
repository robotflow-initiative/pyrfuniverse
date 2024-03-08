# test_save_scene

## 1 Basic Functions

- Test the setup/save/load of scenes

## 2 Implementation Process

### 2.1 Initialize Environment

```python
env = RFUniverseBaseEnv(assets=["Collider_Box", "Rigidbody_Sphere"])
```

### 2.2 Load Objects

```python
box1 = env.InstanceObject(name="Collider_Box", attr_type=attr.ColliderAttr)
box1.SetTransform(position=[-0.5, 0.5, 0], scale=[0.1, 1, 1])
box2 = env.InstanceObject(name="Collider_Box", attr_type=attr.ColliderAttr)
box2.SetTransform(position=[0.5, 0.5, 0], scale=[0.1, 1, 1])
box3 = env.InstanceObject(name="Collider_Box", attr_type=attr.ColliderAttr)
box3.SetTransform(position=[0, 0.5, 0.5], scale=[1, 1, 0.1])
box4 = env.InstanceObject(name="Collider_Box", attr_type=attr.ColliderAttr)
box4.SetTransform(position=[0, 0.5, -0.5], scale=[1, 1, 0.1])
sphere = env.InstanceObject(name="Rigidbody_Sphere", attr_type=attr.RigidbodyAttr)
sphere.SetTransform(position=[0, 0.5, 0], scale=[0.5, 0.5, 0.5])
env.Pend()
```

- `InstanceObject`: Instantiate an object from assets, i.e., create an object in the virtual scene.
- `SetTransform`: Set the position and orientation of the object.

### 2.3 Save Scene

```python
env.SaveScene("test_scene.json")
env.ClearScene()
env.Pend()
```

- `SaveScene`: Save the scene as a json format file.

### 2.4 Load Scene

```python
env.LoadSceneAsync("test_scene.json")
env.Pend()
env.close()
```

- `LoadSceneAsync`: Asynchronously load the scene.