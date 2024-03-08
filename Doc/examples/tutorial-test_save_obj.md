# test_save_obj

## 1 Basic Functionality

- Save multiple objects from the scene as OBJ models

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(scene_file="SimpleYCBModel.json")
```

### 2.2 Save Objects

```python
model = []

for i in env.attrs:
    if type(env.attrs[i]) is attr.RigidbodyAttr:
        model.append(i)
```

- Record all rigidbodies in the `model` list

```python
env.ExportOBJ(model, os.path.abspath("../Mesh/scene_mesh.obj"))

env.Pend()
env.close()
```

- `ExportOBJ`: Saves all objects in the list as an OBJ format file