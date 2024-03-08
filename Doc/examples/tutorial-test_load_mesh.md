# test_load_mesh

## 1 Basic Functionality

![](../image/load_mesh.gif)

- Import obj model files as rigid bodies
- Effect: Import obj model files and duplicate 100 rigid bodies, while simulating the free fall of rigid bodies

## 2 Implementation Process

### 2.1 Initialize the Environment

```python
import random
import os
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv

env = RFUniverseBaseEnv()
env.step()
```

- `env.step()` allows the environment to run for a time $\delta t$, where $\delta t$ represents the concept of a "time slice".

### 2.2 Load the Rigid Body and Set Actions

```python
mesh = env.LoadMesh(id=639787, path=os.path.abspath('../Mesh/002_master_chef_can/google_16k/textured.obj'))
mesh.SetTransform(position=[0, 1, 0], rotation=[random.random() * 360, random.random() * 360, random.random() * 360])
```

- `id=639787`: Each object in the scene needs a unique id
- `LoadMesh` loads the rigid body corresponding to the `.obj` file
- `SetTransform` sets actions, in this test file, it is the free fall from position $[0,1,0]$

### 2.3 Repeat the Simulation of Rigid Body Falling

```python
for i in range(100):
    env.step(20)
    new_mesh = mesh.Copy(new_id=mesh.id + i + 1)
    new_mesh.SetTransform(position=[0, 1, 0], rotation=[random.random() * 360, random.random() * 360, random.random() * 360])

while 1:
    env.step()
```

- Wait for 20 time slices `env.step(20)` to complete the motion of the object and stabilize it
- Duplicate the rigid body and repeat the action