# Embodied Robot Controlling Example

## Introduction

This example will guide you through [test_tobor_move.py](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_tobor_move.py) that loads and controls an embodied robot named Tobor in RFUniverse simulation environment.

## Understanding the Code 

The code consists of several sections. Below we will discuss each part of the code line by line.

1. Importing Libraries: 

```python
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
```

2. Creating a Simulation Environment Instance and Loading Tobor: 

The next code creates an instance of the RFUniverse simulation environment and loads the Tobor robot with two AG95 grippers. Tobor also support other grippers, such as Robotiq85.

```python
env = RFUniverseBaseEnv(assets=['tobor_r300_ag95_ag95'])
torbor = env.InstanceObject(name='tobor_r300_ag95_ag95', attr_type=attr.ControllerAttr)
torbor.SetTransform(position=[0, 0.05, 0])
torbor.SetImmovable(False)
env.step()
```

3. Controlling the Robot: 

The next code section controls the movements of the Tobor robot by calling its various methods, such as `MoveForward`, `TurnLeft`, `MoveBack`, and `TurnRight`. The parameters passed to the methods specify the duration and distance of the movement.

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

4. Running the Simulation: 

The final section of code runs the simulation continuously.

```python
while 1:
    env.step()
```
