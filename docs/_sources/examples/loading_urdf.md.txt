# Loading URDF File Examples

## Introduction

This example will guide you through [test_load_urdf](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_load_urdf.py) that loads various URDF (Unified Robot Description Format) models into RFUniverse and controls the movement of a selected robot.

## Understanding the Code 

The code consists of several sections. Below we will discuss each part of the code line by line.

1. Importing Libraries: 

```python
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.utils.rfuniverse_utility as utility
```

2. Creating a Simulation Environment Instance: 

This line of code creates an instance of the RFUniverse simulation environment.

```python
env = RFUniverseBaseEnv()
```

3. Loading URDF Files: 

The next code loads various URDF models, including UR5, YUMI, and Kinova Gen3 into the RFUniverse simulation environment. For more robots we support, please refer to [rfuniverse_robot_list](https://github.com/mvig-robotflow/rfuniverse/tree/main/Assets/Assets/Prefab/Controller).

```python
ur5 = env.LoadURDF(path=os.path.abspath('../URDF/UR5/ur5_robot.urdf'), native_ik=True)
ur5.SetTransform(position=[1, 0, 0])

yumi = env.LoadURDF(path=os.path.abspath('../URDF/yumi_description/urdf/yumi.urdf'), native_ik=False)
yumi.SetTransform(position=[2, 0, 0])

kinova = env.LoadURDF(path=os.path.abspath('../URDF/kinova_gen3/GEN3_URDF_V12.urdf'), native_ik=False)
kinova.SetTransform(position=[3, 0, 0])
```

4. Control Robot Movement: 

The next code section moves and rotates the UR5 robot manipulator by setting its inverse kinematics (IK) target, performing the move, and waiting for the move to complete. The "duration" parameter specifies the time taken for the movement, and the "relative" parameter specifies whether the movement is relative to the previous position or not.

```python
ur5.IKTargetDoMove(position=[0, 0.5, 0], duration=0.1, relative=True)
ur5.WaitDo()

ur5.IKTargetDoMove(position=[0, 0, -0.5], duration=0.1, relative=True)
ur5.WaitDo()

ur5.IKTargetDoMove(position=[0, -0.2, 0.3], duration=0.1, relative=True)
ur5.IKTargetDoRotateQuaternion(quaternion=utility.UnityEularToQuaternion([0, 90, 0]), duration=30, relative=True)
ur5.WaitDo()
```

5. Running the Simulation: 

The final section of code runs the simulation continuously.

```python
while 1:
    env.step()
```

By modifying the path parameter in the "LoadURDF" method, you can load any URDF model into the RFUniverse simulation environment.