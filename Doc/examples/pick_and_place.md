# Pick and Place Example

## Introduction 

This example will guide you through [test_pick_and_plave.py](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_pick_and_place.py) that is used to simulate a robot arm executing pick and place task in RFUniverse.

## Understanding the Code 

The code consists of several sections. Below we will discuss each part of the code line by line.

1. Importing Libraries:

```python
import random
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
```

2. Creating a Simulation Environment Instance: 

This line creates an instance of the RFUniverse simulation environment.

```python
env = RFUniverseBaseEnv()
```

3. Creating a Robot and updating its position 

The next five lines of code create franka robot instance with id 123456 and set its inverse kinematics target for the pick and place task. 

```python
robot = env.InstanceObject(name='franka_panda', id=123456, attr_type=attr.ControllerAttr)
robot.SetIKTargetOffset(position=[0, 0.105, 0])
env.step(200)
```

4. Opening Gripper, rotating and moving the robot arm to pick the object 

The next set of code executes the pick and place task. It first opens the gripper and then moves the robot arm to pick the object. 

```python
gripper = env.GetAttr(1234560)
gripper.GripperOpen()
robot.IKTargetDoMove(position=[0, 0.7, 0.5], duration=0, speed_based=False)
robot.IKTargetDoRotate(rotation=[0, 45, 180], duration=0, speed_based=False)
robot.WaitDo()
```

5. Object Creation, Destroying and Robot Arm Movement 

In this code block, two objects are created and placed randomly in the simulation environment. Then, the robot arm is moved to pick and place these objects successively. 

```python
while 1:
    box1 = env.InstanceObject(name='Rigidbody_Box', id=111111, attr_type=attr.RigidbodyAttr)
    box1.SetTransform(position=[random.uniform(-0.5, -0.3), 0.03, random.uniform(0.3, 0.5)], scale=[0.06, 0.06, 0.06])
    
    box2 = env.InstanceObject(name='Rigidbody_Box', id=222222, attr_type=attr.RigidbodyAttr)
    box2.SetTransform(position=[random.uniform(0.3, 0.5), 0.03, random.uniform(0.3, 0.5)], scale=[0.06, 0.06, 0.06])
    
    # For box1
    position1 = box1.data['position']
    robot.IKTargetDoMove(position=[position1[0], position1[1] + 0.5, position1[2]], duration=2, speed_based=False)
    robot.WaitDo()
    robot.IKTargetDoMove(position=[position1[0], position1[1], position1[2]], duration=2, speed_based=False)
    robot.WaitDo()
    gripper.GripperClose()
    env.step(50)
    robot.IKTargetDoMove(position=[0, 0.5, 0], duration=2, speed_based=False, relative=True)
    robot.WaitDo()
    
    # For box2
    position2 = box2.data['position']
    robot.IKTargetDoMove(position=[position2[0], position2[1] + 0.5, position2[2]], duration=4, speed_based=False)
    robot.WaitDo()
    robot.IKTargetDoMove(position=[position2[0], position2[1] + 0.06, position2[2]], duration=2, speed_based=False)
    robot.WaitDo()
    gripper.GripperOpen()
    env.step(50)
    robot.IKTargetDoMove(position=[0, 0.5, 0], duration=2, speed_based=False, relative=True)
    robot.WaitDo()
    
    #Remove the boxes from the simulation
    box1.Destroy()
    box2.Destroy()
    env.step()