(understanding_pyrfuniverse_1)=
# Understanding pyrfuniverse

In `pyrfuniverse`, we provide a lot of useful API to help users build their own simulation environment and verify or train with any algorithms. To achieve this, we have to find a way to communicate between Unity and Python, since most algorithms are implemented in Python, not C# (which is Unity official support language). We build our own communication system based on [ML-Agents](https://github.com/Unity-Technologies/ml-agents) and we are working on implementing a more light-weight communication base from scratch.

In this page, you will know the basic usage of [pyrfuniverse.attributes](pyrfuniverse_attributes) module and how to extend your own API if we haven't provided the function you need.

(first_example)=
## The basic usage and data flow in pyrfuniverse.attributes

In `pyrfuniverse.attributes`, we provide all useful APIs which can operate the agents in Unity do whatever you want. For example, you can move a robot arm by setting target joint positions or capture a screenshot of camera by sending a signal. As you can see, different operations will need various parameters and may return multiple values. This will need a case-by-case implementation, both in Python and Unity (C#).

Each class in `pyrfuniverse.attributes` will have a member variable named `id` with a unique value, and it's bind with a unique object in Unity. When we call a function from the instance of such attribute, it will 'tell' the object in Unity what to do. 

Let's take camera_attr for example. 

First, let's import classes and define our environment with assets.
```python
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
import cv2
import numpy as np

env = RFUniverseBaseEnv(assets=['Camera'])
```

Then, we will instanciate a camera object with a given ID and the corresponding attribute type. Since we instanciate a camera here, we will use a `CameraAttr` class.
```python
camera = env.InstanceObject(name='Camera', id=123456, attr_type=attr.CameraAttr)
```

Next, we can use the awesome APIs provided in `CameraAttr` class. Let's 
set the camera to a given pose and get some images from the camera.
```python
camera.SetTransform(position=[0, 0.25, 0], rotation=[30, 0, 0])
camera.GetDepth(width=512, height=512, zero_dis=1, one_dis=5)
camera.GetDepthEXR(width=512, height=512)
camera.GetRGB(width=512, height=512)
```

After the codes above, Unity has accepted our operations and the instanciated camera will begin working! Now we have to wait Unity handle these work and we can get our image back. In `pyrfuniverse`, we use a simple function `step()` to wait for Unity handle all complex work.
```python
env.step()
```

After a short wait, we can finally get our images and process them.
```python
print(camera.data['depth'])
print(camera.data['depth_exr'])
print(camera.data['rgb'])
image_np = np.frombuffer(camera.data['rgb'], dtype=np.uint8)
image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
print(image_np.shape)
env.close()
cv2.imshow("rgb", image_np)
cv2.waitKey(0)
```

From the above example, as you can see, we use provided API to operate any object, and get the useful information back from a dict member variable named `data`. Of course, there are more keys in `data`. For the full list of keys, please refer to [pyrfuniverse.attributes](pyrfuniverse_attributes) documentation. Note that data keys in parent class is also available in the child class: i.e., `CameraAttr.data['position']` is available though key 'position' is only listed in `BaseAttr.data`, since `CameraAttr` is inherited from `BaseAttr`.

## Extend custom attributes and API

We have mentioned above that operations in `pyrfuniverse` all need a case-by-case implementation, both in Python and Unity (C#). Thus, when we want to extend a custom attribute, we must also implement both Python and C#.

For the python side, please refer to [custom_attr.py](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/pyrfuniverse/attributes/custom_attr.py) and add codes according to the comments.

For the Unity side, please refer to [CustomAttr.cs](https://github.com/mvig-robotflow/rfuniverse/blob/main/Assets/RFUniverse/Scripts/Attributes/CustomAttr.cs) and add codes according to the comments.
