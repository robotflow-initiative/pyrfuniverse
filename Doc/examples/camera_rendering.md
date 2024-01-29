# Camera Rendering Example

## Introduction

This example will guide you through [test_camera_image.py](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_camera_image.py) that captures images from the RFUniverse simulation environment using a simulated camera.

## Understanding the Code 

The code consists of several sections. Below we will discuss each part of the code line by line.

1. Importing Libraries: 

```python
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr
import cv2
```

2. Creating a Simulation Environment Instance: 

This line of code creates an instance of the RFUniverse simulation environment and defines the camera as an asset.

```python
env = RFUniverseBaseEnv(assets=['Camera'])
```

3. Creating a Camera and capturing its image: 

The next set of code initializes the camera object by assigning it an ID and position in the environment. Then, it captures its RGB image, the depth image, and depth image in EXR format.

```python
camera = env.InstanceObject(name='Camera', id=123456, attr_type=attr.CameraAttr)
camera.SetTransform(position=[0, 0.25, 0], rotation=[30, 0, 0])
camera.GetDepth(width=512, height=512, zero_dis=1, one_dis=5)
camera.GetDepthEXR(width=512, height=512)
camera.GetRGB(width=512, height=512)
```

4. Displaying the Camera Image: 

The next set of code retrieves the captured RGB image from the Camera object and converts it to a numpy array using the `np.frombuffer()` method. Next, the image is decoded using the "cv2.imdecode()" method to form a colored image. The dimensions of the image are then printed using the ".shape" attribute. Last, the image is displayed using the "cv2.imshow()" function.

```python
image_np = np.frombuffer(camera.data['rgb'], dtype=np.uint8)
image_np = cv2.imdecode(image_np, cv2.IMREAD_COLOR)
print(image_np.shape)
cv2.imshow("rgb", image_np)
cv2.waitKey(0)
```

5. Closing the Simulation Environment: 

The last line of the code closes the RFUniverse simulation environment.

```python
env.close()
```

You can modify the position of the camera by changing the `position` parameter in the `SetTransform` method. And, you can modify the width and height of the image by changing the `width` and `height` parameters in the `GetDepth`, `GetDepthEXR`, and `GetRGB` methods, respectively.
