# Annotating Image and Scene Example

## Introduction

This example will guide you through [test_label.py](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_label.py) which captures an image from an RFUniverse simulation environment, annotates it with various information, and saves the annotated image.

## Understanding the Code 

The code consists of several sections. Below we will discuss each part of the code line by line.

1. Importing Libraries: 

```python
from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import numpy as np
import cv2
```

2. Creating a Simulation Environment Instance: 

This line of code creates an instance of the RFUniverse simulation environment by loading a predefined JSON file describing the scene.

```python
env = RFUniverseBaseEnv(scene_file='SimpleYCBModel.json')
```

3. Creating a Camera and capturing its image and scene: 

The next set of code retrieves the camera object by its ID and captures its RGB, depth, normal, amodal mask images, 2d bounding box, and 3d bounding box. It also retrieves the object IDs from the camera object's ID map.

```python
env.step()
camera = env.GetAttr(981613)
camera.GetRGB(512, 512)
camera.GetID(512, 512)
camera.GetNormal(512, 512)
camera.GetDepth(0.1, 2., 512, 512, )
camera.GetAmodalMask(655797, 512, 512)
camera.Get2DBBox(512, 512)
camera.Get3DBBox()

id_map = np.frombuffer(camera.data['id_map'], dtype=np.uint8)
id_map = cv2.imdecode(id_map, cv2.IMREAD_COLOR)
```

4. Annotating the Captured Images: 

The next set of code annotates the images by overlaying a rectangle around each identified object. The ID map is used to identify the location, size and ID of the objects in the image.

```python
print('2d_bounding_box:')
for i in camera.data['2d_bounding_box']:
    print(i)
    print(camera.data['2d_bounding_box'][i])
    position = camera.data['2d_bounding_box'][i]['position']
    size = camera.data['2d_bounding_box'][i]['size']
    tl_point = (int(position[0] + size[0]/2), int(512 - position[1] + size[1]/2))
    br_point = (int(position[0] - size[0]/2), int(512 - position[1] - size[1]/2))
    cv2.rectangle(id_map, tl_point, br_point, (255, 255, 255), 1)

print('3d_bounding_box:')
for i in camera.data['3d_bounding_box']:
    print(i)
    print(camera.data['3d_bounding_box'][i])
```

5. Saving the Annotated Image: 

The next set of code displays each annotated image and saves the annotated image into a file.

```python
cv2.imshow("Image with Object Annotations", id_map)
cv2.waitKey(0)
filename = 'annotated_image.png'
cv2.imwrite(filename, id_map)
```

6. Cleaning Up: 

The last set of code cleans up the simulation environment.

```python
env.Pend()
env.close()
```
