# test_image_stream

## 1. Basic Functions

- Test the display of the image stream
- Basic idea: A camera in the virtual environment continuously captures objects, while another thread is responsible for displaying the obtained images on the screen.

## 2. Implementation Process

### 2.1 Initialize the Environment

```python
env = RFUniverseBaseEnv(assets=["Camera", "GameObject_Box"])

camera = env.InstanceObject(name="Camera", id=123456, attr_type=attr.CameraAttr)
camera.SetTransform(position=[0, 0.25, 0], rotation=[30, 0, 0])
box = env.InstanceObject(name="GameObject_Box", attr_type=attr.GameObjectAttr)
box.SetTransform(position=[0, 0.05, 0.5], scale=[0.1, 0.1, 0.1])
box.SetColor([1, 0, 0, 1])
```

- Create a camera and an object in the scene and place them in appropriate positions.

### 2.2 Capture and Display the Image Stream

```python
img = None

class ImageThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)

    def run(self):
        while True:
            global img
            if img is not None:
                cv2.imshow("image", img)
                cv2.waitKey(10)

thread = ImageThread()
thread.start()
while True:
    camera.GetRGB(width=512, height=512)
    box.Rotate([0, 1, 0], False)
    env.step()
    image = np.frombuffer(camera.data["rgb"], dtype=np.uint8)
    img = cv2.imdecode(image, cv2.IMREAD_COLOR)
```

- The `ImageThread` class creates a thread that continuously displays the global variable named `img` (i.e., the images captured in the virtual environment) on the screen.
- `GetRGB` is used to control the camera to capture RGB images.
- `Rotate` rotates the object in the virtual environment to create a dynamic effect in the image stream.