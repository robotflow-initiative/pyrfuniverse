# Learning Path

Creating an easy-to-understand and follow learning path is crucial for newcomers, especially when learning a complex robotics physics simulation platform like this. The following is an organized learning directory based on difficulty and practicality, from basics to advanced, covering common to special functionalities:

## Getting Started

### **Scene Construction and Basic Concepts**
- **Scene Building**: [`test_scene`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_scene.py) - Learn how to build, save, and load simulation scenes.
- **Camera Image**: [`test_camera_image`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_camera_image.py) - Understand how to capture camera screenshots, mastering basic viewpoints and observations.
- **Object Basic Data**: [`test_object_data`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_object_data.py) - Understand how to access and manipulate basic data of objects.

### **Model Importing**
- **Importing OBJ Models**: [`test_load_mesh`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_load_mesh.py) - Learn how to import OBJ models as rigid bodies.
- **Importing URDF Files**: [`test_load_urdf`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_load_urdf.py) - Master the importation of URDF files, laying the foundation for advanced robotic arm operations.

### **Visual Aid Tools**
- **Scene Labeling**: [`test_label`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_label.py) - Learn how to perform scene labeling and generate 2D bounding boxes, laying the groundwork for subsequent object recognition tasks.

## Intermediate Applications

### **Interactive Sensor Simulation**
- **Tactile Sensors**: [`test_digit`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_digit.py) - Explore and simulate interactive Digit tactile sensors.
- **GelSlim Tactile Sensor**: [`test_gelslim`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_gelslim.py) - Simulate GelSlim tactile sensors, understanding the application of tactile sensing in robotics.

### **Robotic Arms and Control**
- **Native IK**: [`test_articulation_ik`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_articulation_ik.py) - Learn how to control a robotic arm using inverse kinematics, preparing for advanced tasks.
- **Pick and Place**: [`test_pick_and_place`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_pick_and_place.py) - Learn basic pick and place skills driven by native IK.

### **Debugging and Optimization**
- **Debugging Tools**: [`test_debug`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_debug.py) - Master the loop display of various debugging modules to optimize the simulation experience.

## Advanced Skills

### **Advanced Sensors and Visual Effects**
- **Infrared Depth**: [`test_active_depth`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_active_depth.py) - Delve into the working principles and applications of infrared depth sensors.
- **Point Cloud Processing**: Multiple scripts involve point cloud generation and rendering, such as [`test_point_cloud`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_point_cloud.py), providing tools for handling complex 3D data.

### **Dynamic Interaction and Simulation**
- **Cloth Simulation**: [`test_cloth_attach`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_cloth_attach.py) - Learn how to perform cloth simulation, adding more dynamics and realism to the simulation.
- **Advanced Grasping**: [`test_grasp_sim`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_grasp_sim.py) - Understand complex grasping tasks handling through the Franka arm grasping test.

### **Customization and Special Features**
- **Custom Messages**: [`test_custom_message`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_custom_message.py) - Learn how to create and use custom messages, providing more flexibility for complex interactions.
- **Obstacle Avoidance Planning**: [`test_ompl`](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_ompl.py) - Explore obstacle avoidance planning for robotic arms, a challenging task for advanced users that requires a Linux environment and OMPL installation.

Through this structured learning path from simple to complex, newcomers can gradually build their understanding and mastery of the robotics physics simulation platform, from basic scene construction and object manipulation to complex sensor simulation, robotic arm control, and customization functionalities.

---