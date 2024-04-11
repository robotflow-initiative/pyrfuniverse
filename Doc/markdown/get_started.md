# Get Started

## Installation

Follow the steps below to configure and run example scenes through the release version

1. Create a new conda environment and install pyrfuniverse
   
   ```
   conda create -n rfuniverse python=3.10 -y
   conda activate rfuniverse
   pip install pyrfuniverse
   ```

2. Download the RFUniverse simulation environment

   Option one: Use the pyrfuniverse command line entry
   ```
   pyrfuniverse download
   ```
   By default, it downloads the latest available version to `~/rfuniverse`, you can add the optional argument `-s` to change the download path, `-v` to change the download version
   
   ---

   Option two: Download from Github Release: [RFUniverse Releases](https://github.com/mvig-robotflow/rfuniverse/releases)
   
   After downloading and unzipping, run the program once, and you can close it after entering the scene:
   
   Linux: `RFUniverse_For_Linux/RFUniverse.x86_64`
   
   Windows: `RFUniverse_For_Windows/RFUniverse.exe`

3. Install the test package pyrfuniverse-test and run the example script
   ```
   pip install pyrfuniverse-test
   pyrfuniverse-test test_pick_and_place
   ```
   More examples can be viewed with `pyrfuniverse-test -h`

---

Additional operations that may be required on Linux systems:

If an error occurs during runtime, please check this [document](https://github.com/mvig-robotflow/rfuniverse/issues/3) to supplement dependencies

---

## Test Directory

| Script Name | Function Introduction |
| --- | --- |
| [test_active_depth](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_active_depth.py) | Infrared Depth |
| [test_articulation_ik](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_articulation_ik.py) | Native IK |
| [test_camera_image](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_camera_image.py) | Camera Screenshot Example |
| [test_custom_message](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_custom_message.py) | Custom Messages and Dynamic Messages |
| [test_debug](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_debug.py) | Loop Display of Various Debug Modules |
| [test_digit](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_digit.py) | Interactive Digit Sensor Simulation |
| [test_grasp_pose](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_grasp_pose.py) | Franka Grasp Point Preview |
| [test_grasp_sim](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_grasp_sim.py) | Franka Grasping Test |
| [test_heat_map](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_heat_map.py) | Interactive Heatmap |
| [test_humanbody_ik](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_humanbody_ik.py) | Human Body IK Interface |
| [test_label](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_label.py) | Scene Labeling 2DBBOX |
| [test_ligth](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_light.py) | Lighting Parameter Settings |
| [test_load_mesh](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_load_mesh.py) | Importing OBJ Model as Rigid Body |
| [test_load_urdf](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_load_urdf.py) | Importing URDF File |
| [test_object_data](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_object_data.py) | Object Basic Data |
| [test_pick_and_place](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_pick_and_place.py) | Basic Interface and Grasping Driven by Native IK |
| [test_point_cloud](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_point_cloud.py) | Obtaining Depth Image and Converting to Point Cloud Using Image Width, Height, and FOV |
| [test_point_cloud_render](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_point_cloud_render.py) | Importing and Displaying .PLY Point Cloud File |
| [test_point_cloud_with_intrinsic_matrix](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_point_cloud_with_intrinsic_matrix.py) | Obtaining Depth Image and Converting to Point Cloud Using Camera Intrinsic Matrix |
| [test_save_gripper](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_save_gripper.py) | Saving Gripper as OBJ Model |
| [test_save_obj](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_save_obj.py) | Saving Multiple Objects in the Scene as OBJ Models |
| [test_scene](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_scene.py) | Scene Building/Saving/Loading |
| [test_tobor_move](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_tobor_move.py) | Tobor Wheel Drive Movement |
| [test_urdf_parameter](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_urdf_parameter.py) | Joint Target Position Setting Panel |
| [test_ompl](https://github.com/robotflow-initiative/pyrfuniverse/tree/main/test/pyrfuniverse_test/test/test_ompl.py) | Robotic Arm Obstacle Avoidance Planning<br/>**This example requires Linux and self-installation of OMPL** |

---

## Enter Edit mode

Launch RFUniverse with the <-edit> parameter to enter Edit mode:

Linux:

```
RFUniverse.x86_64 -edit
```

Windows:

```
RFUniverse.exe -edit
```