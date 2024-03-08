# Get Started

## Installation

Follow the steps below to configure and run the sample scene through the published version:

1. Download the [RFUniverse Releases](https://github.com/mvig-robotflow/rfuniverse/releases) and unzip it:
  
  Run the program once, enter the scene and then close it:
  
  Linux：`RFUniverse_For_Linux/RFUniverse.x86_64`
  
  Windows：`RFUniverse_For_Windows/RFUniverse.exe`
  
2. Create a conda environment and activate it, install the 'pyrfuniverse' package with the **<mark>same version number</mark>** as the RFUniverse Release:
  
  > :warning:Please replace [0.10.7] in the following command with the version number of the RFUniverse Releases you downloaded.
  
  ```bash
  conda create -n rfuniverse python=3.10 -y
  conda activate rfuniverse
  pip install pyrfuniverse==0.10.7
  ```
  
3. Clone the pyrfuniverse, switch to the Tag with the **<mark>same version number</mark>** as the RFUniverse Release:
  
  > :warning:Please replace [0.10.7] in the following command with the version number of the RFUniverse Releases you downloaded.
  
  ```bash
  git clone https://github.com/mvig-robotflow/pyrfuniverse.git
  cd pyrfuniverse
  git checkout v0.10.7
  ```
  
4. Run any Python script under 'pyrfuniverse/Test' (some scripts may require installation of large packages such as 'pybullet', 'open3d', etc.):
  
  ```bash
  cd Test
  python test_pick_and_place.py
  ```
  

---

Additional operations that may be required on Linux systems:

If an error occurs during runtime, please check this [document](https://github.com/mvig-robotflow/rfuniverse/issues/3) to supplement dependencies

---

## Test Directory

| Script Name | Function Introduction |
| --- | --- |
| [test_active_depth](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_active_depth.py) | Infrared Depth |
| [test_articulation_ik](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_articulation_ik.py) | Native IK |
| [test_camera_image](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_camera_image.py) | Camera Screenshot Example |
| [test_custom_message](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_custom_message.py) | Custom Messages and Dynamic Messages |
| [test_debug](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_debug.py) | Loop Display of Various Debug Modules |
| [test_digit](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_digit.py) | Interactive Digit Sensor Simulation |
| [test_grasp_pose](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_grasp_pose.py) | Franka Grasp Point Preview |
| [test_grasp_sim](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_grasp_sim.py) | Franka Grasping Test |
| [test_heat_map](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_heat_map.py) | Interactive Heatmap |
| [test_humanbody_ik](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_humanbody_ik.py) | Human Body IK Interface |
| [test_label](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_label.py) | Scene Labeling 2DBBOX |
| [test_ligth](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_light.py) | Lighting Parameter Settings |
| [test_load_mesh](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_load_mesh.py) | Importing OBJ Model as Rigid Body |
| [test_load_urdf](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_load_urdf.py) | Importing URDF File |
| [test_object_data](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_object_data.py) | Object Basic Data |
| [test_pick_and_place](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_pick_and_place.py) | Basic Interface and Grasping Driven by Native IK |
| [test_point_cloud](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_point_cloud.py) | Obtaining Depth Image and Converting to Point Cloud Using Image Width, Height, and FOV |
| [test_point_cloud_render](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_point_cloud_render.py) | Importing and Displaying .PLY Point Cloud File |
| [test_point_cloud_with_intrinsic_matrix](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_point_cloud_with_intrinsic_matrix.py) | Obtaining Depth Image and Converting to Point Cloud Using Camera Intrinsic Matrix |
| [test_save_gripper](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_save_gripper.py) | Saving Gripper as OBJ Model |
| [test_save_obj](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_save_obj.py) | Saving Multiple Objects in the Scene as OBJ Models |
| [test_scene](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_scene.py) | Scene Building/Saving/Loading |
| [test_tobor_move](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_tobor_move.py) | Tobor Wheel Drive Movement |
| [test_urdf_parameter](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_urdf_parameter.py) | Joint Target Position Setting Panel |
| [test_ompl](https://github.com/mvig-robotflow/pyrfuniverse/blob/main/Test/test_ompl.py) | Robotic Arm Obstacle Avoidance Planning<br/>**This example requires Linux and self-installation of OMPL** |

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