# test_debug

## 1 Basic Functions

- Cycle through the functionalities of various debug modules.

## 2 Implementation Process

### 2.1 Initialize Environment

```python
env = RFUniverseBaseEnv(scene_file="DebugScene.json")
```

- Import the scene from a pre-configured `DebugScene.json` file.

### 2.2 Debug Functionality Demonstration

- The debug functions mentioned below can accept a boolean variable named `enable` to toggle the corresponding debug feature on or off.

#### 2.2.1 DebugGraspPoint

![](../image/debug/grasp_point.png)

```python
env.DebugGraspPoint()
env.SendLog("DebugGraspPoint")
env.step(300)
env.DebugGraspPoint(False)
```

- `DebugGraspPoint` can show or hide the position and orientation information of the end-effector of the robotic arm.

#### 2.2.2 DebugObjectID

![](../image/debug/object_id.png)

```python
env.DebugObjectID()
env.SendLog("DebugObjectID")
env.step(300)
env.DebugObjectID(False)
```

- `DebugObjectID` can show or hide the ID information of all objects.

#### 2.2.3 DebugObjectPose

![](../image/debug/pose.png)

```python
env.DebugObjectPose()
env.SendLog("DebugObjectPose")
env.step(300)
env.DebugObjectPose(False)
```

- `DebugObjectPose` can show or hide the base point coordinates, orientation, and scaling information of all objects.

#### 2.2.4 DebugColliderBound

![](../image/debug/collider.png)

```python
env.DebugColliderBound()
env.SendLog("DebugColliderBound")
env.step(300)
env.DebugColliderBound(False)
```

- `DebugColliderBound` can show or hide the collision boxes of all objects.

#### 2.2.5 DebugCollisionPair

![](../image/debug/collision_pair.png)

```python
env.DebugCollisionPair()
env.SendLog("DebugCollisionPair")
env.step(300)
env.DebugCollisionPair(False)
```

- `DebugCollisionPair` can show or hide the collision pairs of all objects.

#### 2.2.6 Debug3DBBox

![](../image/debug/3d_bounding_box.png)

```python
env.Debug3DBBox()
env.SendLog("Debug3DBBox")
env.step(300)
env.Debug3DBBox(False)
```

- `Debug3DBBox` can show or hide the 3D bounding boxes of all objects.

#### 2.2.7 Debug2DBBox

![](../image/debug/2d_bounding_box.png)

```python
env.Debug2DBBox()
env.SendLog("Debug2DBBox")
env.step(300000)
env.Debug2DBBox(False)
```

- `Debug2DBBox` can show or hide the 2D bounding boxes of all objects.

#### 2.2.8 DebugJointLink

![](../image/debug/joint_link.png)

```python
env.DebugJointLink()
env.SendLog("DebugJointLink")
env.step(300)
env.DebugJointLink(False)
```

- `DebugJointLink` can show or hide all joint information.