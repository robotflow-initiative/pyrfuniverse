from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
import pyrfuniverse.attributes as attr

env = RFUniverseBaseEnv(assets=['Rigidbody_Box', 'franka_panda'])

box = env.InstanceObject(name='Rigidbody_Box', id=123456, attr_type=attr.RigidbodyAttr)
box.SetTransform(position=[0, 1, 0])
env.step(5)
print('Rigidbody_Box:')
print('position:')
print(box.data['position'])
print('rotation:')
print(box.data['rotation'])
print('quaternion:')
print(box.data['quaternion'])
print('local_position:')
print(box.data['local_position'])
print('local_rotation:')
print(box.data['local_rotation'])
print('local_quaternion:')
print(box.data['local_quaternion'])
print('velocity:')
print(box.data['velocity'])
print('angular_vel:')
print(box.data['angular_vel'])

robot = env.InstanceObject(name='franka_panda', id=789789, attr_type=attr.ControllerAttr)
robot.SetTransform(position=[1, 0, 0])
env.step()
print('franka_panda:')
print('number_of_joints:')
print(robot.data['number_of_joints'])
print('positions:')
print(robot.data['positions'])
print('rotations:')
print(robot.data['rotations'])
print('quaternion:')
print(robot.data['quaternion'])
print('local_positions:')
print(robot.data['local_positions'])
print('local_rotations:')
print(robot.data['local_rotations'])
print('local_quaternion:')
print(robot.data['local_quaternion'])
print('velocities:')
print(robot.data['velocities'])
print('number_of_moveable_joints:')
print(robot.data['number_of_moveable_joints'])
print('joint_positions:')
print(robot.data['joint_positions'])
print('joint_velocities:')
print(robot.data['joint_velocities'])
print('joint_accelerations:')
print(robot.data['joint_accelerations'])
print('joint_force:')
print(robot.data['joint_force'])
print('joint_types:')
print(robot.data['joint_types'])
print('joint_lower_limit:')
print(robot.data['joint_lower_limit'])
print('joint_upper_limit:')
print(robot.data['joint_upper_limit'])
env.close()