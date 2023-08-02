# from pyrfuniverse.envs.base_env import RFUniverseBaseEnv
# try:
#     from pyrfuniverse.envs.gym_wrapper_env import RFUniverseGymWrapper
#     from pyrfuniverse.envs.gym_goal_wrapper import RFUniverseGymGoalWrapper
# except ImportError:
#     pass
# from pyrfuniverse.envs.franka_grasp_env import FrankaGraspEnv
# from pyrfuniverse.envs.franka_push_env import FrankaPushEnv
# from pyrfuniverse.envs.balance_ball_env import BalanceBallEnv
# from pyrfuniverse.envs.balance_ball_env import BalanceBallEnvV0
# from pyrfuniverse.envs.bouncer_env import BouncerEnv
# from pyrfuniverse.envs.bouncer_env import BouncerEnvV0
# from pyrfuniverse.envs.roller_env import RollerEnv
# from pyrfuniverse.envs.roller_env import RollerEnvV0
# from pyrfuniverse.envs.gripper_nail import NailCardEnv
# from pyrfuniverse.envs.gripper_nail import NailCanEnv
# from pyrfuniverse.envs.multi_agent_navigation_env import MultiAgentNavigationEnv
# from pyrfuniverse.envs.tobor_robotiq85_manipulation_env import ToborRobotiq85ManipulationEnv
# from pyrfuniverse.envs.ur5_box_env import Ur5BoxEnv
# from pyrfuniverse.envs.ur5_drawer_env import Ur5DrawerEnv

# Other projects based on RFUniverse
# import pyrfuniverse.envs.robotube
# import pyrfuniverse.envs.rcareworld

# __all__ = [
#     'RFUniverseBaseEnv', 'FrankaGraspEnv', 'FrankaPushEnv', 'BalanceBallEnv',
#     'RFUniverseGymWrapper', 'BalanceBallEnvV0', 'RFUniverseGymGoalWrapper',
#     'BouncerEnv', 'BouncerEnvV0', 'RollerEnv', 'RollerEnvV0', 'NailCardEnv',
#     'MultiAgentNavigationEnv', 'ToborRobotiq85ManipulationEnv', 'Ur5BoxEnv',
#     'Ur5DrawerEnv',
# ]
#
#
# rfuniverse_build_base_root = '/home/haoyuan/workspace/rfuniverse/rfuniverse/rfuniverse_build/'
#
# try:
#     from gym.envs.registration import register
#
#     register(
#         id='BalanceBallEnv-v0',
#         entry_point='pyrfuniverse.envs:BalanceBallEnvV0'
#     )
#
#     register(
#         id='BouncerEnv-v0',
#         entry_point='pyrfuniverse.envs:BouncerEnvV0'
#     )
#
#     register(
#         id='RollerEnv-v0',
#         entry_point='pyrfuniverse.envs:RollerEnvV0'
#     )
#
#     register(
#         id='MultiAgentNavigation-v1',
#         entry_point='pyrfuniverse.envs:MultiAgentNavigationEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'MultiAgentNavigationServer/RFUniverse.x86_64',
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'num_agents': 5,
#         },
#     )
#
#     register(
#         id='MultiAgentNavigation-v2',
#         entry_point='pyrfuniverse.envs:MultiAgentNavigationEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'MultiAgentNavigationServer/RFUniverse.x86_64',
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'num_agents': 5,
#             'reset_on_collision': True
#         },
#     )
#
#     register(
#         id='MultiAgentNavigation-v3',
#         entry_point='pyrfuniverse.envs:MultiAgentNavigationEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'MultiAgentNavigationServer/RFUniverse.x86_64',
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'num_agents': 3,
#         },
#     )
#
#     register(
#         id='MultiAgentNavigation-v4',
#         entry_point='pyrfuniverse.envs:MultiAgentNavigationEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'MultiAgentNavigationServer/RFUniverse.x86_64',
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'num_agents': 3,
#             'reset_on_collision': True
#         },
#     )
#
#     # Franka robotics
#     for reward_type in ['sparse', 'dense']:
#         suffix = 'Dense' if reward_type == 'dense' else ''
#         kwargs = {
#             'executable_file': rfuniverse_build_base_root + 'FrankaRoboticsServer/RFUniverse.x86_64',
#             'max_episode_length': 50,
#             'reward_type': reward_type,
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody'
#         }
#
#         register(
#             id='FrankaPickAndPlace{}-v1'.format(suffix),
#             entry_point='pyrfuniverse.envs.robotics:FrankaPickAndPlaceEnv',
#             kwargs=kwargs,
#             max_episode_steps=50,
#         )
#
#         register(
#             id='FrankaReach{}-v1'.format(suffix),
#             entry_point='pyrfuniverse.envs.robotics:FrankaReachEnv',
#             kwargs=kwargs,
#             max_episode_steps=50,
#         )
#
#         register(
#             id='FrankaPush{}-v1'.format(suffix),
#             entry_point='pyrfuniverse.envs.robotics:FrankaPushEnv',
#             kwargs=kwargs,
#             max_episode_steps=50,
#         )
#
#     # Franka-Cloth env
#     franka_cloth_kwargs = {
#         'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/obi_cloth',
#         'executable_file': rfuniverse_build_base_root + 'FrankaCloth/RFUniverse.x86_64',
#         'reward_type': 'dense'
#     }
#     register(
#         id='FrankaCloth-v1',
#         entry_point='pyrfuniverse.envs.robotics:FrankaClothEnv',
#         kwargs=franka_cloth_kwargs,
#         max_episode_steps=50
#     )
#
#     # Franka-Softbody env
#     franka_softbody_kwargs = {
#         'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/softbody',
#         # 'executable_file': rfuniverse_build_base_root + 'FrankaSoftbodyServer/RFUniverse.x86_64',
#         'executable_file': None,
#         'reward_type': 'sparse'
#     }
#     register(
#         id='FrankaSoftbody-v1',
#         entry_point='pyrfuniverse.envs.robotics:FrankaSoftbodyEnv',
#         kwargs=franka_softbody_kwargs,
#         max_episode_steps=50
#     )
#
#     # Franka-ClothFold env
#     franka_cloth_fold_kwargs = {
#         'asset_bundle_file': '/home/haoyuan/rfuniverse/RFUniverse/Assets/AssetBundles/Linux/obi_cloth',
#         # 'executable_file': rfuniverse_build_base_root + 'FrankaClothFoldServer/RFUniverse.x86_64',
#         'executable_file': None,
#         'reward_type': 'sparse'
#     }
#     register(
#         id='FrankaClothFold-v1',
#         entry_point='pyrfuniverse.envs.robotics:FrankaClothFoldEnv',
#         kwargs=franka_cloth_fold_kwargs,
#         max_episode_steps=20
#     )
#
#     # A relatively easy and naive NailCard environment
#     register(
#         id='NailCard-v1',
#         entry_point='pyrfuniverse.envs:NailCardEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'NailCardServer/RFUniverse.x86_64',
#             'rotation_factor': 0,
#             'goal_baseline': 0.02,
#         },
#         max_episode_steps=50,
#     )
#
#     # The normal version
#     register(
#         id='NailCard-v2',
#         entry_point='pyrfuniverse.envs:NailCardEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'NailCardServer/RFUniverse.x86_64',
#         },
#         max_episode_steps=50,
#     )
#
#     # Fixed the nail movement
#     register(
#         id='NailCard-v3',
#         entry_point='pyrfuniverse.envs:NailCardEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'NailCardServer/RFUniverse.x86_64',
#             'nail_movement_factor': 0
#         },
#         max_episode_steps=50,
#     )
#
#     # A relatively easy and naive NailCard environment
#     register(
#         id='Robotiq85NailCard-v1',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85NailCardEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85NailCardServer/RFUniverse.x86_64',
#             'rotation_factor': 0,
#             'goal_baseline': 0.02,
#         },
#         max_episode_steps=50,
#     )
#
#     # The normal version
#     register(
#         id='Robotiq85NailCard-v2',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85NailCardEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85NailCardServer/RFUniverse.x86_64',
#         },
#         max_episode_steps=50,
#     )
#
#     # Fixed the nail movement
#     register(
#         id='Robotiq85NailCard-v3',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85NailCardEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85NailCardServer/RFUniverse.x86_64',
#             'vertical_movement_factor': 0.02,
#             'nail_movement_factor': 0
#         },
#         max_episode_steps=50,
#     )
#
#     # Robotiq85 Nail Coin
#     register(
#         id='Robotiq85NailCoin-v1',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85NailCoinEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85NailCoinServer/RFUniverse.x86_64',
#             'vertical_movement_factor': 0.02,
#             'nail_movement_factor': 0
#         },
#         max_episode_steps=50,
#     )
#
#     # Nail-Can environment
#     register(
#         id='NailCan-v1',
#         entry_point='pyrfuniverse.envs:NailCanEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'NailCanServer/RFUniverse.x86_64',
#         }
#     )
#
#     register(
#         id='Robotiq85NailCan-v1',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85NailCanEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85NailCanServer/RFUniverse.x86_64',
#         }
#     )
#
#     register(
#         id='Robotiq85NailBook-v1',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85NailBookEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85NailBook/RFUniverse.x86_64',
#         }
#     )
#
#     # Robotiq85 insert
#     register(
#         id='Robotiq85Insert-v1',
#         entry_point='pyrfuniverse.envs.gripper_nail:Robotiq85InsertEnv',
#         kwargs={
#             'executable_file': rfuniverse_build_base_root + 'Robotiq85InsertServer/RFUniverse.x86_64',
#         }
#     )
#
#     register(
#         id='Cleaner-v1',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerServer/RFUniverse.x86_64'
#         }
#     )
#
#     register(
#         id='Cleaner-v2',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'reset_on_collision': False,
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerServer/RFUniverse.x86_64',
#         }
#     )
#
#     register(
#         id='Cleaner-v3',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerWithWallServer/RFUniverse.x86_64'
#         }
#     )
#
#     register(
#         id='Cleaner-v4',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'reset_on_collision': False,
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerWithWallServer/RFUniverse.x86_64'
#         }
#     )
#
#     register(
#         id='Cleaner-v5',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerWithWallServer/RFUniverse.x86_64',
#             'velocity_reward': True
#         }
#     )
#
#     register(
#         id='Cleaner-v6',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerWithWallServer/RFUniverse.x86_64',
#             'velocity_reward': True,
#             'collision_multiplier': 50,
#         }
#     )
#
#     register(
#         id='Cleaner-v7',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'asset_bundle_file': '/home/haoyuan/rfuniverse/RFUniverse/Assets/AssetBundles/Linux/rigidbody',
#             # 'executable_file': rfuniverse_build_base_root + 'CleanerWithWall/RFUniverse.x86_64',
#             'executable_file': None,
#             'velocity_reward': True,
#             'collision_multiplier': 50,
#             'obs_type': 'box',
#             'max_episode_length': 200
#         }
#     )
#
#     register(
#         id='Cleaner-v8',
#         entry_point='pyrfuniverse.envs.multi_agent:CleanerEnv',
#         kwargs={
#             'asset_bundle_file': rfuniverse_build_base_root + 'AssetBundles/rigidbody',
#             'executable_file': rfuniverse_build_base_root + 'CleanerWithWallServer/RFUniverse.x86_64',
#             'grid_reward_per_step': True
#         }
#     )
#
#     register(
#         id='UR5Box-v1',
#         entry_point='pyrfuniverse.envs:Ur5BoxEnv',
#         kwargs={
#             'max_steps': 50,
#             'reward_type': 'sparse',
#             'min_open_angle': 30,
#             'executable_file': rfuniverse_build_base_root + 'UR5Box/RFUniverse.x86_64'
#         },
#         max_episode_steps=50,
#     )
#
#     register(
#         id='ToborPull-v1',
#         entry_point='pyrfuniverse.envs.tobor_robotics:ToborPushPullEnv',
#         kwargs={
#             'max_steps': 100,
#             'asset_bundle_file': '/home/haoyuan/workspace/rfuniverse/rfuniverse/RFUniverse/Assets/AssetBundles/Linux/articulation',
#             'pull': True,
#             'executable_file': rfuniverse_build_base_root + 'ToborPushPull/RFUniverse.x86_64'
#         },
#         max_episode_steps=100
#     )
#
#     register(
#         id='ToborPush-v1',
#         entry_point='pyrfuniverse.envs.tobor_robotics:ToborPushPullEnv',
#         kwargs={
#             'max_steps': 100,
#             'asset_bundle_file': '/home/haoyuan/workspace/rfuniverse/rfuniverse/RFUniverse/Assets/AssetBundles/Linux/articulation',
#             'pull': False,
#             'executable_file': rfuniverse_build_base_root + 'ToborPushPull/RFUniverse.x86_64'
#         },
#         max_episode_steps=100
#     )
#
# except ImportError:
#     print('No gym installed. Please install gym!')
#     pass
