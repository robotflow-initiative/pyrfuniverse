import argparse

def pyrfuniverse_test_entry_points():
    parser = argparse.ArgumentParser(description='RFUniverse test entry points')
    subparsers = parser.add_subparsers(dest='script', help='test_active_depth/etc, See the github repo for details')
    subparsers.add_parser('test_active_depth')
    subparsers.add_parser('test_articulation_ik')
    subparsers.add_parser('test_articulation_root')
    subparsers.add_parser('test_camera_image')
    subparsers.add_parser('test_cloth_attach')
    subparsers.add_parser('test_custom_message')
    subparsers.add_parser('test_debug')
    subparsers.add_parser('test_digit')
    subparsers.add_parser('test_gelslim')
    subparsers.add_parser('test_grasp_pose')
    subparsers.add_parser('test_grasp_sim')
    subparsers.add_parser('test_heat_map')
    subparsers.add_parser('test_humanbody_ik')
    subparsers.add_parser('test_image_stream')
    subparsers.add_parser('test_lable')
    subparsers.add_parser('test_light')
    subparsers.add_parser('test_load_mesh')
    subparsers.add_parser('test_load_urdf')
    #subparsers.add_parser('test_load_urdf_akb')
    subparsers.add_parser('test_object_data')
    subparsers.add_parser('test_ompl')
    subparsers.add_parser('test_pick_and_place')
    subparsers.add_parser('test_ply_render')
    subparsers.add_parser('test_point_cloud')
    subparsers.add_parser('test_point_cloud_with_intrinsic_matrix')
    subparsers.add_parser('test_rigidbody_link')
    subparsers.add_parser('test_save_gripper')
    subparsers.add_parser('test_save_obj')
    subparsers.add_parser('test_scene')
    subparsers.add_parser('test_tobor_move')
    subparsers.add_parser('test_urdf_parameter')

    args = parser.parse_args()
    if args.script == 'test_active_depth':
        import pyrfuniverse_test.test.test_active_depth
    elif args.script == 'test_articulation_ik':
        import pyrfuniverse_test.test.test_articulation_ik
    elif args.script == 'test_articulation_root':
        import pyrfuniverse_test.test.test_articulation_root
    elif args.script == 'test_camera_image':
        import pyrfuniverse_test.test.test_camera_image
    elif args.script == 'test_cloth_attach':
        import pyrfuniverse_test.test.test_cloth_attach
    elif args.script == 'test_custom_message':
        import pyrfuniverse_test.test.test_custom_message
    elif args.script == 'test_debug':
        import pyrfuniverse_test.test.test_debug
    elif args.script == 'test_digit':
        import pyrfuniverse_test.test.test_digit
    elif args.script == 'test_gelslim':
        import pyrfuniverse_test.test.test_gelslim
    elif args.script == 'test_grasp_pose':
        import pyrfuniverse_test.test.test_grasp_pose
    elif args.script == 'test_grasp_sim':
        import pyrfuniverse_test.test.test_grasp_sim
    elif args.script == 'test_heat_map':
        import pyrfuniverse_test.test.test_heat_map
    elif args.script == 'test_humanbody_ik':
        import pyrfuniverse_test.test.test_humanbody_ik
    elif args.script == 'test_image_stream':
        import pyrfuniverse_test.test.test_image_stream
    elif args.script == 'test_lable':
        import pyrfuniverse_test.test.test_lable
    elif args.script == 'test_light':
        import pyrfuniverse_test.test.test_light
    elif args.script == 'test_load_mesh':
        import pyrfuniverse_test.test.test_load_mesh
    elif args.script == 'test_load_urdf':
        import pyrfuniverse_test.test.test_load_urdf
    # elif args.script == 'test_load_urdf_akb':
    #    import pyrfuniverse_test.test.test_load_urdf_akb
    elif args.script == 'test_object_data':
        import pyrfuniverse_test.test.test_object_data
    elif args.script == 'test_ompl':
        import pyrfuniverse_test.test.test_ompl
    elif args.script == 'test_pick_and_place':
        import pyrfuniverse_test.test.test_pick_and_place
    elif args.script == 'test_ply_render':
        import pyrfuniverse_test.test.test_ply_render
    elif args.script == 'test_point_cloud':
        import pyrfuniverse_test.test.test_point_cloud
    elif args.script == 'test_point_cloud_with_intrinsic_matrix':
        import pyrfuniverse_test.test.test_point_cloud_with_intrinsic_matrix
    elif args.script == 'test_rigidbody_link':
        import pyrfuniverse_test.test.test_rigidbody_link
    elif args.script == 'test_save_gripper':
        import pyrfuniverse_test.test.test_save_gripper
    elif args.script == 'test_save_obj':
        import pyrfuniverse_test.test.test_save_obj
    elif args.script == 'test_scene':
        import pyrfuniverse_test.test.test_scene
    elif args.script == 'test_tobor_move':
        import pyrfuniverse_test.test.test_tobor_move
    elif args.script == 'test_urdf_parameter':
        import pyrfuniverse_test.test.test_urdf_parameter
    else:
        print('Invalid script name: {}'.format(args.script))
