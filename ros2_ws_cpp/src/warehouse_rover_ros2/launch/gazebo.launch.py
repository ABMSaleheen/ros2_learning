
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_dir = get_package_share_directory('warehouse_rover_ros2')  # pkg dir after building in install/share
    urdf = os.path.join(package_dir,'urdf', 'rover_proto_A1.urdf')   # urdf is saved here in install/share after build
    
    # rviz_config_file=os.path.join(package_dir, 'urdf', 'config.rviz') # config.rviz is saved here in install/share after build
    
    # print("pkg rover location:",urdf)

    # world_file = 'blocks.sdf' # for right wall following
    # world_path = os.path.join(package_dir,'worlds', world_file) 

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            arguments=[urdf]),


        # Node(
        #     package='joint_state_publisher_gui',
        #     executable='joint_state_publisher_gui',
        #     name='joint_state_publisher_gui',
        #     arguments=[urdf]),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d',rviz_config_file],
        #     output='screen'),


        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s','libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so'],
            output='screen',
            # additional_env={
            #     'GAZEBO_MAX_STEP_SIZE': '0.004',
            #     'GAZEBO_REAL_TIME_FACTOR': '1',
            #     'GAZEBO_REAL_TIME_UPDATE_RATE': '250',
            # }
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=["-topic", "/robot_description", "-entity", "rover_proto_A1"],
            name='urdf_spawner',
            output='screen', 
            
        )


    ])

