
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_dir = get_package_share_directory('rover_cpp')  # pkg dir after building in install/share
    urdf = os.path.join(package_dir,'urdf', 'rover_cpp.urdf')   # urdf is saved here in install/share after build
    rviz_config_file=os.path.join(package_dir, 'urdf', 'config.rviz') # config.rviz is saved here in install/share after build
    # print("pkg rover_cpp location:",urdf)
    world_file = 'blocks.sdf' # for right wall following
    world_path = os.path.join(package_dir,'worlds', world_file) 

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher', #'joint_state_publisher_gui'
            name='joint_state_publisher',
            arguments=[urdf]),

        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d',rviz_config_file],
        #     output='screen'),




        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so', '-world', world_path],
            # cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen', 
            arguments=["-topic", "/robot_description", "-entity", "rover_cpp",
                        "-x", "6.50",  # X position
                        "-y", "-2.30",  # Y position
                        "-z", "0.15",  # Z position (height)
                        "-R", "0.0",  # Roll
                        "-P", "0.0",  # Pitch
                        "-Y", ".157"]  # Yaw (rotation around the Z axis)]
        )
    ])


