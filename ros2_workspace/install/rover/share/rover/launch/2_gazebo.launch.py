
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_dir = get_package_share_directory('rover')  # pkg dir after building in install/share
    urdf = os.path.join(package_dir,'urdf', 'rover.urdf')   # urdf is saved here in install/share after build
    rviz_config_file=os.path.join(package_dir, 'urdf', 'config.rviz') # config.rviz is saved here in install/share after build
    # print("pkg rover location:",urdf)


    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            arguments=[urdf]),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            arguments=[urdf]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d',rviz_config_file],
            output='screen'),




        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            name='urdf_spawner',
            output='screen', 
            arguments=["-topic", "/robot_description", "-entity", "rover"]
        )
    ])


