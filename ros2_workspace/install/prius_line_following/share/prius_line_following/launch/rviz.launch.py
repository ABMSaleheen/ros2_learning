import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    package_dir = get_package_share_directory('prius_line_following')
    sdf_file = os.path.join(package_dir, 'urdf', 'prius_hybrid', 'model.sdf')

    return LaunchDescription([
        # Launch Gazebo with the SDF model
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # Spawn the model in Gazebo
        ExecuteProcess(
            cmd=['ros2', 'run', 'gazebo_ros', 'spawn_entity.py', 
                 '-file', sdf_file,
                 '-entity', 'prius_hybrid'],
            output='screen'
        ),
    ])
