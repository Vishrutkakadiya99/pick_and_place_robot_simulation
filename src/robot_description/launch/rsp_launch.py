import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. Declare Launch Arguments
    # Argument to decide if simulation time (from Gazebo) should be used
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    
    # Get the package share directory
    pkg_description = get_package_share_directory('robot_description')

    # 2. Get the URDF/Xacro File Content
    # Use the 'Command' substitution to execute the 'xacro' command and process the file
    robot_description_content = Command([
        'xacro ', os.path.join(pkg_description, 'urdf', 'arm.urdf.xacro')
    ])

    # 3. Robot State Publisher Node (RSP)
    # Reads the processed URDF and publishes /tf transforms
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    # 4. Return Launch Description
    return LaunchDescription([
        robot_state_publisher_node
    ])
