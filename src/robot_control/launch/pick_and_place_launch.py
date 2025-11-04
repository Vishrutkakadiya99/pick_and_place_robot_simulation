import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Package Paths
    pkg_description = get_package_share_directory('robot_description')
    pkg_moveit_config = get_package_share_directory('arm_moveit_config') # CHANGE THIS TO YOUR MOVEIT CONFIG PACKAGE NAME
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    # Arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    robot_description_content = Command(['xacro ', os.path.join(pkg_description, 'urdf', 'arm.urdf.xacro')])

    # 1. Robot State Publisher Node (RSP)
    # Publishes the /robot_description and joint transforms to /tf
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description_content, 
                     'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 2. Gazebo Launch (Starts the simulation world)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'world': 'empty.world'}.items() # Or your custom world file
    )

    # 3. Spawn Robot Entity in Gazebo (Uses the /robot_description topic)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-entity', 'six_dof_arm',
                   '-topic', 'robot_description',
                   '-x', '0.0', '-y', '0.0', '-z', '0.0'],
        output='screen',
        # Wait for Gazebo to be ready before spawning
    )
    
    # 4. Controller Manager & Spawning Controllers
    # The 'joint_trajectory_controller' is needed for MoveIt2 execution
    joint_trajectory_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_trajectory_controller', '--controller-manager', '/controller_manager'],
    )
    
    # 5. MoveIt2 Launch (Starts the planning server and RViz)
    # This assumes your MoveIt2 config package has a file like demo.launch.py
    moveit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_moveit_config, 'launch', 'arm_planning_execution.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6. Pick and Place Logic Node
    # Use a TimerAction to ensure MoveIt2 is fully loaded before the P&P node starts
    pick_and_place_node_timer = TimerAction(
        period=10.0, # Wait 10 seconds (adjust as needed for MoveIt2 to load)
        actions=[
            Node(
                package='robot_control',
                executable='pick_and_place_node',
                name='pick_and_place_executor',
                output='screen',
            )
        ]
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock if true'),
        robot_state_publisher_node,
        gazebo_launch,
        
        # Use TimerAction to ensure components spawn in order
        TimerAction(
            period=5.0,
            actions=[spawn_entity]
        ),
        TimerAction(
            period=7.0,
            actions=[joint_trajectory_controller_spawner]
        ),
        
        moveit_launch,
        pick_and_place_node_timer,
    ])
