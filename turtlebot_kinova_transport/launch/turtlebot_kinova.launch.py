from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, RegisterEventHandler, LogInfo
from launch.event_handlers import OnProcessExit
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Launch arguments and parameters
    
    # First, start the TurtleBot bringup
    turtlebot_bringup = ExecuteProcess(
        cmd=['ros2', 'launch', 'turtlebot3_bringup', 'robot.launch.py'],
        output='screen'
    )
    
    # Next, start the Kinova ARM bringup
    kinova_bringup = ExecuteProcess(
        cmd=['ros2', 'launch', 'kortex_driver', 'kortex_driver.launch.py'],
        output='screen'
    )
    
    # Wait until both robots are ready, then launch our integrated node
    integrated_node = Node(
        package='pick_place_lab',
        executable='integrated_pick_place',
        name='integrated_pick_place',
        output='screen'
    )
    
    # Create an event handler to log when the TurtleBot bringup is complete
    turtlebot_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=turtlebot_bringup,
            on_exit=[
                LogInfo(msg="TurtleBot is ready. Starting Kinova arm..."),
                kinova_bringup
            ]
        )
    )
    
    # Create an event handler to log when the Kinova bringup is complete
    kinova_ready = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=kinova_bringup,
            on_exit=[
                LogInfo(msg="Kinova arm is ready. Starting task..."),
                integrated_node
            ]
        )
    )
    
    return LaunchDescription([
        turtlebot_bringup,
        turtlebot_ready,
        kinova_ready
    ])