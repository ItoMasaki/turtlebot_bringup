from launch import LaunchDescription

import launch.actions
import launch_ros.actions

def generate_launch_description():
    return LaunchDescription([
        launch.actions.LogInfo(
    	    msg="ROS2 start turtlebot_bringup minimal node."
	),

        launch_ros.actions.Node(
            package="ydlidar",
            node_executable="ydlidar_node",
            output="screen",
    ),

        launch_ros.actions.Node(
            package="turtlebot_bringup",
            node_executable="turtlebot2",
			output="screen",
	),

    ])
