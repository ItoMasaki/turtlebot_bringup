# ros2 for TurtleBot2_bringup
This is the turtlebot2 bring up on Dashing(ROS2)

## implimentation
 turtlebot2/commands/velocity - you can control by velocity which is based on Twist type.  
 turtlebot2/odometry    - you can subscribe odometry.  
 turtlebot2/imu     - you can subscribe inertial sensor data.
 
## crytical stop
 If you bring up the turtlebot2, turtlebot execute the crytical stop function.

## INSTALL
 First, you have to build using colcon builder.

 ```
 colcon build --packages-select turtlebot_bringup
 ```

 Second, you have to load the setup.bash

 ```
 source install/local_setup.bash
 source install/setup.bash
 ```

 Third, you can run the turtlebot_bringup

 ```
 ros2 run turtlebot_bringup turtlebot2
 ```

## turtlebot_bringup with ydlidar_ros2
 firstly, download ydlidar_ros2
 ```
 git clone https://github.com/yangfuyuan/ydlidar_ros2.git
 ```

 secondly, build any pacakges
 ```
 colcon build
 ```
 
 finally, load any packages
 ```
 source install/local_setup.bash
 source install/setup.bash
 ```

### bringup minimal launch
 ```
 ros2 launch turtlebot_bringup minimal.launch.py
 ```
