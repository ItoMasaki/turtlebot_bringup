# ros2 for TurtleBot2_bringup
This is the turtlebot2 bring up on Dashing.  
Now not supported Crystal.  

## implimentation
turtlebot2/commands/velocity  
 - you can control by velocity which is based on Twist type.  
 If you use anguler velocity, you have to use way of degree.  
 So you can only use from -100 to 100 \[degree/t.\]
 
turtlebot2/commands/reset_pose  
 - able to reset pose (0, 0, 0) by send True.  
 
turtlebot2/odometry  
 - you can subscribe odometry which type is Quaternion.  
 Not Euler!!! Be careful.  
 
turtlebot2/imu  
 - you can subscribe inertial sensor data.
 
## dashing stop
 If you bring up the turtlebot2, turtlebot execute the dashing stop function.

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
