#include <libkobuki.h>

#include <rclcpp/rclcpp.hpp>

#include <tf2_ros/transform_broadcaster.h>

#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>

using namespace rt_net;
using namespace std;
using namespace chrono_literals;

class Turtlebot :
    public rclcpp::Node {
        private :

            // init kobuki
            Kobuki *kobuki;
    
            float Target_Angular_Velocity = 0;
            float System_Angular_Velocity = 0;
            float Target_Linear_Velocity = 0;
    
            // max voltage
            // float max_voltage = 15.3;
    
            // init chrono timer
            chrono::system_clock::time_point base_time;
            chrono::system_clock::time_point now_time;
    
            // run time
            float run_time = 1/50;
    
            // init subscription
            rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity;
            rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr reset;
    
            // init publisher
            rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery;
            rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr inertial;
            rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_0;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_1;
            rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr button_2;
            
            // transform broadcaster
            std::shared_ptr<tf2_ros::TransformBroadcaster> odom_broadcaster;
    
            // init timer
            rclcpp::TimerBase::SharedPtr odometryTimer;
            rclcpp::TimerBase::SharedPtr inertialTimer;
            rclcpp::TimerBase::SharedPtr PIDTimer;
            rclcpp::TimerBase::SharedPtr emergencyTimer;
            rclcpp::TimerBase::SharedPtr buttonTimer;
      
            // now position
            double N_position_x = 0;
            double N_position_y = 0;
            double N_orientation_theta = 0;
    
            // now velocity
            double N_linear_x_velocity = 0;
            double N_linear_y_velocity = 0;
            double N_linear_z_velocity = 0;
      
            // old position
            double O_position_x = 0;
            double O_position_y = 0;
            double O_orientation_theta = 0;

            double heading;
            double heading_offset = 0.0/0.0;
            double head_angle;
    
            // seconds
            double millisec;
      
            // check Wheel Drop
            void checkWheelDrop();
      
            // translate_coordinate
            geometry_msgs::msg::Quaternion translateCoordinate(double x, double y, double z);
            geometry_msgs::msg::TransformStamped odom_trans;
    
            // init odom
            nav_msgs::msg::Odometry odom_msg = nav_msgs::msg::Odometry();

            // init bool
            std_msgs::msg::Bool bool_msg = std_msgs::msg::Bool();
       
            // calculate velocity
            double calculateVelocity(double now_velocity, double old_velocity, float time);
    
            // get velocity
            void getVelocity(geometry_msgs::msg::Twist::SharedPtr msg);
    
            // controle that based on velocity
            void controleByVelocity();
    
            // publish odometry
            void publishOdometry();
    
            // publish inertial
            void publishInertial();

            // get emergency
            void getEmergency();

            void getButtonPush();

            void resetPose(std_msgs::msg::Bool::SharedPtr msg);
    
    public :
        Turtlebot();
};
