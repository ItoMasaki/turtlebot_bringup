#include <libkobuki.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <builtin_interfaces/msg/time.hpp>


using namespace rt_net;
using namespace std;
using namespace chrono_literals;


class Turtlebot : public rclcpp::Node {
	private :
		// kobuki device special
		const char * device_special = "/dev/kobuki";

		// init kobuki
		Kobuki *kobuki = createKobuki(KobukiStringArgument(device_special));

		// max voltage
		// float max_voltage = 15.3;

		// init chrono timer
		chrono::system_clock::time_point base_time;
		chrono::system_clock::time_point now_time;

		// init subscription
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;

		// init publisher
		rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
		rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr inertial;
		
		// init timer for odom
		rclcpp::TimerBase::SharedPtr odom_timer;

		// init timer for inertial
		rclcpp::TimerBase::SharedPtr inertial_timer;

		// now position
		double N_position_x = 0;
		double N_position_y = 0;
		double N_orientation_theta = 0;

		// old position
		double O_position_x = 0;
		double O_position_y = 0;
		double O_orientation_theta = 0;

		// seconds
		double millisec;

		// check Wheel Drop
		void checkWheelDrop();

		// translate_coordinate
        geometry_msgs::msg::Quaternion translateCoordinate(double x, double y, double z);

		// calculate velocity
		double calculateVelocity(double now_velocity, double old_velocity, float time);

		// controle that based on velocity
		void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

		// publish odometry
		void publishOdometry();

		// publish inertial
		void publishInertial();

		// Quality of Service
		rmw_qos_profile_t odom_qos_profile = rmw_qos_profile_sensor_data;
		rmw_qos_profile_t imu_qos_profile = rmw_qos_profile_sensor_data;
		rmw_qos_profile_t cmd_vel_qos_profile = rmw_qos_profile_sensor_data;

	public :
		Turtlebot() :
			Node("Turtlebot") {
				cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
					"cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						controleByVelocity(msg);
					},
					cmd_vel_qos_profile
				);

				odom = this->create_publisher<nav_msgs::msg::Odometry>("odom", odom_qos_profile);
				odom_timer = this->create_wall_timer(20ms, bind(&Turtlebot::publishOdometry, this));
				
				inertial = this->create_publisher<sensor_msgs::msg::Imu>("imu", imu_qos_profile);
				inertial_timer = this->create_wall_timer(20ms, bind(&Turtlebot::publishInertial, this));
			}
};
