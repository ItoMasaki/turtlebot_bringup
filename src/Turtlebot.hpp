#include <libkobuki.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>
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
		float max_voltage = 15.3;

		// init chrono timer
		chrono::system_clock::time_point base_time;
		chrono::system_clock::time_point now_time;

		// init subscription
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;

		// init publisher
		rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery;
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
		
		// init timer for odom
		rclcpp::TimerBase::SharedPtr odom_timer;

		// now position
		double N_position_x;
		double N_position_y;
		double N_orientation_theta;

		// old position
		double O_position_x = 0;
		double O_position_y = 0;
		double O_orientation_theta = 0;

		// velocity
		double velocity_x;
		double velocity_y;
		double velocity_theta;

		// seconds
		double millisec;

		// check Wheel Drop
		bool checkWheelDrop();

		// controle that based on velocity
		void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg);

		// publish odometry
		void publishOdometry();

	public :
		Turtlebot() :
			Node("Turtlebot") {}
};
