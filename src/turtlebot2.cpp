#include <iostream>
#include <chrono>

#include <libkobuki.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using namespace rt_net;
using namespace std;
using namespace chrono_literals;

class Turtlebot : public rclcpp::Node {
	private :
		// kobuki device special
		const char* device_special = "/dev/kobuki";
		// velocity
		double linear_velocity = 0;
		double angular_velocity = 0;

		// subscriber
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;
		// publisher
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
		rclcpp::TimerBase::SharedPtr timer;

		// init kobuki
		Kobuki *kobuki;

		// controle by velocity
		void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {
			this->kobuki->setTargetVelocity(msg->linear.x, msg->angular.z);
		};


	public :
		Turtlebot() :
			Node("Turtlebot") {
				cmd_vel = this->create_subscription <geometry_msgs::msg::Twist> (
					"/cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						this->controleByVelocity(msg);
					}
				);
			};
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
