#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;

auto twist = geometry_msgs::msg::Twist();

void callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
	twist.linear.x = msg->axes[1]/2;
	twist.angular.z = msg->axes[0];
	//cmd_vel_topic.publish(twist);
}

int main(int argc, char *argv[]) {

	rclcpp::init(argc, argv);
	auto node = rclcpp::Node::make_shared("teleop");
	
	auto cmd_vel_topic = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel");

	// init topic to subscribe joy topic
	auto joy_topic = node->create_subscription<sensor_msgs::msg::Joy>("joy",callback);

	while (rclcpp::ok()) {
		cout << twist.linear.x << "\t" << twist.angular.z << endl;
		cmd_vel_topic->publish(twist);

		rclcpp::spin_some(node);
	}

	rclcpp::shutdown();

	return 0;
}


