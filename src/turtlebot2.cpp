#include <iostream>
#include <chrono>

#include <libkobuki.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>


using namespace std;

class Turtlebot : public rclcpp::Node {
	public : Turtlebot()
		 : Node("Turtlebot") {
		 	auto _cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg){
					this->display(msg);
					});
		 };

	private:
		 rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel;

		 void display(geometry_msgs::msg::Twist::SharedPtr msg) {
		 	cout << msg << endl;
		 }
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
