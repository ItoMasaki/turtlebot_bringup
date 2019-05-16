#include "Turtlebot.hpp"

int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);

	auto node = make_shared<Turtlebot>();
	RCLCPP_INFO(node->get_logger(), "START TURTLEBOT.");
	
	rclcpp::spin(node);
	rclcpp::shutdown();

	return 0;
}
