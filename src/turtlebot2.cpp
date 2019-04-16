#include <iostream>
#include <chrono>

#include <libkobuki.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>


using std::placeholders::_1;
using namespace rt_net;
using namespace std;

class Turtlebot : public rclcpp::Node {
	public : Turtlebot()
		 : Node("Turtlebot") {
			// Kobuki device special
			const char* deviceSpecial = "/dev/kobuki";
			// init kobuki
			kobuki = createKobuki(KobukiStringArgument(deviceSpecial));

			// subscriber cmd_vel
			_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
					"/cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						this->controleByVelocity(msg);
					}
					//bind(&Turtlebot::controleByVelocity, this, _1)
					);
			// publishger
			// [TODO] create topic to publish odometry
			// _timer = this->create_publisher
			
			// [TODO] create topic to publish battey voltage residual quantity data.

		 }

	private:
		 // init velocity;
		 // linear must use 'm_per_sec'
		 // angular must use 'rad_per_sec'
		 double linearVelocity = 0;
		 double angularVelocity = 0;

		 // counter
		 double counter = 0.0;

		 // init _cmd_vel
		 rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel;
		 // init kobuki
		 Kobuki *kobuki;

		 // controle the turtlebot2 using velocity data
		 void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {


			 linearVelocity  = msg->linear.x;
			 angularVelocity = msg->angular.z;

			 this->kobuki->setTargetVelocity(linearVelocity, angularVelocity);
		 };
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	
	rclcpp::spin(make_shared<Turtlebot>());

	rclcpp::shutdown();

	return 0;
}
