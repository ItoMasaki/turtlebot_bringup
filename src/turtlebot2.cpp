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
			// kobuki = createKobuki(KobukiStringArgument(deviceSpecial));

			cout << "[*] Bring uped topic name" << endl;

			// subscriber cmd_vel
			_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
					"/cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						this->controleByVelocity(msg);
						}
					);

			if (_cmd_vel != nullptr) {
				cout << "  |- " << _cmd_vel->get_topic_name() << endl;
			};

			// publishger
			// [TODO] create topic to publish odometry
			// _timer = this->create_publisher
			
			// [TODO] create topic to publish battey voltage residual quantity data.
			
			// [TODO] topic to publish time (timer)

			// [TODO] recognize wheel up error

		 }

	private:
		 // init _cmd_vel
		 rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel;

		 // init kobuki
		 Kobuki *kobuki;

		 double count = 0;

		 // controle the turtlebot2 using velocity data
		 void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg){
			count += 0.01;

			cout << msg->linear.x << "\t" << count << endl;

			// this->kobuki->setTargetVelocity(msg->linear.x, msg->angular.z);
		 };
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
