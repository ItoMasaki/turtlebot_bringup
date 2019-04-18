#include <iostream>
#include <chrono>

#include <libkobuki.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace rt_net;
using namespace std;
using namespace chrono_literals;

class Turtlebot : public rclcpp::Node {
	private :
		// kobuki device special
		const char* device_special = "/dev/kobuki";
		
		// init kobuki
		Kobuki *kobuki;

		// max battery voltage
		float max_voltage = 15.3;

		// velocity
		double linear_velocity = 0;
		double angular_velocity = 0;

		// subscriber twist
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;

		// publisher battery status
		rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery;

		// pos_x
		double pos_x;
		double pos_y;
		double pos_th;


		// controle by velocity
		void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {
			kobuki->setTargetVelocity(msg->linear.x, msg->angular.z);

			if (kobuki->isRightWheelDrop() or kobuki->isLeftWheelDrop()) {
				delete kobuki;
			}

			kobuki->getPose(&pos_x, &pos_y, &pos_th);
			cout << pos_x << endl;

		};

	public :
		Turtlebot() :
			Node("Turtlebot") {
				kobuki = createKobuki(KobukiStringArgument(device_special));
				// topic to subscribe twist message
				cmd_vel = this->create_subscription <geometry_msgs::msg::Twist> (
					"/cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						controleByVelocity(msg);
					}
				);
				cout << cmd_vel << endl;
			};
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
