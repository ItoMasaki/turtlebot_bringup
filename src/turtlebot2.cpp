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

		// publisher and timer about odometry
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom;
		rclcpp::TimerBase::SharedPtr odom_timer;

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
		};

		// publishOdometry
		void publishOdometry() {
			kobuki->getPose(&pos_x, &pos_y, &pos_th);

			cout << "x : " << pos_x << "\ty : " << pos_y << "\tth : " << pos_th << endl;
		};

	public :
		Turtlebot() :
			Node("Turtlebot") {
				//////////////
				// init kobuki
				kobuki = createKobuki(KobukiStringArgument(device_special));

				///////////////////////////////////
				// topic to subscribe twist message
				cmd_vel = this->create_subscription <geometry_msgs::msg::Twist> (
					"/cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						controleByVelocity(msg);
					}
				);

				////////////////////////////////////////////////////
				// set timer to call for publishing odometry message
				odom_timer = this->create_wall_timer(10ms, std::bind(&Turtlebot::publishOdometry, this));

			};
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
