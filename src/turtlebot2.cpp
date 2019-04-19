#include <iostream>
#include <stdlib.h>
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

		///////////////////
		// subscriber twist
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel;

		///////////////////////////
		// publisher battery status
		rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery;

		/////////////////////////////////////
		// publisher and timer about odometry
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_odom;
		rclcpp::TimerBase::SharedPtr odom_timer;

		///////////
		// messages
		nav_msgs::msg::Odometry::SharedPtr odom_msg;

		/////////////
		// postition
		double position_x;
		double position_y;
		double orientation_theata;
		////////////
		// velocity
		double velocity_x;
		double velocity_y;
		double velocity_rad;

		// controle by velocity
		void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {
			kobuki->setTargetVelocity(msg->linear.x, msg->angular.z);

		};

		// publishOdometry
		void publishOdometry() {
			// handle error
            if (kobuki->isRightWheelDrop() or kobuki->isLeftWheelDrop()) {

                delete kobuki;
                cout << "[!] Error : Wheel Drop" << endl;
                abort();

			} else {

				kobuki->getPose(&position_x, &position_y, &orientation_theata);
				auto odom_msg = nav_msgs::msg::Odometry();
				odom_msg.pose.pose.position.x = position_x;
				if (position_x >= 1.000) {
					delete kobuki;
				}
				odom_msg.pose.pose.position.y = position_y;
				odom_msg.pose.pose.orientation.z = orientation_theata;

				cout << kobuki->getLeftMotorCurrent() << "\tA" << '\t' << kobuki->getLeftMotorEncoder() << endl;

				pub_odom->publish(odom_msg);

			}
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
				odom_timer = this->create_wall_timer(15ms, std::bind(&Turtlebot::publishOdometry, this));
				pub_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom");

			};
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
