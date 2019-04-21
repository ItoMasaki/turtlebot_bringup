#include <iostream>
#include <stdlib.h>
#include <chrono>

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
		const char* device_special = "/dev/kobuki";

		// init kobuki
		Kobuki *kobuki;

		// max battery voltage
		float max_voltage = 15.3;

		// chrono timer
		chrono::system_clock::time_point base_time = chrono::system_clock::now();
		chrono::system_clock::time_point now_time;

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
		double now_position_x;
		double now_position_y;
		double now_orientation_theta;

		double old_position_x = 0;
		double old_position_y = 0;
		double old_orientation_theta = 0;
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

				auto odom_msg = nav_msgs::msg::Odometry();

				kobuki->getPose(&now_position_x, &now_position_y, &now_orientation_theta);

				// calculate delata time
				now_time = chrono::system_clock::now();
				auto delta_time = chrono::duration_cast<chrono::milliseconds>(now_time - base_time);
				cout << delta_time.count() << endl;
				// [TODO] millisecond and second

				odom_msg.header.frame_id = "base_link";
				// odom_msg.stamp
				odom_msg.pose.pose.position.x = now_position_x;
				odom_msg.pose.pose.position.y = now_position_y;
				odom_msg.pose.pose.orientation.z = now_orientation_theta;

				// [TODO] calc velocity which is based on odometry

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

				///////////////////////////////////////////////////
				// set timer to call for publishing battery message
				// battery_timer - this->create_wall_timer()

			};
};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
