#include <iostream>
#include <chrono>

#include <libkobuki.h>

#include <rclcpp/rclcpp.hpp>

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>

using std::placeholders::_1;
using namespace rt_net;
using namespace std;
using namespace chrono_literals;

class Turtlebot : public rclcpp::Node {
	public : Turtlebot()
		: Node("Turtlebot") {
			// Kobuki device special
			const char* deviceSpecial = "/dev/kobuki";

			// init kobuki
			// kobuki = createKobuki(KobukiStringArgument(deviceSpecial));

			// subscriber cmd_vel
			_cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
					"/cmd_vel",
					[this](geometry_msgs::msg::Twist::SharedPtr msg) {
						this->controleByVelocity(msg);
						}
					);

			// timer
			// timer_ = create_wall_timer(5ms, publishOdometry);
			// publishger
			// [TODO] create topic to publish odometry
			_odom = this->create_publisher<nav_msgs::msg::Odometry>("/odom");

			// [TODO] create topic to publish battey voltage residual quantity data.
			
			// [TODO] topic to publish time (timer)

			// [TODO] recognize wheel up error

		    // controle the turtlebot2 using velocity data
        	void controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {

            	linearVelocity  = msg->linear.x;
                angularVelocity = msg->angular.z;

				cout << this->kobuki->getBatteryVoltage() << endl;

                //this->kobuki->setTargetVelocity(linearVelocity, angularVelocity);
            };

	private:
		// init velocity;
		// linear must use [m/s]
		// angular must use [rad/s]
		double linearVelocity = 0;
		double angularVelocity = 0;

		// init _cmd_vel
		rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr _cmd_vel;
		// init _odom
		rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr _odom;
		// init timer
		rclcpp::TimerBase::SharedPtr timer_;
		// init kobuki
		Kobuki *kobuki;

};


int main(int argc, char *argv[]) {
	rclcpp::init(argc, argv);
	rclcpp::spin(make_shared<Turtlebot>());
	rclcpp::shutdown();

	return 0;
}
