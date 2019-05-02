#include <iostream>
#include <stdlib.h>

#include <Turtlebot.hpp>

using namespace std;

Turtlebot::Turtlebot() : Node("Turtlebot") {

	// topic to subscribe twist message
	cmd_vel = this->create_subscription<geometry_msgs::msg::Twist>(
		"cmd_vel",
		[this](geometry_msgs::msg::Twist::SharedPtr msg) {
			controleByVelocity();
		};

	// set timer to call for publishing odometry message
	odom_timer = this->create_wall_timer(15ms, std::bind(&Turtlebot::publishOdometry, this));
	odom = this->create_publisher("odom");
	
	)
};

void Turtlebot::checkWheelDrop(){
	if (kobuki->isRightWheelDrop() || kobuki->isLeftWheelDrop()) {
		delete kobuki;
		cout << "[!] Error : Wheel Drop" << endl;
		abort();
	}
};

void Turtlebot::controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {
	checkWheelDrop();
};

void Turtlebot::publishOdometry() {
	checkWheelDrop();
    auto odom_msg = nav_msgs::msg::Odometry();

    kobuki->getPose(&now_position_x, &now_position_y, &now_orientation_theta);

    // calculate delata time
    now_time = chrono::system_clock::now();
    auto delta_seconds = chrono::duration_cast<chrono::seconds>(now_time - base_time);
    auto delta_milliseconds = chrono::duration_cast<chrono::milliseconds>(now_time - base_time);

    millisec = delta_milliseconds.count() - delta_seconds.count()*1000;

    odom_msg.child_frame_id = "base_footprint";
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp.sec = delta_seconds.count();
    odom_msg.header.stamp.nanosec = millisec;
    odom_msg.pose.pose.position.x = now_position_x;
    odom_msg.pose.pose.position.y = now_position_y;
    odom_msg.pose.pose.orientation.z = now_orientation_theta;

	odom_msg.twist.twist.linear.x = calculateVelocity(now_position_x, old_position_x, 0.015);
	odom_msg.twist.tiwst.linear.y = calculateVelocity(now_position_y, old_position_y, 0.015);
	odom_msg.twist.twist.angular.z = calculateVelocity(now_orientation_theta, old_orientation_theta, 0.015);

    //velocity_x = (now_position_x - old_position_x)/0.015;
    //velocity_y = (now_position_y - old_position_y)/0.015;
    //velocity_theta = (now_orientation_theta - old_orientation_theta)/0.015;

    cout << velocity_x << endl;

    old_position_x = now_position_x;
    old_position_y = now_position_y;
    old_orientation_theta = now_orientation_theta;

    pub_odom->publish(odom_msg);
};

double Turtlebot::calculateVelocity(double now_velocity, double old_velocity, float time){
	return (old_velocity - now_velocity)/time;
};

