#include <iostream>
#include <stdlib.h>

#include <Turtlebot.hpp>

using namespace std;

void Turtlebot::checkWheelDrop(){
	if (kobuki->isRightWheelDrop() || kobuki->isLeftWheelDrop()) {
		delete kobuki;
		cout << "[!] Error : Wheel Drop" << endl;
		abort();
	}
}

void Turtlebot::translateCoordinate(double x, double y, double z){
	cout << x << y << z << endl;
}

void Turtlebot::controleByVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {
	checkWheelDrop();
	kobuki->setTargetVelocity(msg->linear.x, msg->angular.z);
}

void Turtlebot::publishOdometry() {
	checkWheelDrop();
    auto odom_msg = nav_msgs::msg::Odometry();

    kobuki->getPose(&N_position_x, &N_position_y, &N_orientation_theta);

    // calculate delata time
    now_time = chrono::system_clock::now();
    auto delta_seconds = chrono::duration_cast<chrono::seconds>(now_time - base_time);
    auto delta_milliseconds = chrono::duration_cast<chrono::milliseconds>(now_time - base_time);

    millisec = delta_milliseconds.count() - delta_seconds.count()*1000;

    odom_msg.child_frame_id = "base_footprint";
    odom_msg.header.frame_id = "odom";
    odom_msg.header.stamp.sec = delta_seconds.count();
    odom_msg.header.stamp.nanosec = millisec;
    odom_msg.pose.pose.position.x = N_position_x;
    odom_msg.pose.pose.position.y = N_position_y;
    odom_msg.pose.pose.orientation.z = N_orientation_theta;

	odom_msg.twist.twist.linear.x = calculateVelocity(N_position_x, O_position_x, 0.02);
	odom_msg.twist.twist.linear.y = calculateVelocity(N_position_y, O_position_y, 0.02);

    translateCoordinate(1.0, 2.0, 3.0);
	odom_msg.twist.twist.angular.z = calculateVelocity(N_orientation_theta, O_orientation_theta, 0.015);

    O_position_x = N_position_x;
    O_position_y = N_position_y;
    O_orientation_theta = N_orientation_theta;

    odom->publish(odom_msg);
}

double Turtlebot::calculateVelocity(double now_velocity, double old_velocity, float time){
	return (old_velocity - now_velocity)/time;
}

void Turtlebot::publishInertial() {
	cout << kobuki->getInertialAngle() << endl;
}
