#include <iostream>
#include <cmath>

#include <Turtlebot.hpp>

#include <sensor_msgs/msg/imu.hpp>

using namespace std;


Turtlebot::Turtlebot() : Node("Turtlebot"){
    kobuki = createKobuki(KobukiStringArgument(device_special));

    kobuki->setPose(0.0, 0.0, 0.0);    

    velocity = this->create_subscription<geometry_msgs::msg::Twist>(
        "turtlebot2/commands/velocity",
        [this](geometry_msgs::msg::Twist::SharedPtr msg) {
            getVelocity(msg);
        },
        sensor_qos_profile
    );

    odom = this->create_publisher<nav_msgs::msg::Odometry>("turtlebot2/odometry", sensor_qos_profile);
    odometryTimer = this->create_wall_timer(20ms, bind(&Turtlebot::publishOdometry, this));

    inertial = this->create_publisher<sensor_msgs::msg::Imu>("turtlebot2/imu", sensor_qos_profile);
    inertialTimer = this->create_wall_timer(20ms, bind(&Turtlebot::publishInertial, this));
}


// ホイールが地面から離れたことを検知
void Turtlebot::checkWheelDrop(){
    if (kobuki->isRightWheelDrop() || kobuki->isLeftWheelDrop()) {
        delete kobuki;
        RCLCPP_INFO(this->get_logger(), "WHEEL DROP");
        abort();
    }
}


// オイラー角からクオータニオンへ変換
geometry_msgs::msg::Quaternion Turtlebot::translateCoordinate(double x, double y, double z){
    auto quaternion = geometry_msgs::msg::Quaternion();
    quaternion.w = cos(x/2.0)*cos(y/2.0)*cos(z/2.0) + sin(x/2.0)*sin(y/2.0)*sin(z/2.0);
    quaternion.x = sin(x/2.0)*cos(y/2.0)*cos(z/2.0) - cos(x/2.0)*sin(y/2.0)*sin(z/2.0);
    quaternion.y = cos(x/2.0)*sin(y/2.0)*cos(z/2.0) + sin(x/2.0)*cos(y/2.0)*sin(z/2.0);
    quaternion.z = cos(x/2.0)*cos(y/2.0)*sin(z/2.0) - sin(x/2.0)*sin(y/2.0)*cos(z/2.0);

    return quaternion;
}

// get velocity of geometry
void Turtlebot::getVelocity(geometry_msgs::msg::Twist::SharedPtr msg) {
    checkWheelDrop();

    if (msg->angular.z >= 110) {
        RCLCPP_INFO(this->get_logger(), "OVER 110.0 [deg/s]");
        Target_Angular_Velocity = M_PI*11/18;

    } else if(msg->angular.z <= -110) {
        RCLCPP_INFO(this->get_logger(), "OVER -110.0 [deg/s]");
        Target_Angular_Velocity = -M_PI*11/18;

    } else {
        Target_Angular_Velocity = M_PI*msg->angular.z/180;

    }

    cout << System_Angular_Velocity << endl;

    kobuki->setTargetVelocity(0, Target_Angular_Velocity);
}


// オドメトリのブロードキャスト
void Turtlebot::publishOdometry() {
    checkWheelDrop();

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
    odom_msg.pose.pose.orientation = translateCoordinate(0.0, 0.0, N_orientation_theta);

    N_linear_x_velocity = ((N_position_x - O_position_x)*cos(O_orientation_theta) + (N_position_y - O_position_y)*sin(O_orientation_theta))/0.02;
    N_linear_y_velocity = ((O_position_x - N_position_x)*cos(O_orientation_theta) + (N_position_y - O_position_y)*sin(O_orientation_theta))/0.02;
    N_linear_z_velocity = (N_orientation_theta - O_orientation_theta)/0.02;

    odom_msg.twist.twist.linear.x = N_linear_x_velocity;
    odom_msg.twist.twist.linear.y = N_linear_y_velocity;
    odom_msg.twist.twist.angular.z = N_linear_z_velocity;

    O_position_x = N_position_x;
    O_position_y = N_position_y;
    O_orientation_theta = N_orientation_theta;

    odom->publish(odom_msg);
}


// 速度計算
double Turtlebot::calculateVelocity(double N_position, double O_position, float time){
    return (O_position - N_position)/time;
}


// 回転慣性値のブロードキャスト
void Turtlebot::publishInertial() {
    auto imu_msg = sensor_msgs::msg::Imu();

    now_time = chrono::system_clock::now();
    auto delta_seconds = chrono::duration_cast<chrono::seconds>(now_time - base_time);
    auto delta_milliseconds = chrono::duration_cast<chrono::milliseconds>(now_time - base_time);

    millisec = delta_milliseconds.count() - delta_seconds.count()*1000;

    imu_msg.header.frame_id      = "imu";
    imu_msg.header.stamp.sec     = delta_seconds.count();
    imu_msg.header.stamp.nanosec = millisec;

    imu_msg.orientation = translateCoordinate(0, 0, kobuki->getInertialAngle());

    imu_msg.angular_velocity.x = 0.0;
    imu_msg.angular_velocity.y = 0.0;

    double InertialAngleRate = kobuki->getInertialAngleRate()/60000;
    bool flag = (int)InertialAngleRate;

    //if ( flag ) {
    //  cout << InertialAngleRate << endl;
    //} else {
    //  cout << -InertialAngleRate << endl;
    //}

    imu_msg.angular_velocity.z = kobuki->getInertialAngleRate();

    inertial->publish(imu_msg);
}
