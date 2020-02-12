#include <iostream>
#include <cmath>

#include <Turtlebot.hpp>

#include <sensor_msgs/msg/imu.hpp>

using namespace std;


Turtlebot::Turtlebot() : Node("turtlebot"){
   /*
    * These lines are declaration parameter lines.
    */
    declare_parameter("OdomTransFrameId",       "odom");
    declare_parameter("OdomTransChildFrameId",  "base_link");
    declare_parameter("DeviceSpecial",          "/dev/kobuki");
    declare_parameter("TurtlebotVelocityTopic", "cmd_vel");

   /*
    * kobuki device special
    */
    const string device_special = this->get_parameter("DeviceSpecial").as_string();

   /*
    * set up kobuki for communicating.
    * device_special is device name there may be it in /dev.
    * If you have it, you must set up device special use
    */
    kobuki = createKobuki(KobukiStringArgument(device_special));

   /*
    * run pose reset.
    */
    kobuki->setPose(0.0, 0.0, 0.0);

    odom_trans.header.frame_id = this->get_parameter("OdomTransFrameId").as_string();
    odom_trans.child_frame_id = this->get_parameter("OdomTransChildFrameId").as_string();

    odom_broadcaster = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    const string cmd_vel_topic_name = this->get_parameter("TurtlebotVelocityTopic").as_string();

   /*
    * subscription for command velocity to send kobuki velocity.
    * reveive only Twist structure in geometry_msgs.
    */
    velocity = this->create_subscription<geometry_msgs::msg::Twist>(
        cmd_vel_topic_name,
        10,
        [this](geometry_msgs::msg::Twist::SharedPtr msg)
        {
            getVelocity(msg);
        }
    );

   /*
    * subscription for command reset pose to send kobuki to run reset pose.
    * receive onlu Bool structure in std_msgs.
    */
    reset = this->create_subscription<std_msgs::msg::Bool>(
        "turtlebot2/commands/reset_pose",
	    10,
	    [this](std_msgs::msg::Bool::SharedPtr msg)
        {
	        resetPose(msg);
	    }
    );

   /*
    * publisher for send Odometry message.
    */
    odom = this->create_publisher<nav_msgs::msg::Odometry>(
        "turtlebot2/odometry",
	    10
    );

   /*
    * publisher for sending pushed button.
    */
    button_0 = this->create_publisher<std_msgs::msg::Bool>(
        "turtlebot2/button0",
        10
    );

    button_1 = this->create_publisher<std_msgs::msg::Bool>(
        "turtlebot2/button1",
        10
    );

    button_2 = this->create_publisher<std_msgs::msg::Bool>(
        "turtlebot2/button2",
        10
    );

   /*
    * create timer for subscribe and publish odometry.
    * 20ms means that kobuki can only send data for its rate.
    */
    odometryTimer = this->create_wall_timer(
        20ms,
        bind(&Turtlebot::publishOdometry, this)
    );

   /*
    * imu can't use!!!
    */
    inertial = this->create_publisher<sensor_msgs::msg::Imu>(
        "turtlebot2/imu",
        10
    );

   /*
    * recieve inertial data from kobuki.
    */
    inertialTimer = this->create_wall_timer(
        20ms,
        bind(&Turtlebot::publishInertial, this)
    );

    emergencyTimer = this->create_wall_timer(
        20ms,
        bind(&Turtlebot::getEmergency, this)
    );

    buttonTimer = this->create_wall_timer(
        20ms,
        bind(&Turtlebot::getButtonPush, this)
    );
}


// ホイールが地面から離れたことを検知
// check the wheels on the ground
void Turtlebot::checkWheelDrop(){
    if (kobuki->isRightWheelDrop() || kobuki->isLeftWheelDrop()) {
        delete kobuki;
        RCLCPP_INFO(this->get_logger(), "WHEEL DROP");
        abort();
    }
}

// reset pose to (0, 0, 0)
void Turtlebot::resetPose(std_msgs::msg::Bool::SharedPtr msg){
    if (msg->data == true){
        kobuki->setPose(0, 0, 0);    
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

    if (msg->angular.z >= M_PI/2)
    {
        RCLCPP_INFO(this->get_logger(), "OVER 90.0 [deg/s]");
        Target_Angular_Velocity = M_PI/2;
    }
    else if(msg->angular.z <= -M_PI/2)
    {
        RCLCPP_INFO(this->get_logger(), "OVER -90.0 [deg/s]");
        Target_Angular_Velocity = -M_PI/2;
    }
    else
    {
        Target_Angular_Velocity = msg->angular.z;
    }

    if (msg->linear.x >= 0.9) {
        RCLCPP_INFO(this->get_logger(), "OVER 0.9 [m/s]");
        Target_Linear_Velocity = 0.9;

    } else if(msg->linear.x <= -0.9) {
        RCLCPP_INFO(this->get_logger(), "OVER -0.9 [m/s]");
        Target_Linear_Velocity = -0.9;

    } else {
        Target_Linear_Velocity = msg->linear.x;

    }

    kobuki->setTargetVelocity(Target_Linear_Velocity, Target_Angular_Velocity);
}


// オドメトリのブロードキャスト
void Turtlebot::publishOdometry() {
    checkWheelDrop();

    kobuki->getPose(&N_position_x, &N_position_y, &N_orientation_theta);

    odom_trans.header.stamp = get_clock()->now();
    odom_trans.transform.translation.x = N_position_x;
    odom_trans.transform.translation.y = N_position_y;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = translateCoordinate(0.0, 0.0, N_orientation_theta);
    odom_broadcaster->sendTransform(odom_trans);


    odom_msg.child_frame_id = "base_link";
    odom_msg.header.frame_id = "map";
    odom_msg.header.stamp = get_clock()->now();
    odom_msg.pose.pose.position.x = N_position_x;
    odom_msg.pose.pose.position.y = N_position_y;
    odom_msg.pose.pose.orientation = translateCoordinate(0.0, 0.0, N_orientation_theta);

    N_linear_x_velocity = calculateVelocity(N_position_x, O_position_x, 0.02);
    N_linear_y_velocity = calculateVelocity(N_position_y, O_position_y, 0.02);
    N_linear_z_velocity = calculateVelocity(N_orientation_theta, O_orientation_theta, 0.02);

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

    if (isnan(heading_offset) == true){
        heading_offset = (static_cast<double>(kobuki->getInertialAngle()) / 100.0) * M_PI / 180.0;
    }

    auto imu_msg = sensor_msgs::msg::Imu();

    imu_msg.header.frame_id  = "gyro_link";
    imu_msg.header.stamp     = get_clock()->now();

    heading = (static_cast<double>(kobuki->getInertialAngle()) / 100.0) * M_PI / 180.0;
    head_angle = heading - heading_offset;

    imu_msg.orientation = translateCoordinate(0, 0, head_angle);

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

void Turtlebot::getEmergency() {
    if (kobuki->getAnalogIn(0) < 3.0 && N_position_x != 0) {
        delete kobuki;
        RCLCPP_INFO(this->get_logger(), "Pushed Emergency Botton");
        abort();
    }
}

void Turtlebot::getButtonPush() {
    bool_msg.data = kobuki->isButton0();
    button_0->publish(bool_msg);
    bool_msg.data = kobuki->isButton1();
    button_1->publish(bool_msg);
    bool_msg.data = kobuki->isButton2();
    button_2->publish(bool_msg);

    // add button publish
}

