#include <iostream>
#include <chrono>

#include <libkobuki.h>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>


using namespace rt_net;
using namespace std::chrono_literals;

int main(int argc, char **argv) {

        rclcpp::init(argc, argv);

        auto node = rclcpp::Node::make_shared("greeter");
        auto publisher = node->create_publisher<std_msgs::msg::String>("greeting");

        while (rclcpp::ok()) {


        }

        rclcpp::shutdown();

        Kobuki* kobuki = createKobuki(KobukiStringArgument("/dev/ttyUSB1"));

        delete kobuki;
        return 0;
}
