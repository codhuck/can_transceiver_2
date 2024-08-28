#pragma once

#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/u_int8_multi_array.hpp>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <sys/socket.h>
#include <net/if.h>
#include <unistd.h>
#include <cstring>
#include <fcntl.h>
#include <iostream>
#include <chrono>
#include <thread>
#include <cstdlib>

namespace cantransceiver{
class CanTransceiver : public rclcpp::Node {
public:
    CanTransceiver();
    ~CanTransceiver();

private:
    void callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg);
    void spin();
    void shutdown_can_interface();

    rclcpp::Publisher<std_msgs::msg::UInt8MultiArray>::SharedPtr pub;
    rclcpp::Subscription<std_msgs::msg::UInt8MultiArray>::SharedPtr sub;
    rclcpp::TimerBase::SharedPtr timer;

    int socket_fd;
    struct sockaddr_can addr;
    struct ifreq ifr;
};
}