#include "can_transceiver_2/can_transceiver.hpp"
#include <sys/ioctl.h>  
#include <fcntl.h> 

namespace cantransceiver

{

CanTransceiver::CanTransceiver() 
: Node("can_transceiver") {
    std::system("ip link set can1 up type can bitrate 1000000 restart-ms 1000");

    socket_fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (socket_fd < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to create CAN socket");
        rclcpp::shutdown();
        return;
    }

    std::strncpy(ifr.ifr_name, "can1", IFNAMSIZ - 1);
    ioctl(socket_fd, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;

    if (bind(socket_fd, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
        RCLCPP_FATAL(this->get_logger(), "Failed to bind CAN socket");
        rclcpp::shutdown();
        return;
    }

    pub= this->create_publisher<std_msgs::msg::UInt8MultiArray>("can_rx", 10);
    sub= this->create_subscription<std_msgs::msg::UInt8MultiArray>(
       "can_tx", 10, std::bind(&CanTransceiver::callback, this, std::placeholders::_1));

    timer= this->create_wall_timer(std::chrono::milliseconds(2), std::bind(&CanTransceiver::spin, this));

    RCLCPP_INFO(this->get_logger(), "CAN Started!");
}

CanTransceiver::~CanTransceiver() {
    shutdown_can_interface();
    RCLCPP_INFO(this->get_logger(), "CAN Killed!");
}

void CanTransceiver::callback(const std_msgs::msg::UInt8MultiArray::SharedPtr msg) {
    struct can_frame send_frame;
    send_frame.can_id = msg->data[0];
    send_frame.can_dlc = std::min(static_cast<size_t>(8), msg->data.size() - 1);

    std::copy(msg->data.begin() + 1, msg->data.begin() + 1 + send_frame.can_dlc, send_frame.data);

    // Отправка CAN сообщения
    write(socket_fd, &send_frame, sizeof(send_frame));
}

void CanTransceiver::spin() {
    struct can_frame recv_frame;
    int nbytes = read(socket_fd, &recv_frame, sizeof(struct can_frame));

    if (nbytes > 0) {
        auto arr = std_msgs::msg::UInt8MultiArray();
        arr.data.push_back(recv_frame.can_id);
        arr.data.insert(arr.data.end(), recv_frame.data, recv_frame.data + recv_frame.can_dlc);
        pub->publish(arr);
    }
}

void CanTransceiver::shutdown_can_interface() {
    std::system("/sbin/ifconfig can1 down");
}
}

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cantransceiver::CanTransceiver>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}