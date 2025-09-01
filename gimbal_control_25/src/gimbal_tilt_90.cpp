#include "rclcpp/rclcpp.hpp"
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cstring>
#include <iostream>

#define RECV_BUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "172.16.15.224" // ZR10 Gimbal Default IP

class GimbalUdpNode : public rclcpp::Node
{
public:
    GimbalUdpNode() : Node("gimbal_tilt_90")
    {
        RCLCPP_INFO(this->get_logger(), "🔧 Gimbal UDP Node started");

        // 1. Create socket
        sockfd_ = socket(AF_INET, SOCK_DGRAM, 0);
        if (sockfd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ socket creation failed");
            return;
        }

        // 2. Setup destination address
        memset(&send_addr_, 0, sizeof(send_addr_));
        send_addr_.sin_family = AF_INET;
        send_addr_.sin_port = htons(SERVER_PORT);
        send_addr_.sin_addr.s_addr = inet_addr(SERVER_IP);

        // 3. Send command
        RCLCPP_INFO(this->get_logger(), "✅ Sending gimbal center command...");
        socklen_t addr_len = sizeof(send_addr_);
        if (sendto(sockfd_, send_buf_, sizeof(send_buf_), 0,
                   (struct sockaddr *)&send_addr_, addr_len) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ sendto failed");
            close(sockfd_);
            return;
        }

        // 4. Receive response
        int recv_len = recvfrom(sockfd_, recv_buf_, RECV_BUF_SIZE, 0,
                                (struct sockaddr *)&recv_addr_, &addr_len);
        if (recv_len < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ recvfrom failed");
            close(sockfd_);
            return;
        }

        std::ostringstream oss;
        oss << "✅ Received HEX data (" << recv_len << " bytes): ";
        for (int i = 0; i < recv_len; ++i) {
            oss << std::hex << std::uppercase << std::setw(2) << std::setfill('0')
                << static_cast<int>(recv_buf_[i]) << " ";
        }
        RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());

        close(sockfd_);
    }

private:
    int sockfd_;
    struct sockaddr_in send_addr_, recv_addr_;
    unsigned char send_buf_[14] = {
        0x55, 0x66, 0x01, 0x04,
        0x00, 0x00, 0x00, 0x0e,
        0x00, 0x00, 0xff, 0xa6,
        0x3b, 0x11
    };
    unsigned char recv_buf_[RECV_BUF_SIZE] = {0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GimbalUdpNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
