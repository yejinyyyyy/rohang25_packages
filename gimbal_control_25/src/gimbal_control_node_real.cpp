#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <chrono>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <unistd.h>
#include <cmath>
#include <vector>
#include <sstream>
#include <iomanip>
#include <geometry_msgs/msg/point32.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>
#include <yolov11_msgs/msg/detection.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <Eigen/Dense>

#include <stdint.h>
#include <cmath>

#include <chrono>

#include <array>
// ==============================
// ZR10 짐벌 통신 관련 설정
// ==============================
#define RECV_BUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.144.25" // ZR10 Gimbal Default IP

using namespace px4_msgs::msg;
using namespace std::chrono;
using namespace std::chrono_literals;


class GimbalControlNode : public rclcpp::Node {
public:
    

    GimbalControlNode() : Node("gimbal_control_node") 
    {   
        auto qos = rclcpp::SensorDataQoS();
        RCLCPP_INFO(this->get_logger(), "Gimbal Control Node 시작됨");
        caminfo_sub_ = create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/camera_info", qos,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
              fx_ = msg->k[0];
              fy_ = msg->k[4];
              cx= msg->k[2];
              cy= msg->k[5];
              img_w_ = msg->width;
              img_h_ = msg->height;
              //h_fov_ = 2.0f * atan2(img_w_/2.0f, fx_)  * 180.0f/M_PI;
              //v_fov_ = 2.0f * atan2((img_h_/2.0f), fy_) * 180.0f /M_PI;
              camera_info_ready_ = true;
            
            });
        det_sub_ = create_subscription<yolov11_msgs::msg::Detection>(
            "/yolov11/detection", qos,
            [this](const yolov11_msgs::msg::Detection::SharedPtr msg) {
              det_center_x_ = msg->center_x;
              det_center_y_ = msg->center_y;
              detection_ready_ = true;
            });
        
        timer_ = this->create_wall_timer(500ms, std::bind(&GimbalControlNode::control_loop, this));

    }


private:

	rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr tvec_sub_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr caminfo_sub_;
    rclcpp::Subscription<yolov11_msgs::msg::Detection>::SharedPtr det_sub_;
    rclcpp::TimerBase::SharedPtr timer_;	
    
    uint8_t yaw_hex[2] = {0, 0}; 
    uint8_t pitch_hex[2] = {0, 0};
    float yaw_deg;
    float pitch_deg;
    float fx_, fy_;
    float cx, cy;
    int img_w_, img_h_;
    //float h_fov_, v_fov_;
    bool camera_info_ready_;
    float det_center_x_, det_center_y_;
    bool detection_ready_;
    static constexpr double  DEG_DEAD_BAND = 5.0;  // [°] 데드밴드 문턱
    double last_pan_deg_   = 0.0;
    double last_tilt_deg_ = 0.0;

    // CRC16 테이블
    // =====================
    const uint16_t crc16_tab[256]= {0x0,0x1021,0x2042,0x3063,0x4084,0x50a5,0x60c6,0x70e7,
        0x8108,0x9129,0xa14a,0xb16b,0xc18c,0xd1ad,0xe1ce,0xf1ef,
        0x1231,0x210,0x3273,0x2252,0x52b5,0x4294,0x72f7,0x62d6,
        0x9339,0x8318,0xb37b,0xa35a,0xd3bd,0xc39c,0xf3ff,0xe3de,
        0x2462,0x3443,0x420,0x1401,0x64e6,0x74c7,0x44a4,0x5485,
        0xa56a,0xb54b,0x8528,0x9509,0xe5ee,0xf5cf,0xc5ac,0xd58d,
        0x3653,0x2672,0x1611,0x630,0x76d7,0x66f6,0x5695,0x46b4,
        0xb75b,0xa77a,0x9719,0x8738,0xf7df,0xe7fe,0xd79d,0xc7bc,
        0x48c4,0x58e5,0x6886,0x78a7,0x840,0x1861,0x2802,0x3823,
        0xc9cc,0xd9ed,0xe98e,0xf9af,0x8948,0x9969,0xa90a,0xb92b,
        0x5af5,0x4ad4,0x7ab7,0x6a96,0x1a71,0xa50,0x3a33,0x2a12,
        0xdbfd,0xcbdc,0xfbbf,0xeb9e,0x9b79,0x8b58,0xbb3b,0xab1a,
        0x6ca6,0x7c87,0x4ce4,0x5cc5,0x2c22,0x3c03,0xc60,0x1c41,
        0xedae,0xfd8f,0xcdec,0xddcd,0xad2a,0xbd0b,0x8d68,0x9d49,
        0x7e97,0x6eb6,0x5ed5,0x4ef4,0x3e13,0x2e32,0x1e51,0xe70,
        0xff9f,0xefbe,0xdfdd,0xcffc,0xbf1b,0xaf3a,0x9f59,0x8f78,
        0x9188,0x81a9,0xb1ca,0xa1eb,0xd10c,0xc12d,0xf14e,0xe16f,
        0x1080,0xa1,0x30c2,0x20e3,0x5004,0x4025,0x7046,0x6067,
        0x83b9,0x9398,0xa3fb,0xb3da,0xc33d,0xd31c,0xe37f,0xf35e,
        0x2b1,0x1290,0x22f3,0x32d2,0x4235,0x5214,0x6277,0x7256,
        0xb5ea,0xa5cb,0x95a8,0x8589,0xf56e,0xe54f,0xd52c,0xc50d,
        0x34e2,0x24c3,0x14a0,0x481,0x7466,0x6447,0x5424,0x4405,
        0xa7db,0xb7fa,0x8799,0x97b8,0xe75f,0xf77e,0xc71d,0xd73c,
        0x26d3,0x36f2,0x691,0x16b0,0x6657,0x7676,0x4615,0x5634,
        0xd94c,0xc96d,0xf90e,0xe92f,0x99c8,0x89e9,0xb98a,0xa9ab,
        0x5844,0x4865,0x7806,0x6827,0x18c0,0x8e1,0x3882,0x28a3,
        0xcb7d,0xdb5c,0xeb3f,0xfb1e,0x8bf9,0x9bd8,0xabbb,0xbb9a,
        0x4a75,0x5a54,0x6a37,0x7a16,0xaf1,0x1ad0,0x2ab3,0x3a92,
        0xfd2e,0xed0f,0xdd6c,0xcd4d,0xbdaa,0xad8b,0x9de8,0x8dc9,
        0x7c26,0x6c07,0x5c64,0x4c45,0x3ca2,0x2c83,0x1ce0,0xcc1,
        0xef1f,0xff3e,0xcf5d,0xdf7c,0xaf9b,0xbfba,0x8fd9,0x9ff8,
        0x6e17,0x7e36,0x4e55,0x5e74,0x2e93,0x3eb2,0xed1,0x1ef0
        };

    // =====================
    // CRC 계산 함수
    // =====================
    uint16_t CRC16_cal(uint8_t *ptr, uint32_t len, uint16_t crc_init) {
        uint16_t crc, oldcrc16;
        uint8_t temp;
        crc = crc_init;
        while (len--!=0) {
            temp = (crc >> 8) & 0xff;
            oldcrc16 = crc16_tab[*ptr ^ temp];
            crc = (crc << 8) ^ oldcrc16;
            ptr++;
        }
        return crc;
    }

    // =====================
    // 요, 피치 → 16진수 변환
    // =====================
    void encode_attitude(double yaw_deg, double pitch_deg, uint8_t* out_yaw, uint8_t* out_pitch) {
        int16_t val_yaw = (int16_t)(yaw_deg * 10);       // 0.1 deg 단위로 변환
        int16_t val_pitch = (int16_t)(pitch_deg * 10);

        out_yaw[0] = val_yaw & 0xFF;         //LSB
        out_yaw[1] = (val_yaw >> 8) & 0xFF;  //MSB

        out_pitch[0] = val_pitch & 0xFF;         //LSB
        out_pitch[1] = (val_pitch >> 8) & 0xFF;  //MSB
    }

    // =====================
    // 메인 제어 함수
    // =====================
    void control_loop() {
        if (!camera_info_ready_ || !detection_ready_) return;
        detection_ready_ = false;
      
        // [1] YOLO 검출 결과 (640x640 기준)을 1280x720 기준으로 환산
    float u_yolo = det_center_x_;
    float v_yolo = det_center_y_;

    float u_real = u_yolo * ((float)img_w_ / 640.0f);
    float v_real = v_yolo * ((float)img_h_ / 384.0f);

    // [2] 카메라 기준 상대 위치 (Z는 가정 — 예: 2.0m 전방)
    float Z = 1.0f;  // 실제 거리 대신 고정값 가정
    float X = (u_real - cx) * Z / fx_;
    float Y = (v_real - cy) * Z / fy_;

    float norm = sqrt(X*X + Y*Y + Z*Z);
    X = X / norm;
    Y = Y / norm;
    Z = Z / norm;

    float pan_rad = last_pan_deg_ * M_PI / 180.0;
    float tilt_rad = last_tilt_deg_ * M_PI / 180.0;
    // 회전 행렬 구성 (팬: Z축, 틸트: X축)
    Eigen::Matrix3f R_pan;
    R_pan << std::cos(pan_rad), 0, std::sin(pan_rad),
         0,1,  0,
         -std::sin(pan_rad), 0, std::cos(pan_rad);

    Eigen::Matrix3f R_tilt;
    R_tilt << 1, 0, 0,
          0, std::cos(tilt_rad), -std::sin(tilt_rad),
          0, std::sin(tilt_rad),  std::cos(tilt_rad);
    
    
    Eigen::Vector3f v_cam(X, Y, Z);
    Eigen::Vector3f v_world = R_pan * R_tilt * v_cam;

    // [4] 새 팬/틸트 각도 계산 (Z 기준 거리 가정)
    float new_pan_deg   = -(std::atan2(v_world[0], v_world[2]) * 180.0f / M_PI);
    float new_tilt_deg  = -(std::atan2(v_world[1], std::sqrt(v_world[0]*v_world[0] + v_world[2]*v_world[2])) * 180.0f / M_PI);

    // [5] 데드밴드 적용
    if (std::fabs(new_pan_deg - last_pan_deg_) < DEG_DEAD_BAND &&
        std::fabs(new_tilt_deg - last_tilt_deg_) < DEG_DEAD_BAND)
        return;

    // [6] 누적 업데이트 및 전송
    double calculated_target_pan_deg  = new_pan_deg;
    double calculated_target_tilt_deg = new_tilt_deg;


    last_pan_deg_  = 0.5 * last_pan_deg_ + 0.5 * calculated_target_pan_deg;
    last_tilt_deg_ = 0.5 * last_tilt_deg_ + 0.5 * calculated_target_tilt_deg;
    encode_attitude(last_pan_deg_, last_tilt_deg_, yaw_hex, pitch_hex);
    send_gimbal_command();

    }
    


    void send_gimbal_command() {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) { RCLCPP_ERROR(get_logger(), "소켓 생성 실패"); return; }
    
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(SERVER_PORT);
    addr.sin_addr.s_addr = inet_addr(SERVER_IP);
    
    uint8_t buf[14] = {
          0x55,0x66,0x01,0x04, 0x00,0x00,0x00,0x0e,
          yaw_hex[0], yaw_hex[1],
          pitch_hex[0], pitch_hex[1], 0x00,0x00
    };
    
    uint16_t crc = CRC16_cal(buf, 12, 0);
    buf[12] = crc & 0xFF;
    buf[13] = (crc >> 8) & 0xFF;
    
    sendto(sock, buf, sizeof(buf), 0, (sockaddr*)&addr, sizeof(addr));
    close(sock);
    
    RCLCPP_INFO(get_logger(),
      "📐 Sent yaw=%.1f°, pitch=%.1f° (CRC=0x%04X)",
      last_pan_deg_, last_tilt_deg_, crc);
      }
};



int main(int argc, char * argv[]) {
    
    setenv("ROS_LOG_DIR", "/home/jaehwan/gimbal_logs", 1);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalControlNode>());
    rclcpp::shutdown();
    return 0;
}
