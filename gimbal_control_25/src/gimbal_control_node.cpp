#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
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

// ==============================
// ZR10 짐벌 통신 관련 설정
// ==============================
#define RECV_BUF_SIZE 64
#define SERVER_PORT 37260
#define SERVER_IP "192.168.144.25" // ZR10 Gimbal Default IP

using namespace px4_msgs::msg;


class GimbalControlNode : public rclcpp::Node {
public:

    GimbalControlNode() : Node("gimbal_control_node") 
    {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		qos_profile.depth = 20;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);
        
        
        RCLCPP_INFO(this->get_logger(), "Gimbal Control Node 시작됨");
        
        subscription_object_center_ned= this->create_subscription<geometry_msgs::msg::Point32>( "/object_center_ned", qos, 
            [this](const geometry_msgs::msg::Point32::ConstSharedPtr msg) { this->listener_callback_object_center_ned(msg); }); //지오로케이션 노드에서 퍼블리시하는 topic 받기

        subscription_local_position = this->create_subscription<VehicleLocalPosition>( "/fmu/out/vehicle_local_position", qos,
            [this](const VehicleLocalPosition::ConstSharedPtr msg) { this->listener_callback_local_position(msg); });    
    }


private:


    rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr subscription_object_center_ned;
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_local_position;
    
    float fcc_position_N; float fcc_position_E; float fcc_position_D;    
    geometry_msgs::msg::Point32 object_ned_msg;
    
    void listener_callback_object_center_ned(const geometry_msgs::msg::Point32::ConstSharedPtr msg);
    void listener_callback_local_position(const VehicleLocalPosition::ConstSharedPtr msg);
    // =====================
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
    void encode_attitude(int yaw_deg, int pitch_deg, uint8_t* out_yaw, uint8_t* out_pitch) {
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
    void control_gimbal() {
        int sockfd;
        int recv_len;

        // 0. 주고 받는 곳의 주소
        struct sockaddr_in send_addr, recv_addr;

        // 0. 주고 받는 데이터 저장 배열
        unsigned char recv_buf[RECV_BUF_SIZE] = {0};
        unsigned char att_send_buf[] = {
            0x55, 0x66, 0x01, 0x01,
            0x00, 0x00, 0x00, 0x08,
            0x01, 0xd1, 0x12 // 초기 센터로 정렬
        }; 

        // 1. 통신을 위한 UDP 소켓 생성
        if ((sockfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 소켓 생성 실패");
            return;
        }

        // 2. 보내는 주소 설정
        memset(&send_addr, 0, sizeof(send_addr));
        send_addr.sin_family = AF_INET;
        send_addr.sin_port = htons(SERVER_PORT);
        send_addr.sin_addr.s_addr = inet_addr(SERVER_IP);

        
        // 3. 카메라와 조난자의 NED 좌표 설정
        std::vector<float> target = {object_ned_msg.x, object_ned_msg.y, object_ned_msg.z};  // 조난자 위치
        std::vector<float> origin = {fcc_position_N, fcc_position_E, fcc_position_D};  // 카메라 중심

        // 4. 벡터 차이 계산
        float dx = target[0] - origin[0];
        float dy = target[1] - origin[1];
        float dz = target[2] - origin[2];

        // 5. 요, 피치 계산 (라디안 → 도)
        float pitch_rad = atan2(-dz, sqrt(dx*dx + dy*dy));
        float pitch_deg = pitch_rad * (180.0f / M_PI);
        float yaw_rad = atan2(dy, dx);
        float yaw_deg = yaw_rad * (180.0f / M_PI);

        //Yaw range -135 ~ 135 deg(CCW) , Pitch range -90 ~ 25 deg
        uint8_t pitch_hex[2], yaw_hex[2];
        encode_attitude(yaw_deg, pitch_deg, yaw_hex, pitch_hex);   // input 각도 → hex 변환

        // 6. 요 피치 출력 (디버그용)
        RCLCPP_INFO(this->get_logger(), "Pitch = %.2f, Yaw = %.2f", pitch_deg, yaw_deg);
        RCLCPP_INFO(this->get_logger(), "yaw_hex = %02x, %02x", yaw_hex[0], yaw_hex[1]);
        RCLCPP_INFO(this->get_logger(), "pitch_hex = %02x, %02x", pitch_hex[0], pitch_hex[1]);

        // 7. 전송할 버퍼에 각도 삽입
        att_send_buf[8] = yaw_hex[0];
        att_send_buf[9] = yaw_hex[1];
        att_send_buf[10] = pitch_hex[0];
        att_send_buf[11] = pitch_hex[1];

        // 8. CRC 계산 후 전송 버퍼에 삽입
        uint16_t crc = CRC16_cal(att_send_buf, sizeof(att_send_buf) - 2, 0);
        att_send_buf[12] = crc & 0xFF;  // LSB
        att_send_buf[13] = (crc >> 8) & 0xFF;  // MSB

        RCLCPP_INFO(this->get_logger(), "🔐 CRC16 = 0x%04X", crc);

        // 9. UDP 전송
        socklen_t addr_len = sizeof(struct sockaddr_in);
        if (sendto(sockfd, att_send_buf, sizeof(att_send_buf), 0,
                   (struct sockaddr *)&send_addr, addr_len) < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ sendto 실패");
            close(sockfd);
            return;
        }
        RCLCPP_INFO(this->get_logger(), "✅ 짐벌 명령 전송 완료");

        // 10. 응답 수신
        recv_len = recvfrom(sockfd, recv_buf, RECV_BUF_SIZE, 0,
                            (struct sockaddr *)&recv_addr, &addr_len);
        if (recv_len < 0) {
            RCLCPP_ERROR(this->get_logger(), "❌ 응답 수신 실패");
            close(sockfd);
            return;
        }

        // 11. 수신 데이터 HEX 출력
        std::stringstream ss;
        for (int i = 0; i < recv_len; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)recv_buf[i] << " ";
        }
        RCLCPP_INFO(this->get_logger(), "📨 수신 데이터 (%d bytes): %s", recv_len, ss.str().c_str());

        // 12. 소켓 종료
        close(sockfd);
    }
};

void GimbalControlNode::listener_callback_object_center_ned(const geometry_msgs::msg::Point32::ConstSharedPtr msg)
{
	object_ned_msg = *msg;	
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "받은 NED 좌표: x=%.2f, y=%.2f, z=%.2f",
    object_ned_msg.x, object_ned_msg.y, object_ned_msg.z);
}

void GimbalControlNode::listener_callback_local_position(const VehicleLocalPosition::ConstSharedPtr msg)
{
	fcc_position_N = msg->x;  // FC 기준,짐벌 기준으로 필요하면 보정
	fcc_position_E = msg->y;
	fcc_position_D = msg->z;
}

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GimbalControlNode>());
    rclcpp::shutdown();
    return 0;
}
