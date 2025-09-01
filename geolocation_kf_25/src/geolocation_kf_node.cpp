/*
2025 한국로봇항공기대회
건국대학교 ASEC

Last edit date : 250409
Name : Yejin
Text:
 - Geolocation KF node
 - altitude 5m pub condition temp (5m  kalman data using---> not using)
*/

#include <iostream>
#include <math.h>
#include <chrono>
#include <stdio.h>
#include <stdlib.h>
#include <cmath> // std::fabs, std::isnan

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <std_msgs/msg/char.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/point32.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>  // Re-check after RTK application
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <yolov11_msgs/msg/detection.hpp>  // YOLO center point - custom msg
// #include <sensor_msgs/msg/image.hpp>  // YOLO box size

#include "kalmanfilter.h"

#define dt 0.1     // 1/100
#define posQ 1
#define velQ 1
#define posR 10

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

// ======== 추가: P 바닥값(대각 성분 최소 분산, 0 방지) ========
constexpr double P_MIN_VAR = 0.1;

// ===================== YOLO(640x640) → 원본(1280x720) 역변환 함수 =====================
inline void yoloToOrigResize(float u_y, float v_y, float& u, float& v) {
    constexpr float sx = 1280.0f / 640.0f; // 2.0
    constexpr float sy =  720.0f / 640.0f; // 1.125
    u = u_y * sx;  // 원본 픽셀 X
    v = v_y * sy;  // 원본 픽셀 Y
}
// ================================================================================

class Geolocation_KF : public rclcpp::Node
{
public:
    Geolocation_KF() : Node("geolocation_kf")
    {
        // QoS
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
        qos_profile.depth = 20;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

        // Publisher
        object_lla_publisher_ = this->create_publisher<geometry_msgs::msg::Point32>("/object_center_lla", 10);  // check QoS
        object_ned_publisher_ = this->create_publisher<geometry_msgs::msg::Point32>("/object_center_ned", 10);
        object_raw_publisher_ = this->create_publisher<geometry_msgs::msg::Point32>("/object_center_raw", 10);
        kf_data_valid_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/kf_data_valid_flag", 10);

        // Subscriber
        subscription_local_position = this->create_subscription<VehicleLocalPosition>(
            "/fmu/out/vehicle_local_position", qos,
            [this](const VehicleLocalPosition::ConstSharedPtr msg) { this->listener_callback_local_position(msg); });

        subscription_vehicle_attitude = this->create_subscription<VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            [this](const VehicleAttitude::ConstSharedPtr msg) { this->listener_callback_attitude(msg); });

        subscription_sensor_gps = this->create_subscription<SensorGps>(
            "/fmu/out/vehicle_gps_position", qos,
            [this](const SensorGps::ConstSharedPtr msg) { this->listener_callback_gps(msg); });

        subscription_yolo_center = this->create_subscription<yolov11_msgs::msg::Detection>(
            "/yolov11/detection", qos,
            [this](const yolov11_msgs::msg::Detection::ConstSharedPtr msg) { this->listener_callback_yolo_center(msg); });

        // subscription_yolo_box = this->create_subscription<sensor_msgs::msg::Image>(
        //  "/yolo_box_size", qos,
        //  [this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { this->listener_callback_yolo_box(msg); });

        subscription_geo_node_control_flag = this->create_subscription<std_msgs::msg::Bool>(
            "/vision_nodes_control_flag", qos,
            [this](const std_msgs::msg::Bool::ConstSharedPtr msg) { this->listener_callback_geo_node_control_flag(msg); });

        subscription_ref_position = this->create_subscription<geometry_msgs::msg::Point>(
            "/object_ref_position", qos,
            [this](const geometry_msgs::msg::Point::ConstSharedPtr msg) { this->listener_callback_ref_position(msg); });

        // 100 Hz
        timer_ = this->create_wall_timer(100ms, std::bind(&Geolocation_KF::timer_callback, this));

        /******************************
        *   Initialize KF Matrices
        ******************************/
        x << 0, 0, 0;        // Initial state matrix

        A << 1, 0, 0,        // State Transition matrix
             0, 1, 0,
             0, 0, 1;

        B << dt, 0,  0,      // Control matrix
             0,  dt, 0,
             0,  0,  dt;

        H << 1, 0, 0,        // Measurement matrix
             0, 1, 0,
             0, 0, 1;

        Q << 0.2, 0, 0,      // Process noise covariance
             0, 0.2, 0,
             0, 0, 0.2;

        R << 1.0, 0, 0,      // Measurement noise covariance
             0, 1.0, 0,
             0, 0, 1.0;

        P << 2000, 0,    0,  // Current error covariance
             0,    2000, 0,
             0,    0,    1000;

        // --- Camera intrinsics (from CameraInfo) -----------------------------
        // [CHANGED] fx,fy,cx,cy 정확히 반영
        const double fx = 889.0164708736945;
        const double fy = 889.0164708736945;
        const double cx = 640.5;
        const double cy = 360.5;
        K << fx, 0.0, cx,
             0.0, fy, cy,
             0.0, 0.0, 1.0;
        // ---------------------------------------------------------------------
        
        
	 //K << 1296.678812, 0.0, 714.493790,		// Camera intrinsic parameter matrix (New ZR10)
	 //    0.0, 1300.788873, 233.916367,
	 //    0.0, 0.0, 1;							// ZR10 (REAL?)
	
	// K << 2649.149157, 0.0, 600.476202           // jeahwan real? 
        //      0.0, 2540.369828, 401.606964
        //      0.0, 0.0, 1;
        kf1_ptr = std::make_unique<kalmanfilter>(A, H, Q, R, P, x, B);
    };

private:
    /********************
    *   ROS related
    *********************/
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr object_lla_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr object_ned_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point32>::SharedPtr object_raw_publisher_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr kf_data_valid_flag_publisher_;

    // Subscribers
    rclcpp::Subscription<px4_msgs::msg::VehicleLocalPosition>::SharedPtr subscription_local_position;
    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr subscription_vehicle_attitude;
    rclcpp::Subscription<px4_msgs::msg::SensorGps>::SharedPtr subscription_sensor_gps;
    rclcpp::Subscription<yolov11_msgs::msg::Detection>::SharedPtr subscription_yolo_center;
    // rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr subscription_yolo_box;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_geo_node_control_flag;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr subscription_ref_position;

    // Timer
    rclcpp::TimerBase::SharedPtr timer_;

    // Callback & Publisher functions
    void timer_callback(void);

    void listener_callback_local_position(const VehicleLocalPosition::ConstSharedPtr msg);
    void listener_callback_attitude(const VehicleAttitude::ConstSharedPtr msg);
    void listener_callback_gps(const SensorGps::ConstSharedPtr msg);
    void listener_callback_yolo_center(const yolov11_msgs::msg::Detection::ConstSharedPtr msg);
    // void listener_callback_yolo_box(const geometry_msgs::msg::Point32::ConstSharedPtr msg);
    void listener_callback_geo_node_control_flag(const std_msgs::msg::Bool::ConstSharedPtr msg);
    void listener_callback_ref_position(const geometry_msgs::msg::Point::ConstSharedPtr msg);

    void publish_object_data();  // Publishes all data at once

    /***********************************
    *   Variables & Member functions
    ***********************************/
    // Pub & Sub messages

    float fcc_latitude;  // save at callback function
    float fcc_longitude;
    float fcc_altitude;
    float fcc_roll;
    float fcc_pitch;
    float fcc_yaw;
    float fcc_vel_N;
    float fcc_vel_E;
    float fcc_vel_D;
    float fcc_local_alt;
    int ObjDetection_raw_x = 0; // YOLO center point x (640x640 기준)
    int ObjDetection_raw_y = 0; // YOLO center point y

    geometry_msgs::msg::Point ref_position;

    geometry_msgs::msg::Point32 object_lla_msg;
    geometry_msgs::msg::Point32 object_ned_msg;
    geometry_msgs::msg::Point32 object_raw_msg;

    std_msgs::msg::Bool geo_node_control_flag;
    std_msgs::msg::Bool kf_data_valid_flag_msg;

    // Trim value
    float x_trim = 0.0;
    float y_trim = 0.0;

    double lat_trim = 0.0;
    double lon_trim = 0.0;

    // Kalman filter
    Eigen::Matrix<double, 3, 3> A;
    Eigen::Matrix<double, 3, 3> H;
    Eigen::Matrix<double, 3, 3> Q;
    Eigen::Matrix<double, 3, 3> R;

    Eigen::Matrix<double, 3, 3> P;

    Eigen::Matrix<double, 3, 1> x;
    Eigen::Matrix<double, 3, 1> u; // No use
    Eigen::Matrix<double, 3, 1> u1; // Control input

    Eigen::Matrix<double, 3, 3> K;
    Eigen::Matrix<double, 3, 3> B;

    Eigen::Matrix<double, 3, 1> z;
    Eigen::Matrix<double, 3, 1> z_NED;

    std::unique_ptr<kalmanfilter> kf1_ptr;

    // Rotation matrices
    Eigen::Matrix<double, 3, 3> rotm_gimbal_3d(double roll, double pitch, double yaw);
    Eigen::Matrix<double, 3, 3> rotm_3d(double roll, double pitch, double yaw);

    Eigen::Matrix3d R_fcc2ned;      // body -> NED
    Eigen::Matrix3d R_gimbal2fcc;   // (unused here)

    // Gimbal Euler state
    double roll_gimbal_f = 0.0;
    double pitch_gimbal_f = -3.141592/2;  // 90 deg - FRD (수직 하향)
    double yaw_gimbal_f = 0.0;

    // Data Buffer & Flags
    Eigen::Matrix<double, 3, 1> detections;

    double yolo_lat_diff = 0.0;
    double yolo_lon_diff = 0.0;
    double yolo_diff = 0.0;

    double Obj_Lat = 0.0;  // (픽셀) 원본 u
    double Obj_Lon = 0.0;  // (픽셀) 원본 v

    double Obj_Lat_except[10] = {0.0};
    double Obj_Lon_except[10] = {0.0};

    double pre_Obj_Lat = 0.0;
    double pre_Obj_Lon = 0.0;

    // NED outlier history (m)
    double ned_N_except[10] = {0.0};
    double ned_E_except[10] = {0.0};
    double pre_z_N = 0.0;
    double pre_z_E = 0.0;
    double pre_z_D = 0.0;

    // Unit Conversion constants
    double lat2m = 110961.7060516;
    double lon2m = 89476.51124;
    double m2lat = 1/lat2m;
    double m2lon = 1/lon2m;

    double deg2rad = 3.141592/180;
    double rad2deg = 180/3.141592;

    uint64_t log_counter_;
};


/**
 * @brief Listener callback functions
 */

void Geolocation_KF::listener_callback_local_position(const VehicleLocalPosition::ConstSharedPtr msg)
{
    fcc_vel_N = msg->vx;
    fcc_vel_E = msg->vy;
    fcc_vel_D = msg->vz;
    fcc_local_alt = -msg->z;  // D to alt  (PX4 local z는 Down(+), 고도는 Up(+))
}

void Geolocation_KF::listener_callback_attitude(const VehicleAttitude::ConstSharedPtr msg)
{
    float w = msg->q[0];
    float x = msg->q[1];
    float y = msg->q[2];
    float z = msg->q[3];

    fcc_roll  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
    fcc_pitch = std::asin(2.0f * (w * y - z * x));
    fcc_yaw   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));
}

void Geolocation_KF::listener_callback_gps(const SensorGps::ConstSharedPtr msg)
{
    fcc_latitude  = msg->latitude_deg;
    fcc_longitude = msg->longitude_deg;
    fcc_altitude  = msg->altitude_msl_m;
}

void Geolocation_KF::listener_callback_yolo_center(const yolov11_msgs::msg::Detection::ConstSharedPtr msg)
{
    ObjDetection_raw_x = msg->center_x;  // 640x640 기준 픽셀
    ObjDetection_raw_y = msg->center_y;
}

void Geolocation_KF::listener_callback_geo_node_control_flag(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
    geo_node_control_flag = *msg;
}

void Geolocation_KF::listener_callback_ref_position(const geometry_msgs::msg::Point::ConstSharedPtr msg)
{
    ref_position = *msg;    // Needs fix by mission & frame
}

void Geolocation_KF::timer_callback()
{
    if(geo_node_control_flag.data)
    {
        if((float)ObjDetection_raw_x >= 1.0 && (float)ObjDetection_raw_y >= 1.0)
        {
            publish_object_data();
        }
        else
        {
            log_counter_++;
            if (log_counter_ % 5 == 0) {
                RCLCPP_INFO(this->get_logger(),"\n**************************************\n"
                                               "Object Detection info :: NO VALID DATA\n"
                                               "**************************************");
                kf_data_valid_flag_msg.data = 0;
                kf_data_valid_flag_publisher_->publish(kf_data_valid_flag_msg);
            }
        }
    }
}

/**
 * @brief Calculation, Publishes all topics
 */
void Geolocation_KF::publish_object_data()
{
    // 0) YOLO(640x640) → 원본(1280x720) 역변환
    float u640 = static_cast<float>(ObjDetection_raw_x);
    float v640 = static_cast<float>(ObjDetection_raw_y);
    float u_orig = 0.0f, v_orig = 0.0f;
    yoloToOrigResize(u640, v640, u_orig, v_orig);

    // 1) 지면까지 깊이 = 고도 (수직 하향, 타깃=지면)
    const double depth_f = std::fabs(fcc_local_alt);

    /******************
    *   Geolocation
    *******************/
    // Body -> NED 회전행렬
    R_fcc2ned = rotm_3d(fcc_roll,  fcc_pitch,  fcc_yaw);

    // 픽셀(원본 좌표계) → 카메라 정규화
    Eigen::Vector3d pix(u_orig, v_orig, 1.0);
    Eigen::Vector3d x_cam = K.inverse() * pix;

    // 깊이로 스케일 (카메라 좌표)
    Eigen::Vector3d ray_cam = x_cam * depth_f;

    // [CHANGED] 카메라 → 바디(FRD) 축 매핑
    // cam x->body +Y, cam y->body -X, cam z->body +Z
    static const Eigen::Matrix3d R_cam2body = (Eigen::Matrix3d() <<
         0, -1, 0,
         1,  0, 0,
         0,  0, 1).finished();

    Eigen::Vector3d ray_body = R_cam2body * ray_cam;

    // 바디 -> NED
    Eigen::Vector3d ray_ned = R_fcc2ned * ray_body;

    // 칼만 입력 (N,E,D)
    z << ray_ned(0), ray_ned(1), ray_ned(2);

    /*****************************
    *   Outlier Decision logic (NED, m)
    ******************************/
    for (int i = 0; i < 9; i++) {
        ned_N_except[i] = ned_N_except[i+1];
        ned_E_except[i] = ned_E_except[i+1];
    }
    double curr_N = z(0);
    double curr_E = z(1);
    double curr_D = z(2);
    ned_N_except[9] = curr_N;
    ned_E_except[9] = curr_E;

    unsigned int except_cnt = 0;
    for (int j = 0; j < 10; j++)
    {
        double dN = ned_N_except[j] - curr_N;
        double dE = ned_E_except[j] - curr_E;
        double dist_diff = std::sqrt(dN*dN + dE*dE);
        if (dist_diff > 1.5) {
            except_cnt++;
            RCLCPP_INFO(this->get_logger(),"************ Outlier Detected! (NED space, m) ************\n");
        }
    }
    if (except_cnt > 5) {
        z(0) = pre_z_N;
        z(1) = pre_z_E;
        z(2) = pre_z_D;
    }
    pre_z_N = z(0);
    pre_z_E = z(1);
    pre_z_D = z(2);

    // RAW 픽셀 히스토리 (그대로 유지)
    for( int i = 0 ; i < 9 ; i++ )  {
        Obj_Lat_except[i] = Obj_Lat_except[i+1];
        Obj_Lon_except[i] = Obj_Lon_except[i+1];
    }
    Obj_Lat = u_orig;
    Obj_Lon = v_orig;
    Obj_Lat_except[9] = Obj_Lat;
    Obj_Lon_except[9] = Obj_Lon;
    pre_Obj_Lat = Obj_Lat;
    pre_Obj_Lon = Obj_Lon;

    /***********************
    *   Kalman filtering
    ************************/
    u1 << fcc_vel_N, fcc_vel_E, fcc_vel_D;
    kf1_ptr->update(z, -u1, dt);

    // P 대칭화 및 바닥값
    kf1_ptr->P = 0.5 * (kf1_ptr->P + kf1_ptr->P.transpose());
    for (int i = 0; i < 3; ++i) {
        if (kf1_ptr->P(i,i) < P_MIN_VAR) kf1_ptr->P(i,i) = P_MIN_VAR;
    }

    if (log_counter_ % 5 == 0) {
        RCLCPP_INFO(this->get_logger(),
        "\n*************************\n"
        "z(N)      :: %.2f\n"
        "z(E)      :: %.2f\n"
        "z(D)      :: %.2f\n"
        "kalman(N) :: %.2f\n"
        "kalman(E) :: %.2f\n"
        "kalman(D) :: %.2f\n"
        "P(0)      :: %.6f\n"
        "*************************",
        z(0), z(1), z(2),
        kf1_ptr->x(0), kf1_ptr->x(1), kf1_ptr->x(2), kf1_ptr->P(0));
    }
    log_counter_ ++;

    /***********************
    *   Generate message
    ************************/
    lat_trim = x_trim*std::cos(fcc_yaw)-y_trim*std::sin(fcc_yaw);
    lon_trim = x_trim*std::sin(fcc_yaw)+y_trim*std::cos(fcc_yaw);

    if (z(2) > 5.0){ // Only when alt > 5m !!!
        object_lla_msg.x = fcc_latitude  + kf1_ptr->x(0)* m2lat + lat_trim*m2lat;
        object_lla_msg.y = fcc_longitude + kf1_ptr->x(1)* m2lon + lon_trim*m2lon;
        object_lla_msg.z = fcc_altitude;
    }

    object_ned_msg.x = kf1_ptr->x(0);  // N
    object_ned_msg.y = kf1_ptr->x(1);  // E
    object_ned_msg.z = kf1_ptr->x(2);  // D

    object_raw_msg.x = Obj_Lat;       // u_orig
    object_raw_msg.y = Obj_Lon;       // v_orig
    object_raw_msg.z = fcc_local_alt; // Alt(Up)

    if (std::isnan(kf1_ptr->x(0)) || kf1_ptr->P(0) > 0.5)
        kf_data_valid_flag_msg.data = 0;
    else
        kf_data_valid_flag_msg.data = 1;

    object_lla_publisher_->publish(object_lla_msg);
    object_ned_publisher_->publish(object_ned_msg);
    object_raw_publisher_->publish(object_raw_msg);
    kf_data_valid_flag_publisher_->publish(kf_data_valid_flag_msg);
}


Eigen::Matrix<double, 3, 3> Geolocation_KF::rotm_gimbal_3d(double roll, double pitch, double yaw)
{
    Mat3 R_roll  = Eigen::AngleAxisd(roll,  Vec3::UnitX()).toRotationMatrix();
    Mat3 R_pitch = Eigen::AngleAxisd(pitch, Vec3::UnitY()).toRotationMatrix();
    Mat3 R_yaw   = Eigen::AngleAxisd(yaw,   Vec3::UnitZ()).toRotationMatrix();
    Mat3 R_body_from_gimbal = R_yaw * R_pitch * R_roll;
    return R_body_from_gimbal;
}

Eigen::Matrix<double, 3, 3> Geolocation_KF::rotm_3d(double roll, double pitch, double yaw)
{
    double m11 = std::cos(pitch) * std::cos(yaw);
    double m12 = std::cos(yaw)   * std::sin(roll) * std::sin(pitch) - std::cos(roll) * std::sin(yaw);
    double m13 = std::sin(roll)  * std::sin(yaw) + std::cos(roll) * std::cos(yaw) * std::sin(pitch);

    double m21 = std::cos(pitch) * std::sin(yaw);
    double m22 = std::cos(roll)  * std::cos(yaw) + std::sin(roll) * std::sin(pitch) * std::sin(yaw);
    double m23 = std::cos(roll)  * std::sin(pitch) * std::sin(yaw) - std::cos(yaw) * std::sin(roll);

    double m31 = -std::sin(pitch);
    double m32 = std::cos(pitch) * std::sin(roll);
    double m33 = std::cos(roll)  * std::cos(pitch);

    Eigen::Matrix<double, 3, 3> A;
    A << m11, m21, m31,
         m12, m22, m32,
         m13, m23, m33;
    return A.transpose();
}

int main(int argc, char *argv[])
{
    std::cout << "Starting Geolocation / KF node..." << std::endl;

    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Geolocation_KF>());
    rclcpp::shutdown();
    return 0;
}

