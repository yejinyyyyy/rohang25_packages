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

#define dt 0.05		// 1/20
#define posQ 1
#define velQ 1
#define posR 10

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using Vec3 = Eigen::Vector3d;
using Mat3 = Eigen::Matrix3d;

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
		// 	"/yolo_box_size", qos,  // Check YOLO publisher
		// 	[this](const sensor_msgs::msg::Image::ConstSharedPtr msg) { this->listener_callback_yolo_box(msg); });

		subscription_geo_node_control_flag = this->create_subscription<std_msgs::msg::Bool>(
			"/vision_nodes_control_flag", qos,
			[this](const std_msgs::msg::Bool::ConstSharedPtr msg) { this->listener_callback_geo_node_control_flag(msg); });
		
		subscription_ref_position = this->create_subscription<geometry_msgs::msg::Point>(
			"/object_ref_position", qos,  // Check Publisher
			[this](const geometry_msgs::msg::Point::ConstSharedPtr msg) { this->listener_callback_ref_position(msg); });

		timer_ = this->create_wall_timer(100ms, std::bind(&Geolocation_KF::timer_callback, this));  // Check timer period
		
		/******************************
		*  	Initialize KF Matrices	  *
		******************************/
		x << 0, 0, 0;		// Initial state matrix

		A << 1, 0, 0,		// State Transition matrix
			0, 1, 0,
			0, 0, 1;

		B << dt, 0, 0,		// Control matrix
			0, dt, 0,
			0, 0, dt;

		H << 1, 0, 0,		// Measurement matrix
			0, 1, 0,
			0, 0, 1;

		Q << 0.2, 0, 0,		// Process noise covariance
			0, 0.2, 0,
			0, 0, 0.2;

		R << 1.0, 0, 0,		// Measurement noise covariance
			0, 1.0, 0,
			0, 0, 1.0;
			
		P << 2000, 0, 0,	// Current error covariance
			0, 2000, 0,
			0, 0, 1000;

		K << 1172.596679, 0.0, 639.782266,		// Camera intrinsic parameter matrix
			0.0, 1186.213620, 271.135158,
			0, 0, 1;							// ZR10

		kf1_ptr = std::make_unique<kalmanfilter>(A, H, Q, R, P, x, B);

		/******************************
		*  	Create Log file		      *
		******************************/
		// create_file(SPT_FILE_PATH);		// remove if no need, create log header if needed

	};

private:
	/********************
	*  	ROS related     *
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
	*  	Variables & Member functions   *
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
	int ObjDetection_raw_x = 0;	// YOLO center point x
	int ObjDetection_raw_y = 0;	// YOLO center point y
	// string class_name; 			// YOLO class name

	geometry_msgs::msg::Point ref_position;

	geometry_msgs::msg::Point32 object_lla_msg;
	geometry_msgs::msg::Point32 object_ned_msg;
	geometry_msgs::msg::Point32 object_raw_msg;	

	std_msgs::msg::Bool geo_node_control_flag;
	std_msgs::msg::Bool kf_data_valid_flag_msg;	

	// Trim value  -- all needs to be set
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
	Eigen::Matrix<double, 3, 1> u;	// No use
	Eigen::Matrix<double, 3, 1> u1; // Control input

	Eigen::Matrix<double, 3, 3> K;
	Eigen::Matrix<double, 3, 3> B;

	Eigen::Matrix<double, 3, 1> z;	
	Eigen::Matrix<double, 3, 1> z_NED;	

	std::unique_ptr<kalmanfilter> kf1_ptr;  // 포인터로
	
	// Rotation matrices -- should it be memeber?
	Eigen::Matrix<double, 3, 3> rotm_gimbal_3d(double roll, double pitch, double yaw); 
	Eigen::Matrix<double, 3, 3> rotm_3d(double roll, double pitch, double yaw); 

	Eigen::Matrix3d R_fcc2ned;  	// fcc body frame -> local NED frame
	Eigen::Matrix3d R_gimbal2fcc;	// gimbal cam frame -> fcc body frame

	// Gimbal Euler state
	double roll_gimbal_f = 0.0;
	double pitch_gimbal_f = -3.141592/2;  // 90 deg -  FRD Frame 
	double yaw_gimbal_f = 0.0;

	// Data Buffer & Flags
	Eigen::Matrix<double, 3, 1> detections;	// Stores raw data as matrix

	double yolo_lat_diff = 0.0;  // Previous value - Current value
	double yolo_lon_diff = 0.0;	 
	double yolo_diff = 0.0;		// used to select closest value

	double Obj_Lat = 0.0;		// Chosen(Closest to last loop) value at this loop
	double Obj_Lon = 0.0;
	
	double Obj_Lat_except[10] = {0.0};	// History buffer (last 10 chosen values)
	double Obj_Lon_except[10] = {0.0};

	double pre_Obj_Lat = 0.0;			// Previous value
	double pre_Obj_Lon = 0.0;
	
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
	fcc_local_alt = -msg->z;  // D to alt
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
	ObjDetection_raw_x = msg->center_x;
	ObjDetection_raw_y = msg->center_y;
}

// void Geolocation_KF::listener_callback_yolo_box(const geometry_msgs::msg::Point32::ConstSharedPtr msg)
// {
// 	ObjDetection_box = *msg;
// }

void Geolocation_KF::listener_callback_geo_node_control_flag(const std_msgs::msg::Bool::ConstSharedPtr msg)
{
	geo_node_control_flag = *msg;
}

void Geolocation_KF::listener_callback_ref_position(const geometry_msgs::msg::Point::ConstSharedPtr msg)
{
	ref_position = *msg;	// Needs fix by mission & frame
}

void Geolocation_KF::timer_callback()
{
	// ==========================  constant loop!  =============================
	if(geo_node_control_flag.data)  // start when flag subscribed
	{
		// Check data validity
		if((float)ObjDetection_raw_x >= 1.0 && (float)ObjDetection_raw_y >= 1.0)
		{
			publish_object_data();	// Calulate & publish all data
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

	// Print log?
}	

/**
 * @brief Calculation, Publishes all topics
 */
void Geolocation_KF::publish_object_data()
{

	detections << (float)ObjDetection_raw_x, (float)ObjDetection_raw_y, 1.0000;

	float depth_f;
	depth_f = myAbs(fcc_local_alt);	// Edit needed (case of both missions)
	// std::cout<<"depth       ::    "<<depth_f<<std::endl;


	/******************
	*  	Geolocation   *
	*******************/
	// R_fcc2ned = rotm_3d(fcc_roll,  fcc_pitch,  fcc_yaw);

	// Eigen::Matrix<double, 3, 1> x_normal_coordinate_f = K.inverse() * detections;	
				
	//Eigen::Matrix<double, 3, 1> x_normal_coordinate_FRD_unit;
	// x_normal_coordinate_FRD_unit << x_normal_coordinate_f(0, 0), x_normal_coordinate_f(1, 0), x_normal_coordinate_f(2, 0);  // gimbal pitch 0 deg ver
	
	//Eigen::Matrix<double, 3, 1> x_normal_coordinate_FRD;
	//x_normal_coordinate_FRD = x_normal_coordinate_FRD_unit *  myAbs(depth_f);

	//Eigen::Matrix<double, 3, 3> R_gimbal2fcc = rotm_gimbal_3d(roll_gimbal_f, pitch_gimbal_f, yaw_gimbal_f);
	
	//Eigen::Matrix<double, 3, 1> x_normal_coordinate_ned_f = R_fcc2ned*R_gimbal2fcc*x_normal_coordinate_FRD;
	
	//z_NED << x_normal_coordinate_ned_f(0, 0), x_normal_coordinate_ned_f(1, 0), x_normal_coordinate_ned_f(2, 0);   
	
	Obj_Lat = ObjDetection_raw_x;	// ObjDetection_raw_x -> pixel
	Obj_Lon = ObjDetection_raw_y;   // ObjDetection_raw_y -> pixel
	
		

	/*****************************
	*  	Outlier Decision logic   *
	******************************/
	for( int i = 0 ; i < 9 ; i++ )  // saves the last 10 values 
	{
		Obj_Lat_except[i] = Obj_Lat_except[i+1];
		Obj_Lon_except[i] = Obj_Lon_except[i+1];
	}
	Obj_Lat_except[9] = Obj_Lat;  // update last value to current value
	Obj_Lon_except[9] = Obj_Lon;
	
	unsigned int except_cnt = 0;

	for (int j = 0; j < 10; j++)  
	{
		float x_diff = (Obj_Lat_except[j] - Obj_Lat);
		float y_diff = (Obj_Lon_except[j] - Obj_Lon);

		float dist_diff = sqrt(x_diff*x_diff + y_diff*y_diff);

		if( dist_diff > 0.25 )
		{					
			except_cnt = except_cnt + 1;
		}									
	}

	if( except_cnt > 5 )
	{
		Obj_Lat = pre_Obj_Lat;	// ignore new data
		Obj_Lon = pre_Obj_Lon;	
	}
	else
	{
		Obj_Lat = Obj_Lat;
		Obj_Lon = Obj_Lon;
	}

	pre_Obj_Lat = Obj_Lat;	// Update previous value to current value
	pre_Obj_Lon = Obj_Lon;	

	/***********************
	*  	Kalman filtering   *
	************************/
	// Kalman filter update
	z << Obj_Lat, Obj_Lon, fcc_local_alt;	
	u1 << fcc_vel_N, fcc_vel_E, fcc_vel_D ; // static model input	
	
	kf1_ptr->update(z, -u1, dt);	

	if (log_counter_ % 5 == 0) {
		RCLCPP_INFO(this->get_logger(),
		"\n*************************\n"
		"z(N)      :: %.2f\n"
		"z(E)      :: %.2f\n"
		"z(D)      :: %.2f\n"

		"kalman(N)      :: %.2f\n"
		"kalman(E)      :: %.2f\n"
		"kalman(D)      :: %.2f\n"
		"P(0)      	    :: %.2f\n"
		"*************************",
		z(0), z(1), z(2),
		kf1_ptr->x(0), kf1_ptr->x(1), kf1_ptr->x(2), kf1_ptr->P(0));
	}
	log_counter_ ++;

	/***********************
	*  	Generate message   *
	************************/

	lat_trim = x_trim*cos(fcc_yaw)-y_trim*sin(fcc_yaw);
	lon_trim = x_trim*sin(fcc_yaw)+y_trim*cos(fcc_yaw);

	// std::cout<<"lat_trim(m) 		::    "<<lat_trim<<std::endl;
	// std::cout<<"lon_trim(m) 		::    "<<lon_trim<<std::endl;

	if (z(2) > 5.0){	// Only when alt > 5m !!!
		object_lla_msg.x = fcc_latitude + kf1_ptr->x(0)* m2lat + lat_trim*m2lat;
		object_lla_msg.y = fcc_longitude + kf1_ptr->x(1)* m2lon + lon_trim*m2lon;
	}
	
	// RCLCPP_INFO(this->get_logger(),"Lat(pub) 	::    ");
	// RCLCPP_INFO(this->get_logger(),"Lon(pub) 	::    ");

	double distance_ref;  // distance from home point(== landing pad) -> not during rescue mission
	distance_ref = sqrt ((object_lla_msg.x - ref_position.x)*lat2m *(object_lla_msg.x - ref_position.x)*lat2m + (object_lla_msg.y - ref_position.y)*lon2m*(object_lla_msg.y - ref_position.y)*lon2m);  // Needs fix after ref_position fix
	
	// std::cout<<"Relative Distance   ::    "<< distance_ref <<std::endl;

	object_ned_msg.x = kf1_ptr->x(0);
	object_ned_msg.y = kf1_ptr->x(1);
	object_ned_msg.z = fcc_local_alt;

	object_raw_msg.x = Obj_Lat;
	object_raw_msg.y = Obj_Lon;
	object_raw_msg.z = fcc_local_alt;

	if(kf1_ptr->x(0) == NAN || kf1_ptr->P(0)>0.5)	// Check filter validity
	{
		kf_data_valid_flag_msg.data = 0;
	}
	else
	{
		kf_data_valid_flag_msg.data = 1;
	}

	/***********************
	*  	Publish message    *
	************************/
	object_lla_publisher_->publish(object_lla_msg);
	object_ned_publisher_->publish(object_ned_msg);
	object_raw_publisher_->publish(object_raw_msg);
	kf_data_valid_flag_publisher_->publish(kf_data_valid_flag_msg);
}


Eigen::Matrix<double, 3, 3> Geolocation_KF::rotm_gimbal_3d(double roll, double pitch, double yaw)
{
	// Eigen::Matrix<double, 3, 3> Rx;
	// Eigen::Matrix<double, 3, 3> Ry;
	// Eigen::Matrix<double, 3, 3> Rz;
	// Eigen::Matrix<double, 3, 3> R;

	// Rx << 1, 0, 0,
	// 0, cos(roll), sin(roll),
	// 0, -sin(roll), cos(roll);
	// Ry << cos(pitch), 0, -sin(pitch),
	// 0, 1, 0,
	// sin(pitch), 0, cos(pitch);

	// Rz << cos(yaw), sin(yaw), 0,
	// -sin(yaw), cos(yaw), 0,
	// 0, 0, 1;
	// R = Ry*Rx*Rz;
	// body_from_gimbal
	Mat3 R_roll  = Eigen::AngleAxisd(roll,  Vec3::UnitX()).toRotationMatrix();
	Mat3 R_pitch = Eigen::AngleAxisd(pitch, Vec3::UnitY()).toRotationMatrix();
	Mat3 R_yaw   = Eigen::AngleAxisd(yaw,   Vec3::UnitZ()).toRotationMatrix();

	Mat3 R_body_from_gimbal = R_yaw * R_pitch * R_roll;
	Mat3 R_gimbal_from_body = R_body_from_gimbal.transpose();

	return R_body_from_gimbal;
}

Eigen::Matrix<double, 3, 3> Geolocation_KF::rotm_3d(double roll, double pitch, double yaw)
{
	
	double m11;
	double m12;
	double m13;
	double m21;
	double m22;
	double m23;
	double m31;
	double m32;
	double m33;

	m11 = cos(pitch) * cos(yaw);
	m12 = cos(yaw)   * sin(roll) * sin(pitch) - cos(roll) * sin(yaw);
	m13 = sin(roll)  * sin(yaw) + cos(roll) * cos(yaw) * sin(pitch);

	m21 = cos(pitch) * sin(yaw);
	m22 = cos(roll)  * cos(yaw) + sin(roll) * sin(pitch) * sin(yaw);
	m23 = cos(roll)  * sin(pitch) * sin(yaw) - cos(yaw) * sin(roll);

	m31 = -sin(pitch);
	m32 = cos(pitch) * sin(roll);
	m33 = cos(roll)  * cos(pitch);
	
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
