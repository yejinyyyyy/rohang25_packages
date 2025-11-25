/*
/ 2025 한국로봇항공기대회
/ 건국대학교 ASEC
/ 
/ Last edit date : 250906
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/vehicle_global_position.hpp>
#include <px4_msgs/msg/airspeed.hpp>//
#include <px4_msgs/msg/vehicle_attitude.hpp>//
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp> 
#include <px4_msgs/msg/mission_result.hpp> 
#include <geometry_msgs/msg/point32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <rclcpp/rclcpp.hpp>
#include <yolov11_msgs/msg/detection.hpp>  

#include "rohang25_test/srv/set_guidance_param.hpp"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <algorithm> 
#include <deque>

#include "algebra.h"
#include "config.h"
#include "control.h"
#include "frame.h"
#include "global_def.h"
#include "guidance.h"
#include "mission.h"
#include "log.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using namespace std;
using rohang25_test::srv::SetGuidanceParam;
using std::placeholders::_1;
using std::placeholders::_2;

int current_flight_mode = MC;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) : Node("offboard_control_test"),
		log_counter_(0),
		current_confidence_(0.0f), current_timestamp_vehicle(0), current_timestamp_sample_(0),
		current_indicated_airspeed_(0.0f), current_true_airspeed_(0.0f),
		current_roll_(0.0f), current_pitch_(0.0f), current_yaw_(0.0f), current_heading_(0.0f), lat(0.0f), lon(0.0f),
		current_latitude_(0.0f), current_longitude_(0.0f), current_altitude_(0.0f), heading(0.0f), timer_sec(0.0f), ref_timestamp(0),
		state_{State::init}, service_result_{0}, service_done_{false}, nav_state_(0), arm_state_(1), allow_run_(false), latched_stop_(false), did_mode_arm_(false)
	{	
		precision_control_mode = this->declare_parameter<int>("precision_control", 1); // 0: Pixel-based control, 1: NED-based control 

		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		qos_profile.depth = 20;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vision_nodes_control_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/vision_nodes_control_flag", 10);
		gimbal_node_control_flag_publisher_ = this->create_publisher<std_msgs::msg::Int32>("/gimbal_node_control_flag", 10);

		subscription_local_position = this->create_subscription<VehicleLocalPosition>(
		    "/fmu/out/vehicle_local_position", qos,
		    [this](const VehicleLocalPosition::SharedPtr msg) { this->listener_callback_local_position(msg); });
		
		subscription_sensor_gps = this->create_subscription<SensorGps>(
		    "/fmu/out/vehicle_gps_position", qos,
		    [this](const SensorGps::SharedPtr msg) { this->listener_callback_gps(msg); });
		    
		subscription_global_position = this->create_subscription<VehicleGlobalPosition>(
		    "/fmu/out/vehicle_global_position", qos,
		    [this](const VehicleGlobalPosition::SharedPtr msg) { this->listener_callback_global_position(msg); });
		    
		subscription_airspeed = this->create_subscription<Airspeed>(
		    "/fmu/out/airspeed", qos,
		    [this](const Airspeed::SharedPtr msg) { this->listener_callback_airspeed(msg); });
		
		subscription_vehicle_status = this->create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status", qos,
			[this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) { this->listener_callback_vehicle_status(msg); });
	
		subscription_vehicle_attitude = this->create_subscription<VehicleAttitude>(
		    "/fmu/out/vehicle_attitude", qos,
		    [this](const VehicleAttitude::SharedPtr msg) { this->listener_callback_attitude(msg); });
		
		subscription_object_ned = this->create_subscription<geometry_msgs::msg::Point32>(
			"/object_center_ned", qos,
			[this](const geometry_msgs::msg::Point32::SharedPtr msg) { this->listener_callback_object_ned(msg); });

		subscription_object_pixel = this->create_subscription<yolov11_msgs::msg::Detection>(
			"/yolov11/detection", qos,
			[this](const yolov11_msgs::msg::Detection::SharedPtr msg) { this->listener_callback_object_pixel(msg); });

		subscription_kf_data_valid_flag = this->create_subscription<std_msgs::msg::Bool>(
			"/kf_data_valid_flag", qos,
			[this](const std_msgs::msg::Bool::SharedPtr msg) { this->listener_callback_kf_data_valid_flag(msg); });

		subscription_mission_state = this->create_subscription<std_msgs::msg::Int32>(
			"gpio_state", qos,
			[this](const std_msgs::msg::Int32::SharedPtr msg) { this->listener_callback_mission_state(msg); });

		subscription_mission_result = this->create_subscription<MissionResult>(
			"/fmu/out/mission_result", qos,
			[this](const px4_msgs::msg::MissionResult::SharedPtr msg) { this->listener_callback_mission_result(msg); });
		
		param_service_ = this->create_service<SetGuidanceParam>(
			"/guidance_param",
			std::bind(&OffboardControl::srv_callback_paramUpdate, this, _1, _2));

		vehicle_command_client_ = this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command");

		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));

		// Initialize velocity guidance parameter values
		if (precision_control_mode == 0)  // Pixel
		{
			param[0] = 0;   // control type - 0: PIXEL, 1: NED axis
			param[1] = 320; // goal position - X axis(1280.0) 
			param[2] = 320; // goal position - Y axis(720.0) 
		}
		else  // NED
		{
			param[0] = 1;   // control type - 0: PIXEL, 1: NED axis
			param[1] = 0; // goal position - North axis 35.0018
			param[2] = 0; // goal position - East axis 27.3916
		}
		param[3] = 0.1;  // p gain - X or North axis
		param[4] = 0.1;  // p gain - Y or East axis
		param[5] = 1.0;   // velocity saturation
		param[6] = 0.0;   // down velocity - 0.0 at start
		param[7] = 0.0;
		param[8] = 0.0;
		param[9] = 0.0;

		RCLCPP_INFO(this->get_logger(), "\nMission Start =================== 0906 UPDATE\n");

		/******************************
		*  	Create Log file		  *
		******************************/
		create_file(SUB_FILE_PATH);
		// create_file(PIXEL_FILE_PATH);
		// create_file(ALT_FILE_PATH);

	};

	void switch_to_offboard_mode();
	void arm();
	void disarm();
	void do_vtol_transition();
	void do_back_transition();
	void land();


private:

	rclcpp::Service<SetGuidanceParam>::SharedPtr param_service_;

	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr vision_nodes_control_flag_publisher_;
	rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr gimbal_node_control_flag_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_local_position;
	rclcpp::Subscription<VehicleGlobalPosition>::SharedPtr subscription_global_position;
	rclcpp::Subscription<Airspeed>::SharedPtr subscription_airspeed;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr subscription_vehicle_attitude;
	rclcpp::Subscription<SensorGps>::SharedPtr subscription_sensor_gps;
	rclcpp::Subscription<VehicleStatus>::SharedPtr subscription_vehicle_status;
	rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr subscription_object_ned;
	rclcpp::Subscription<yolov11_msgs::msg::Detection>::SharedPtr subscription_object_pixel;
	rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr subscription_mission_state;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_kf_data_valid_flag;
	rclcpp::Subscription<MissionResult>::SharedPtr subscription_mission_result;

	rclcpp::TimerBase::SharedPtr timer_;
	
	enum class State{  // only for arm & offboard mode at start
		init,
		offboard_requested,
		offboard_running,
		offboard_ready,
		offboard_switch,
		wait_for_stable_offboard_mode,
		arm_requested,
		armed,
		transition_requested,
		flying,
		land_requested
	} state_;

	uint8_t service_result_;
	bool service_done_;
	uint64_t log_counter_;
	uint8_t num_of_steps = 0;
	uint8_t offboard_setpoint_counter_ = 0;
	string MISSION_STATUS;
	string PRECISION_CONTROL_MODE;
	int precision_control_mode = 1;

	int nav_state_;
	int arm_state_;
	bool allow_run_;
	bool latched_stop_;
	bool did_mode_arm_;

	float current_confidence_;
	uint64_t current_timestamp_vehicle;
	uint64_t current_timestamp_gps;
	uint64_t current_time_gps_utc;
	uint64_t current_timestamp_sample_;
	float current_indicated_airspeed_;
	float current_true_airspeed_;
	float current_roll_;
	float current_pitch_;
	float current_yaw_;
	float current_heading_; 
	double current_latitude_; // sensor_gps
	double current_longitude_;
	double lat;
	double lon;
	float current_altitude_;
	float current_altitude_rel;  // local position Z offset 보정
	float current_global_altitude_; // vehicle_global_position
	float home_latitude_; // home_position(global)
	float home_longitude_;
	float home_altitude_; 
	float heading;			 // for calculation inside loop
	float yolo_score;
	float object_dist;
	float timer_sec;  
	uint64_t ref_timestamp;  
	int mission_no;

	VehicleLocalPosition local_pose{}; // local position -> subscribed
	TrajectorySetpoint pose{};  	   // local position -> to be published
	std::deque<px4_msgs::msg::VehicleLocalPosition> pose_history_;

	geometry_msgs::msg::Point32 current_object_ned; // subscribed object ned
	geometry_msgs::msg::Point32 current_object_pixel;
	geometry_msgs::msg::Point32 current_object_;  // pixel / NED 둘 중 선택한 값
	geometry_msgs::msg::Point32 final_object_;  // 정밀착륙 시 마지막 좌표 저장

	int process = 0;
	int i = 0;
	int WPT_LOG;
    
	bool control_mode[3] = {true, false, false}; // 0: position, 1: velocity, 2: attitude
	bool hold_flag = false;
	bool hold_flag2 = false;
	bool gps_flag = false;
	bool vision_nodes_control_flag = false;
	int gimbal_node_control_flag = 0;	 // 0: initial state, 1: start tracking, 2: finished tracking	
	int mission_flag = 1; // 들어오면 1 나가면 0
	bool detection_flag = false;  // kf_data_valid_flag
	bool descend_flag = false;
	bool landing_flag = false;

	float step = MC_SPEED;
	double h_err = MC_HORIZONTAL_ERROR;
	double v_err = MC_VERTICAL_ERROR;
	double a_err = HEADING_ERROR;
	double DESC_RATE;
	float MC_ACCEL = 0.25;
	float set_accel_s = 0;

	std::vector<double> start;
	std::vector<double> middle;
	std::vector<double> end;
	std::vector<double> center;
	std::vector<double> local;
	std::vector<double> local3;
	std::vector<double> local_vel;
	std::vector<double> home_local = {0, 0, 0};
	std::vector<double> set = {0, 0, 0};
	std::vector<double> set_vel = {0, 0, 0};
	std::vector<double> set_acc = {0, 0, 0};
	std::vector<double> vel_prev = {0, 0, 0};
	double remain;

	double_t param[10]; // Precise landing guidance parameters

	void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void srv_callback_paramUpdate(
        const std::shared_ptr<SetGuidanceParam::Request> request,
        std::shared_ptr<SetGuidanceParam::Response> response);
	void timer_callback(void);

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	void publish_node_control_flag();
	
	void listener_callback_local_position(const VehicleLocalPosition::SharedPtr msg);
	void listener_callback_global_position(const VehicleGlobalPosition::SharedPtr msg);
	void listener_callback_airspeed(const Airspeed::SharedPtr msg);
	void listener_callback_attitude(const VehicleAttitude::SharedPtr msg);
	void listener_callback_gps(const SensorGps::SharedPtr msg);
	void listener_callback_object_ned(const geometry_msgs::msg::Point32::SharedPtr msg);
	void listener_callback_object_pixel(const yolov11_msgs::msg::Detection::SharedPtr msg);
	void listener_callback_mission_state(const std_msgs::msg::Int32::SharedPtr msg);
	void listener_callback_mission_result(const MissionResult::SharedPtr msg);
	void listener_callback_kf_data_valid_flag(const std_msgs::msg::Bool::SharedPtr msg);
	void listener_callback_vehicle_status(const VehicleStatus::SharedPtr msg);
};

/**
 * @brief Send a command to switch to offboard mode / arm & disarm / transition & back
 */
void OffboardControl::switch_to_offboard_mode()
{
	RCLCPP_INFO(this->get_logger(), "requesting switch to Offboard mode");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
}

void OffboardControl::arm()
{
	RCLCPP_INFO(this->get_logger(), "requesting arm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
}

void OffboardControl::disarm()
{
	RCLCPP_INFO(this->get_logger(), "requesting disarm");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0);
}

void OffboardControl::do_vtol_transition()
{
	RCLCPP_INFO(this->get_logger(), "requesting transition to fixed-wing");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, FW);
	state_ = State::transition_requested;
}

void OffboardControl::do_back_transition()
{
	RCLCPP_INFO(this->get_logger(), "requesting transition to multicopter");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_VTOL_TRANSITION, MC);
	state_ = State::transition_requested;
}

void OffboardControl::land()
{
	RCLCPP_INFO(this->get_logger(), "requesting AUTO LAND");
	request_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
	state_ = State::land_requested;
}

// void OffboardControl::servo_retrive() 
// {
// 	RCLCPP_INFO(this->get_logger(), "Activating servo to retrieve payload");
// 	request_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_ACTUATOR); // needs parameters
// }


/**
 * @brief Request vehicle commands (Client)
 * @param command   Command code (matches VehicleCommand and MAVLink MAV_CMD codes)
 * @param param1    Command parameter 1
 * @param param2    Command parameter 2
 */
void OffboardControl::request_vehicle_command(uint16_t command, float param1, float param2)
{
	auto request = std::make_shared<px4_msgs::srv::VehicleCommand::Request>();

	VehicleCommand msg{};
	msg.param1 = param1;
	msg.param2 = param2;
	msg.command = command;
	msg.target_system = 1;
	msg.target_component = 1;
	msg.source_system = 1;
	msg.source_component = 1;
	msg.from_external = true;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	request->request = msg;

	service_done_ = false;
	auto result = vehicle_command_client_->async_send_request(request, std::bind(&OffboardControl::response_callback, this,
                           std::placeholders::_1));
	RCLCPP_INFO(this->get_logger(), "Command send");
}

/**
 * @brief Service callback function (Server)
 */
void OffboardControl::srv_callback_paramUpdate(
	const std::shared_ptr<SetGuidanceParam::Request> request,
	std::shared_ptr<SetGuidanceParam::Response> response)
{
	uint8_t idx = request->index;
	double val = request->value;
	if (idx < 10) {
	  param[idx] = val;
	  response->success = true;
	  RCLCPP_INFO(this->get_logger(), "[GUIDANCE] param[%u] set to %.3f", idx, val);
	} 
	else {
	  response->success = false;
	  RCLCPP_WARN(this->get_logger(), "[GUIDANCE] invalid index %u", idx);
	}
}

  
/**
 * @brief Listener callback functions
 */

void OffboardControl::listener_callback_local_position(const VehicleLocalPosition::SharedPtr msg)
{
	local_pose = *msg;

	current_heading_ = msg->heading * R2D;
	if (current_heading_ < 0) {
		current_heading_ += 360.0f;
	}
}

void OffboardControl::listener_callback_airspeed(const Airspeed::SharedPtr msg)
{
	current_confidence_ = msg->confidence;
	current_indicated_airspeed_ = msg->indicated_airspeed_m_s;
	current_true_airspeed_ = msg->true_airspeed_m_s;
}

void OffboardControl::listener_callback_attitude(const VehicleAttitude::SharedPtr msg)
{
	float w = msg->q[0];
	float x = msg->q[1];
	float y = msg->q[2];
	float z = msg->q[3];

	current_roll_  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
	current_pitch_ = std::asin(2.0f * (w * y - z * x));
	current_yaw_   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

	// current_heading_ = current_yaw_ * (180.0f / M_PI);
	// if (current_heading_ < 0) {
	// 	current_heading_ += 360.0f;
	// }
}

void OffboardControl::listener_callback_gps(const SensorGps::SharedPtr msg)
{
	current_latitude_  = msg->latitude_deg;
	current_longitude_ = msg->longitude_deg;
	current_altitude_  = msg->altitude_msl_m;
	current_timestamp_gps = msg->timestamp;
	current_time_gps_utc = msg->time_utc_usec;

	/******************************
	*  	Coordinates setting		  *
	******************************/

	if(FRAME_MODE == LLH && gps_flag == false ){  // wait GPS data

		home_local = {local_pose.y, local_pose.x, -local_pose.z}; // NED -> ENU
		std::vector<double> home_global = {current_latitude_, current_longitude_, current_altitude_};

		RCLCPP_INFO(this->get_logger(), "global: %f, %f, %f", home_global[0], home_global[1], home_global[2]);
		RCLCPP_INFO(this->get_logger(), "local: %f, %f, %f", home_local[0], home_local[1], home_local[2]);  // SensorGps data

		WPT[2][0] = home_global[0];  //Vertiport
		WPT[2][1] = home_global[1];
		WPT[3][0] = home_global[0];  
		WPT[3][1] = home_global[1];  

		for (int i=0; i<4; i++)
		{
			WPT[i][2] = WPT[i][2] + home_global[2];
		}

		mission_llh2enu(WPT, home_global);
		// mission_calib_local(WPT, home_local);

		gps_flag = true;
	}
}

void OffboardControl::listener_callback_vehicle_status(const VehicleStatus::SharedPtr msg)
{
	// Offboard → 이탈 시 파일럿 개입으로 간주해 래치(재진입 금지)
	if (!latched_stop_ && nav_state_ == 14 && msg->nav_state != 14) {  // Offboard 였다가 다른걸로 바뀌면
		latched_stop_ = true;
		RCLCPP_WARN(this->get_logger(), "Pilot override detected. Stop all commands.");
	}
	nav_state_ = msg->nav_state;
}

void OffboardControl::listener_callback_global_position(const VehicleGlobalPosition::SharedPtr msg)
{
	current_timestamp_vehicle = msg->timestamp;
	current_timestamp_sample_ = msg->timestamp_sample;
	lat = msg->lat;
	lon = msg->lon;
	current_global_altitude_ = msg->alt; // asml
}

void OffboardControl::listener_callback_object_pixel(const yolov11_msgs::msg::Detection::SharedPtr msg)
{
	current_object_pixel.x = msg->center_x;
	current_object_pixel.y = msg->center_y;
	yolo_score = msg->score;
}


void OffboardControl::listener_callback_object_ned(const geometry_msgs::msg::Point32::SharedPtr msg)
{
	current_object_ned = *msg;
}

void OffboardControl::listener_callback_mission_state(const std_msgs::msg::Int32::SharedPtr msg)
{
	mission_flag = msg->data;
}

void OffboardControl::listener_callback_mission_result(const MissionResult::SharedPtr msg)
{
	mission_no = msg->seq_current;
}

void OffboardControl::listener_callback_kf_data_valid_flag(const std_msgs::msg::Bool::SharedPtr msg)
{
	detection_flag = msg->data;
}


/**
 * @brief Set control mode 
 */
 void OffboardControl::publish_offboard_control_mode()
 {
	 OffboardControlMode msg{};
	 msg.position = control_mode[0];
	 msg.velocity = control_mode[1];
	 msg.acceleration = control_mode[2];
	 msg.attitude = false;
	 msg.body_rate = false;
	 msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	 offboard_control_mode_publisher_->publish(msg);
 }

 /**
 * @brief Publish mission node control flags 
 */
 void OffboardControl::publish_node_control_flag()
 {
	 std_msgs::msg::Bool vision_msg{};
	 std_msgs::msg::Int32 gimbal_msg{};
	 vision_msg.data = vision_nodes_control_flag;
	 gimbal_msg.data = gimbal_node_control_flag;
	 vision_nodes_control_flag_publisher_->publish(vision_msg);
	 gimbal_node_control_flag_publisher_->publish(gimbal_msg);
 }

/**
 * @brief Publish local setpoint & heading ===========================================
 */
void OffboardControl::publish_trajectory_setpoint()	
{
	/******************************
 	*   Position Update	  *
 	******************************/
	local3 = {local_pose.y, local_pose.x, -local_pose.z}; // ENU 
	current_altitude_rel = -local_pose.z - home_local[2]; // 지상고도

	if (precision_control_mode == 0)  // Pixel raw
	{
		current_object_ = current_object_pixel;  // Kalman filtered Pixel -> current_object_ned
		PRECISION_CONTROL_MODE = "RAW";
	}
	else  // Pixel NED
	{
		current_object_ = current_object_ned; 
		PRECISION_CONTROL_MODE = "NED";
	}
	/******************************
 	*   Calculate Setpoint  	  *
 	******************************/

	if(state_ == State::offboard_running){ 
		switch(process)
		{
		case 0:
			// i=0; Switched to Offboard mode

			// =========================================== Switching ==============================================
			MISSION_STATUS = "Entered Offboard mode";
			set_position(pose, WPT[i]);

			vision_nodes_control_flag = true;
			gimbal_node_control_flag = 1; // Start searching target
	
			if (is_arrived_hori(local_pose, WPT[i], h_err)) {
				if(hold_flag == false && hold(3))
    				hold_flag = true;
				if(hold_flag){
					heading = get_angle({WPT[i][0], WPT[i][1]}, {WPT[i+1][0], WPT[i+1][1]});
					set_heading(pose, heading);

					// if(is_arrived_direc(local_pose, heading, a_err)){
					//     if(hold_flag2 == false && hold(2))
					//         hold_flag2 = true;
					//     if(hold_flag2){
							RCLCPP_INFO(this->get_logger(), "=================== Arrived at WPT0 ===================");
							MISSION_STATUS = "Offboard switch COMPLETE";
							control_mode[0] = false; // position control off
							control_mode[1] = true; // velocity control on
							control_mode[2] = true; // acceleration control on
							//i++;       // 직진 할거면 주석 해제, 원래 미션이면 주석 필수***
							process++;

							hold_flag = false;
							hold_flag2 = false;
					// 	}
					// }
				}

			}
			break;
			
			// =========================================== 원본 이륙 코드 - QGC arming 필요 ==============================================
			// MISSION_STATUS = "Take Off";
			// set = {local_pose.y, local_pose.x, WPT[i][2]};
			// set_position(pose, set);
			// set_velocity(pose, {NAN, NAN, NAN});  // init
			// set_heading(pose, current_yaw_);

			// if(is_arrived_verti(local_pose, WPT[i], v_err)){
							
			// 	if(hold_flag == false && hold(2))
    		// 		hold_flag = true;
			// 	if(hold_flag){
			// 		start = {local_pose.x,local_pose.y};
			// 		end = {WPT[i][0], WPT[i][1]};
			// 		heading = get_angle({WPT[i][0], WPT[i][1]}, {WPT[i+1][0], WPT[i+1][1]});
			// 		set_heading(pose, heading);

			// 		if(is_arrived_direc(local_pose, heading, a_err)){
			// 		    if(hold_flag2 == false && hold(1))
			// 		        hold_flag2 = true;
			// 		    if(hold_flag2){
			// 				RCLCPP_INFO(this->get_logger(), "Take off Complete");
			// 				process++; // 다음 단계
			// 				i++;

			// 				hold_flag = false;
			// 				hold_flag2 = false;
			// 			}
			// 		}
			// 	}
			// }

			break;
		

		case 1: // to WPT1 (mission point)
			// i=1; WPT#1
		#ifndef MISSION_TEST_MODE
			
			// control_mode[0] = false; // position control off
			// control_mode[1] = true; // velocity control on
			// control_mode[2] = true; // acceleration control on
			MISSION_STATUS = "Move to next WP";

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			local_vel = {local_pose.vy, local_pose.vx};
			set = line_guidance(start, end, local, step);  //change to start
			// set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_vel = velocity_guidance_mc(local_vel, vel_prev, set, step, MC_ACCEL, set_accel_s, remain_dist(local_pose, WPT[i]));
			set_acc = mult_const(mult_const(set_vel, 1/norm(set_vel)), set_accel_s);
			vel_prev = set_vel;
			
			set_position(pose, {NAN, NAN, WPT[i][2]});
			set_velocity(pose, set_vel);
			set_accel(pose, set_acc);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				if(hold_flag == false && hold(2))
					hold_flag = true;
				if(hold_flag){
					RCLCPP_INFO(this->get_logger(), "\n================== Arrived at WPT2 ==================");
					process++; 
					// i++;

					hold_flag = false;
					hold_flag2 = false;

				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 5: arrived at WPT2
				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 6: P turn start
				}
			}
			break;
		#else
			process++; //pass
		#endif


		case 2: // Precise Landing
			// i=1; WPT#1 
		#ifndef MISSION_TEST_MODE
			if(detection_flag == true){  // if valid detection
				object_dist = sqrt(current_object_.x*current_object_.x + current_object_.y*current_object_.y);
				
				if(object_dist <= 0.2 && !descend_flag){  // 1회 실행
					param[6] = 0.4;  // 일정 범위 이내로 정렬시 하강 시작
					set = {NAN, NAN, NAN};
					descend_flag = true;
				}
				
				if(descend_flag && -local_pose.z < 5 && !landing_flag){  // 1회 실행
					param[6] = 0.2;  // 5m까지 정밀착륙시 더 느리게 하강
					//set_vel = {0, 0, -param[6]};  // precise_landing_guidance 안들어가서 바로 ENU로 반환
					final_object_ = current_object_;  // 마지막 인식값 사용
					landing_flag = true;
				}
			// ====================================================================================================
				if(object_dist > 0.2 && !descend_flag){  // 정렬 중, 하강 X
					MISSION_STATUS = "Detected Marker, Start Aligning";
								
					set = {NAN, NAN, -local_pose.z};
					set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
				}
				else if(descend_flag && -local_pose.z > 3.5){  // 하강하는 모든 경우	
					if(!landing_flag){
						MISSION_STATUS = "Detected Marker, Start Descending -0.4m/s";
						set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
					} 
					else {  // landing_flag == true
						MISSION_STATUS = " Final Marker, Start Descending -0.2m/s";
						set_vel = precise_landing_guidance(param, current_heading_, final_object_, local3);  // 5m 이하 - 마지막 값으로만 하강
					}
				}
				else if(landing_flag && -local_pose.z < 3.5){  
					RCLCPP_INFO(this->get_logger(), "\n================== Auto-Land ==================");
					land();
					MISSION_STATUS = "AUTO-LAND";
					landing_flag = false;
					// =============================== end of mission ====================================
				}
			// ====================================================================================================
				set_position(pose, set);
				set_velocity(pose, set_vel);
			}
			else{
				if(-local_pose.z > 5.0){
					MISSION_STATUS = "Searching for Marker..";
					if(hold_flag == false){
						set = WPT[i]; // 좌표 유지
						set_vel = {NAN, NAN, NAN}; 
						if(hold(10)) hold_flag = true;
					}
					if(hold_flag){
						set = {WPT[i][0], WPT[i][1], NAN}; // 좌표 유지
						set_vel = {NAN, NAN, -0.4};   // -0.4m/s로 하강
					}
					set_position(pose, set);
					set_velocity(pose, set_vel);
				}
				else{
					MISSION_STATUS = "Cannot find marker, Auto-Land";
					
					land();

					if(state_ == State::land_requested)
						process++;
				}
			}

			break;
		#else
			process++; //pass
		#endif


		case 3: // Rescue mission - Hovering & Descend
			// i=1; WPT#1
			if(detection_flag == true){  // if valid detection
				object_dist = sqrt(current_object_.x*current_object_.x + current_object_.y*current_object_.y);
				
				if(object_dist <= 0.2 && !descend_flag){  // 1회 실행
					param[6] = 0.4;  // 일정 범위 이내로 정렬시 하강 시작
					set = {NAN, NAN, NAN};
					descend_flag = true;
				}
				
				if(descend_flag && -local_pose.z < (5 - 1) && !landing_flag){  // 1회 실행
					param[6] = 0.2;  // 5m까지 정밀착륙시 더 느리게 하강
					//set_vel = {0, 0, -param[6]};  // precise_landing_guidance 안들어가서 바로 ENU로 반환
					// final_object_ = current_object_;  // 마지막 인식값 사용 - 구조자는 계속 인식하는게 나을듯
					landing_flag = true;
				}

				if(object_dist > 0.2 && !descend_flag){  // 정렬 중, 하강 X
					MISSION_STATUS = "Detected Target, Start Aligning";
								
					set = {NAN, NAN, -local_pose.z};  
					set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
				}
				else if(descend_flag && -local_pose.z > (2 - 1)){  // 하강하는 모든 경우	
					if(!landing_flag){  
						MISSION_STATUS = "Detected Target, Start Descending -0.4m/s";
						set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
					} 
					else {   // landing_flag == true
						MISSION_STATUS = "Detecting Target, Start Descending -0.2m/s";
						set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);  // 5m 이하 - 마지막 값으로만 하강
					}
				}
				else if(landing_flag && -local_pose.z <= (2 - 1)){  // 1회 실행

					MISSION_STATUS = "Final Aligning";
								
					set = {NAN, NAN, (2-1)};  
					set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);

					if(object_dist <= 0.2){
						RCLCPP_INFO(this->get_logger(), "\n============ Start Hovering ============");
						MISSION_STATUS = "Hovering for Rescue Mission";
						set = {local_pose.y, local_pose.x, (2 - 1)};  // check Z
						set_vel = {NAN, NAN, NAN};
						set_position(pose, set);
						set_velocity(pose, set_vel);
						ref_timestamp = current_timestamp_vehicle;  // save timestamp since hover start
	
						descend_flag = false;
						landing_flag = false;
						hold_flag = false;
						process++;
					}
				}  // else?
				
				set_position(pose, set);
				set_velocity(pose, set_vel);
			}
			
			else{
				if(-local_pose.z > (5 - 1)){
					MISSION_STATUS = "Searching for Target..";
					if(hold_flag == false){
						set = WPT[i]; // 좌표 유지
						set_vel = {NAN, NAN, NAN}; 
						if(hold(10)) hold_flag = true;
					}
					if(hold_flag){
						set = {WPT[i][0], WPT[i][1], NAN}; // 좌표 유지
						set_vel = {NAN, NAN, -0.3};   // -0.3m/s로 하강
					}
					set_position(pose, set);
					set_velocity(pose, set_vel);
				}
				else{
					MISSION_STATUS = "Cannot find target, hovering";
					set = {local_pose.y, local_pose.x, (2 - 1)};  // check Z
					set_vel = {NAN, NAN, NAN};
					set_position(pose, set);
					set_velocity(pose, set_vel);
					ref_timestamp = current_timestamp_vehicle;  // save timestamp since hover start
					
					hold_flag = false;
					process++;
				}
			}

			break;


		case 4: // Complete mission & Return to WPT1
			// i=1; WPT#1

			timer_sec = (current_timestamp_vehicle - ref_timestamp) / 1e6; // microsec -> sec

			if(!hold_flag){  // 리프트 장치 하강 전
				MISSION_STATUS = "Holding, !!! Activate Rescue Device !!!";
				if(hold(90)) {  
					MISSION_STATUS = "Waiting for Return...";
					hold_flag = true;  // mission_flag 1 -> 0s
				}
			}

			if(hold_flag){  // 리프트장치 복귀  ((&& mission_flag == 1))
				MISSION_STATUS = "** MISSION COMPLETE** | 고도 상승중...";

				set = {WPT[i][0], WPT[i][1], NAN}; 
				set_vel = {NAN, NAN, 1.0};   // 1m/s 상승
				set_position(pose, set);
				set_velocity(pose, set_vel);
			}

			// hold X
			if(is_arrived_verti(local_pose, WPT[i+1], v_err)){  // 고도 10m 도달
				RCLCPP_INFO(this->get_logger(), "================= Move to release point "); // P턴 원 탈출 지점 도달
				ref_timestamp = 0.0; 
				timer_sec = 0.0;  // reset timer
				hold_flag = false;
				param[6] = 0.0;  // reset descend speed

				process++;
				i++;
			}
			
			break;

			
		case 5: // to WPT2 (Release point)
			// i=2; WPT#2
			MISSION_STATUS = "Moving to release point...";

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			local_vel = {local_pose.vy, local_pose.vx};
			set = line_guidance(start, end, local, step);  //change to start
			// set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_vel = velocity_guidance_mc(local_vel, vel_prev, set, step, MC_ACCEL, set_accel_s, remain_dist(local_pose, WPT[i]));
			set_acc = mult_const(mult_const(set_vel, 1/norm(set_vel)), set_accel_s);
			vel_prev = set_vel;
			
			set_position(pose, {NAN, NAN, WPT[i][2]});
			set_velocity(pose, set_vel);
			set_accel(pose, set_acc);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				if(hold_flag == false && hold(2))
					hold_flag = true;
				if(hold_flag){
					RCLCPP_INFO(this->get_logger(), "\n================== Arrived at WPT2 ==================");
					process++; 
					// i++;

					hold_flag = false;
					hold_flag2 = false;

				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 5: arrived at WPT2
				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 6: P turn start
				}
			}
			break;

		case 6: // Relase Target
			// i=2; WPT#2
			if(detection_flag == true){  // if valid detection
				object_dist = sqrt(current_object_.x*current_object_.x + current_object_.y*current_object_.y);
				
				if(object_dist <= 0.2 && !descend_flag){  // 1회 실행
					param[6] = 0.4;  // 일정 범위 이내로 정렬시 하강 시작
					set = {NAN, NAN, NAN};
					descend_flag = true;
				}
				
				if(descend_flag && -local_pose.z < (5 - 1) && !landing_flag){  // 1회 실행
					param[6] = 0.2;  // 5m까지 정밀착륙시 더 느리게 하강
					//set_vel = {0, 0, -param[6]};  // precise_landing_guidance 안들어가서 바로 ENU로 반환
					final_object_ = current_object_;  // 마지막 인식값 사용
					landing_flag = true;
				}

				if(object_dist > 0.2 && !descend_flag){  // 정렬 중, 하강 X
					MISSION_STATUS = "Detected Marker, Start Aligning";
								
					set = {NAN, NAN, -local_pose.z};
					set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
				}
				else if(descend_flag && -local_pose.z > (2 - 1)){  // 하강하는 모든 경우	
					if(!landing_flag){  
						MISSION_STATUS = "Detected Marker, Start Descending -0.4m/s";
						set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
					} 
					else {   // landing_flag == true
						MISSION_STATUS = " Final Marker, Start Descending -0.2m/s";
						set_vel = precise_landing_guidance(param, current_heading_, final_object_, local3);  // 5m 이하 - 마지막 값으로만 하강
					}
				}
				else if(landing_flag && -local_pose.z <= (2 - 1)){  // 1회 실행
					RCLCPP_INFO(this->get_logger(), "\n============ Start Hovering ============");
					MISSION_STATUS = "Hovering for Release";
					set = {local_pose.y, local_pose.x, (2 - 1)};  // check Z
					set_vel = {NAN, NAN, NAN};
					set_position(pose, set);
					set_velocity(pose, set_vel);
					ref_timestamp = current_timestamp_vehicle;  // save timestamp since hover start

					descend_flag = false;
					landing_flag = false;
					hold_flag = false;
					process++;
				}  // else?
				
				set_position(pose, set);
				set_velocity(pose, set_vel);
			}
			
			else{
				if(-local_pose.z > (2 - 1)){
					MISSION_STATUS = "Searching for Marker..";
					if(hold_flag == false){
						set = WPT[i]; // 좌표 유지
						set_vel = {NAN, NAN, NAN}; 
						if(hold(10)) hold_flag = true;
					}
					if(hold_flag){
						set = {WPT[i][0], WPT[i][1], NAN}; // 좌표 유지
						set_vel = {NAN, NAN, -0.3};   // -0.3m/s로 하강
					}
					set_position(pose, set);
					set_velocity(pose, set_vel);
				}
				else{
					MISSION_STATUS = "Cannot find Marker, hovering";
					set = {local_pose.y, local_pose.x, (2 - 1)};  // check Z
					set_vel = {NAN, NAN, NAN};
					set_position(pose, set);
					set_velocity(pose, set_vel);
					ref_timestamp = current_timestamp_vehicle;  // save timestamp since hover start

					hold_flag = false;
					process++;
				}
			}

			break;


		case 7: // Complete mission & Return to WPT2
			// i=2; WPT#2

			timer_sec = (current_timestamp_vehicle - ref_timestamp) / 1e6; // microsec -> sec

			if(!hold_flag){  // 리프트 장치 하강 전
				MISSION_STATUS = "Holding, !!! Activate Rescue Device !!!";
				if(hold(60)) { 
					MISSION_STATUS = "Waiting for Return...";
					hold_flag = true;  
				}
			}

			if(hold_flag){
				MISSION_STATUS = "** MISSION COMPLETE** | 고도 상승중...";

				set = {WPT[i][0], WPT[i][1], NAN}; 
				set_vel = {NAN, NAN, 1.0};   // 1m/s 상승
				set_position(pose, set);
				set_velocity(pose, set_vel);
			}

			// hold X
			if(is_arrived_verti(local_pose, WPT[i], v_err)){
				RCLCPP_INFO(this->get_logger(), "================= Move to Vertiport "); // P턴 원 탈출 지점 도달
				ref_timestamp = 0.0; 
				timer_sec = 0.0;  // reset timer
				hold_flag = false;
				param[6] = 0.0;  // reset descend speed

				process++;
				i++;
			}
			
			break;

		case 8: // to WPT3 (Vertiport)
			// i=3; WPT#3
			MISSION_STATUS = "Return to Home...";

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			local_vel = {local_pose.vy, local_pose.vx};
			set = line_guidance(start, end, local, step);  //change to start
			// set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_vel = velocity_guidance_mc(local_vel, vel_prev, set, step, MC_ACCEL, set_accel_s, remain_dist(local_pose, WPT[i]));
			set_acc = mult_const(mult_const(set_vel, 1/norm(set_vel)), set_accel_s);
			vel_prev = set_vel;
			
			set_position(pose, {NAN, NAN, WPT[i][2]});
			set_velocity(pose, set_vel);
			set_accel(pose, set_acc);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				if(hold_flag == false && hold(2))
					hold_flag = true;
				if(hold_flag){
					RCLCPP_INFO(this->get_logger(), "\n================== Arrived at Vertiport ==================");
					process++; 
					i++;

					hold_flag = false;
					hold_flag2 = false;

				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 5: arrived at WPT2
				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 6: P turn start
				}
			}
			break;

		case 9: // Precise Landing
			// i=3; WPT#3

			if(detection_flag == true){  // if valid detection
				object_dist = sqrt(current_object_.x*current_object_.x + current_object_.y*current_object_.y);
				
				if(object_dist <= 0.2 && !descend_flag){  // 1회 실행
					param[6] = 0.4;  // 일정 범위 이내로 정렬시 하강 시작
					set = {NAN, NAN, NAN};
					descend_flag = true;
				}
				
				if(descend_flag && -local_pose.z < 5 && !landing_flag){  // 1회 실행
					param[6] = 0.2;  // 5m까지 정밀착륙시 더 느리게 하강
					//set_vel = {0, 0, -param[6]};  // precise_landing_guidance 안들어가서 바로 ENU로 반환
					final_object_ = current_object_;  // 마지막 인식값 사용
					landing_flag = true;
				}

				if(object_dist > 0.2 && !descend_flag){  // 정렬 중, 하강 X
					MISSION_STATUS = "Detected Marker, Start Aligning";
								
					set = {NAN, NAN, -local_pose.z};
					set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
				}
				else if(descend_flag && -local_pose.z > 3.5){  // 하강하는 모든 경우	
					if(!landing_flag){
						MISSION_STATUS = "Detected Marker, Start Descending -0.4m/s";
						set_vel = precise_landing_guidance(param, current_heading_, current_object_, local3);
					} 
					else {  // landing_flag == true
						MISSION_STATUS = " Final Marker, Start Descending -0.2m/s";
						set_vel = precise_landing_guidance(param, current_heading_, final_object_, local3);  // 5m 이하 - 마지막 값으로만 하강
					}
				}
				else if(landing_flag && -local_pose.z < 3.5){  
					RCLCPP_INFO(this->get_logger(), "\n================== Auto-Land ==================");
					land();
					MISSION_STATUS = "AUTO-LAND";
					hold_flag = false;
					landing_flag = false;
					// =============================== end of mission ====================================
				}
				set_position(pose, set);
				set_velocity(pose, set_vel);
			}
			else{
				if(-local_pose.z > 5.0){
					MISSION_STATUS = "Searching for Marker..";
					if(hold_flag == false){
						set = WPT[i]; // 좌표 유지
						set_vel = {NAN, NAN, NAN}; 
						if(hold(10)) hold_flag = true;
					}
					if(hold_flag){
						set = {WPT[i][0], WPT[i][1], NAN}; // 좌표 유지
						set_vel = {NAN, NAN, -0.4};   // -0.4m/s로 하강
					}
					set_position(pose, set);
					set_velocity(pose, set_vel);
				}
				else{
					MISSION_STATUS = "Cannot find marker, AUTO-LAND";
					
					land();
					
				}
			}

			break;

		}
	}

	/******************************
 	*  Publish Setpoint           *
 	******************************/
	pose.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(pose);
}

/** ===================================================================================
 * @brief Timer callback function (**Main loop**) 
 */
void OffboardControl::timer_callback(void){

	/*************************************
 	*  	Service state (Arming sequence)  *
 	**************************************/		
	 if (latched_stop_) return; // ★ 파일럿 개입 후엔 완전 정지
	 if (offboard_setpoint_counter_ < 11) {  // 카운터 갱신
		 offboard_setpoint_counter_++;
	}

	switch (state_)
	{
	case State::init :
	{	
		RCLCPP_INFO(this->get_logger(), "노드 실행");
		set_position(pose, {NAN, NAN, NAN});
		state_ = State::offboard_ready;
	}
	break;
	case State::offboard_ready :
	{	
		if (offboard_setpoint_counter_ >= 10) {  // WPT0 근처에 오면 오프보드 신호 주기
			double dist_to_SWITCH = hypot(local_pose.y - WPT[0][0], local_pose.x - WPT[0][1]);
			double SWITCH_DIST = 5.0f;

			if (dist_to_SWITCH < SWITCH_DIST) {
				MISSION_STATUS = "Offboard 조건 1 달성, Sending Sepoints... ";
				state_ = State::offboard_switch;
			} 
			else MISSION_STATUS = "** OFFBOARD NOT READY **";        		
		}
	}
	break;

	case State::offboard_switch :
	{	
		set_position(pose, WPT[i]);  // Send setpoint before switch command
		set_heading(pose, current_yaw_);  

		publish_offboard_control_mode();
		publish_trajectory_setpoint(); 

		// 1. 현재 위치 push
		pose_history_.push_back(local_pose);
	
		// 2. 50개 이상이면 오래된 것 제거 (5초치만 유지)
		if (pose_history_.size() > 10) {
			pose_history_.pop_front();
		}
	
		// 3. 평균 위치 계산
		double avg_x = 0.0, avg_y = 0.0;
		for (const auto& p : pose_history_) {
			avg_x += p.x;
			avg_y += p.y;
		}
		avg_x /= pose_history_.size();
		avg_y /= pose_history_.size();
	
		// 4. 웨이포인트와 평균 위치 거리 계산
		double dist_to_SWITCH = hypot(avg_x - WPT[0][1], avg_y - WPT[0][0]);
	
		if (dist_to_SWITCH < 0.7) {
			switch_to_offboard_mode();
			RCLCPP_INFO(this->get_logger(), "=================== Offboard requested ===================");
			state_ = State::offboard_running;
		}
	}
	break;

	case State::offboard_running :
	{
		publish_offboard_control_mode();
		publish_trajectory_setpoint(); 
	}		
		
	// case State::transition_requested :
	// 	if(service_done_){
	// 		if (service_result_==0){
	// 			RCLCPP_INFO(this->get_logger(), "Transition success");
	// 				state_ = State::armed;
	// 		}
	// 		else{
	// 			RCLCPP_ERROR(this->get_logger(), "Transition Failed, exiting");
	// 			rclcpp::shutdown();
	// 		}
	// 	}
	// 	break;

	default:
		break;
	}

	/******************************
 	*  	Publishes Data here       *
 	******************************/
	publish_node_control_flag();	// publish all node control flags 

	/****************************
 	*  	Print Data on Screen    *
 	*****************************/
	remain = remain_dist(local_pose, WPT[i]);
	// ======================== WPT Log ========================
	// 이륙~WP1 이동 : 1
	// VP 상공 도착 및 착륙 : 9 (마지막 WP)
	if(nav_state_ == 14 || mission_no >= 20) // nav_state_ 14(Offboard) or Switching 지점(미션 번호 확인**)
		WPT_LOG = i + 6; // 코드 내 플래그 i+6
	else{
      if(mission_no == 0 || mission_no == 1) WPT_LOG = 1;
	  else if(mission_no == 2 || mission_no == 3 || mission_no == 4 || mission_no == 5) WPT_LOG = 2;
	  else if(mission_no == 6 || mission_no == 7 || mission_no == 8) WPT_LOG = 3;
	  else if(mission_no == 9 || mission_no == 10 || mission_no == 11 || mission_no == 12) WPT_LOG = 4;
	  else if(mission_no == 13 || mission_no == 14 || mission_no == 15 || mission_no == 16) WPT_LOG = 5;  
	  else if(mission_no == 17 || mission_no == 18 || mission_no == 19) WPT_LOG = 6;  
	  else WPT_LOG = mission_no;
   }

	log_counter_++;
	if (log_counter_ % 5 == 0) {
		RCLCPP_INFO(this->get_logger(),
					"\n============== CURRENT STATE ==============\n"
					"Timestamp: %ld\n"
					"Current(NED): %.2f, %.2f %.2f\n"
					"Setpoint(NED): %.2f, %.2f, %.2f\n"
					"Velocity setpoint: %.3f, %.3f, %.2f | %.2fm/s | %.2fm/s^2\n"
					"Object(%s) :: %.2f, %.2f | Dist: %.2f\n"
					"Current heading: %.2f | Set heading: %.2f\n"
					"Remain Dist: %f\n"
					"Case #%d  | WPT#%d  | Mission No: %d\n"
					"============== MISSION STATUS =============\n"
					"%s\n"
					"=========================== Timer: %.1f sec\n",
					// "==================================\n"
					// "|Altitude|  Global Rel: %.4f, Local: %.4f\n",
					//"  GPS: %.4f, Fused: %.4f\n",
					current_timestamp_vehicle, local_pose.x, local_pose.y, -local_pose.z,  
					pose.position[0], pose.position[1], pose.position[2],
					pose.velocity[0], pose.velocity[1], pose.velocity[2], norm(set_vel), set_accel_s,
					PRECISION_CONTROL_MODE.c_str(), current_object_.x, current_object_.y, object_dist,
					current_heading_, DEF_R2D(heading),
					remain, process, WPT_LOG, mission_no, MISSION_STATUS.c_str(), timer_sec
					// current_altitude_-local_pose.ref_alt, local_pose.z
					// current_altitude_, current_global_altitude_
				);
	}
	
	/**********************
 	*  CSV file Logging   *
 	***********************/
	save_submit_data(SUB_FILE_PATH, nav_state_, WPT_LOG, current_timestamp_gps,
		lat, lon, current_global_altitude_, local_pose.ax, local_pose.ay, local_pose.az,
		// , current_time_gps_utc, 
		current_roll_, current_pitch_, current_yaw_);
	// 1. 비행모드 (자동 / 수동) 	-> nav_state_ (int)
	// 2. WPT number			-> WPT_LOG (int)
	// 3. GPS time(sec)			-> current_timestamp_gps (int)
	// 4. Latitude(deg)			-> lat(double)
	// 5. Longitude(deg)		-> lon(double)
	// 6. Altitude(MSL, m)		-> current_global_altitude_ (float)
	// 7. Ax(m/s^2)				-> local_pose.ax (float)
	// 8. Ay(m/s^2)				-> local_pose.ay (float)
	// 9. Az(m/s^2)				-> local_pose.az (float)
	// 10. Roll					-> current_roll_ (float)
	// 11. Pitch				-> current_pitch_ (float)
	// 12. Yaw					-> current_yaw_ (float)

	// save_setpoint_local(SPT_FILE_PATH, current_timestamp_vehicle, i, set, local3);
	// save_object_pixel(PIXEL_FILE_PATH, current_timestamp_vehicle, -local_pose.z, yolo_score, current_object_pixel.x, current_object_pixel.y);
	// save_altitude(ALT_FILE_PATH, current_timestamp_vehicle, current_altitude_, current_global_altitude_, local_pose.ref_alt, -local_pose.z);
}
// ====================================================================================

/**
 * @brief service status ???
 */
void OffboardControl::response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future) 
{
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
	  auto reply = future.get()->reply;
	  service_result_ = reply.result;
      switch (service_result_)
		{
		case reply.VEHICLE_CMD_RESULT_ACCEPTED:
			RCLCPP_INFO(this->get_logger(), "command accepted");
			break;
		case reply.VEHICLE_CMD_RESULT_TEMPORARILY_REJECTED:
			RCLCPP_WARN(this->get_logger(), "command temporarily rejected");
			break;
		case reply.VEHICLE_CMD_RESULT_DENIED:
			RCLCPP_WARN(this->get_logger(), "command denied");
			break;
		case reply.VEHICLE_CMD_RESULT_UNSUPPORTED:
			RCLCPP_WARN(this->get_logger(), "command unsupported");
			break;
		case reply.VEHICLE_CMD_RESULT_FAILED:
			RCLCPP_WARN(this->get_logger(), "command failed");
			break;
		case reply.VEHICLE_CMD_RESULT_IN_PROGRESS:
			RCLCPP_WARN(this->get_logger(), "command in progress");
			break;
		case reply.VEHICLE_CMD_RESULT_CANCELLED:
			RCLCPP_WARN(this->get_logger(), "command cancelled");
			break;
		default:
			RCLCPP_WARN(this->get_logger(), "command reply unknown");
			break;
		}
      service_done_ = true;
    } else {
    RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
}


int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;

	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>("/fmu/"));
	rclcpp::shutdown();
	return 0;
}

