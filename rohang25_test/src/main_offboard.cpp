/*
/ 2025 한국로봇항공기대회
/ 건국대학교 ASEC
/
/ Code for Mission Testing
/ 
/ Last edit date : 250605
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/airspeed.hpp>//
#include <px4_msgs/msg/vehicle_attitude.hpp>//
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>
#include <algorithm> 

#include "algebra.h"
#include "config.h"
#include "control.h"
#include "frame.h"
#include "global_def.h"
#include "guidance.h"
#include "mission.h"
#include "log.h"
#include "guidance/guidance_param.h"

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;
using std::placeholders::_1;
using std::placeholders::_2;

int current_flight_mode = MC;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl(std::string px4_namespace) : Node("offboard_control"),
		log_counter_(0),
		current_confidence_(0.0f), current_timestamp_(0), current_timestamp_sample_(0),
		current_indicated_airspeed_(0.0f), current_true_airspeed_(0.0f),
		current_roll_(0.0f), current_pitch_(0.0f), current_yaw_(0.0f), current_heading_(0.0f),
		current_latitude_(0.0f), current_longitude_(0.0f), current_altitude_(0.0f), state_{State::init},
		service_result_{0}, service_done_{false}
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		qos_profile.depth = 20;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		geo_node_control_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/geo_node_control_flag", 10);
		gimbal_node_control_flag_publisher_ = this->create_publisher<std_msgs::msg::Bool>("/gimbal_node_control_flag", 10);

		subscription_local_position = this->create_subscription<VehicleLocalPosition>(
		    "/fmu/out/vehicle_local_position", qos,
		    [this](const VehicleLocalPosition::SharedPtr msg) { this->listener_callback_local_position(msg); });

		subscription_airspeed = this->create_subscription<Airspeed>(
		    "/fmu/out/airspeed", qos,
		    [this](const Airspeed::SharedPtr msg) { this->listener_callback_airspeed(msg); });

		subscription_vehicle_attitude = this->create_subscription<VehicleAttitude>(
		    "/fmu/out/vehicle_attitude", qos,
		    [this](const VehicleAttitude::SharedPtr msg) { this->listener_callback_attitude(msg); });

		subscription_sensor_gps = this->create_subscription<SensorGps>(
		    "/fmu/out/vehicle_gps_position", qos,
		    [this](const SensorGps::SharedPtr msg) { this->listener_callback_gps(msg); });
		
		subscription_object_ned = this->create_subscription<geometry_msgs::msg::Point32>(
			"/object_center_ned", qos,
			[this](const geometry_msgs::msg::Point32::SharedPtr msg) { this->listener_callback_object_ned(msg); });

		subscription_mission_flag = this->create_subscription<std_msgs::msg::Bool>(
			"/mission_flag", qos,
			[this](const std_msgs::msg::Bool::SharedPtr msg) { this->listener_callback_mission_flag(msg); });
		
		param_service_ = node_->create_service<guidance::srv::SetGuidanceParam>(
			"/guidance_param",
			std::bind(&guidance_param::srv_callback_paramUpdate, this, _1, _2));

		vehicle_command_client_{this->create_client<px4_msgs::srv::VehicleCommand>(px4_namespace+"vehicle_command")};

		RCLCPP_INFO_STREAM(this->get_logger(), "Waiting for " << px4_namespace << "vehicle_command service");
		while (!vehicle_command_client_->wait_for_service(1s)) {
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
				return;
			}
		}
		RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");

		timer_ = this->create_wall_timer(100ms, std::bind(&OffboardControl::timer_callback, this));


		RCLCPP_INFO(this->get_logger(), "\nMission Start =================== 0314 UPDATE\n");

		/******************************
		*  	Create Log file		  *
		******************************/
		create_file(SPT_FILE_PATH);

	};

	void switch_to_offboard_mode();
	void arm();
	void disarm();
	void do_vtol_transition();
	void do_back_transition();
	void land();

	double_t param[10]; // Precise landing guidance parameters
	// Initialize parameter values	
	param[0] = 0.0;   // control type - 0: PIXEL, 1: NED axis
	param[1] = 640.0; // goal position - X axis or North axis
	param[2] = 360.0; // goal position - Y axis or East axis
	param[3] = 0.01;  // p gain - X or North axis
	param[4] = 0.01;  // p gain - Y or East axis
	param[5] = 3.0;   // velocity saturation
	param[6] = 0.5;   // down velocity
	param[7] = 0.0;
	param[8] = 0.0;
	param[9] = 0.0;

private:

	rclcpp::Service<guidance::srv::SetGuidanceParam>::SharedPtr param_service_;

	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr geo_node_control_flag_publisher_;
	rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr gimbal_node_control_flag_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_local_position;
	rclcpp::Subscription<Airspeed>::SharedPtr subscription_airspeed;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr subscription_vehicle_attitude;
	rclcpp::Subscription<SensorGps>::SharedPtr subscription_sensor_gps;
	rclcpp::Subscription<geometry_msgs::msg::Point32>::SharedPtr subscription_object_ned;
	rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_mission_flag;

	rclcpp::TimerBase::SharedPtr timer_;
	
	enum class State{  // only for arm & offboard mode at start
		init,
		offboard_requested,
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

	float current_confidence_;
	uint64_t current_timestamp_;
	uint64_t current_timestamp_sample_;
	float current_indicated_airspeed_;
	float current_true_airspeed_;
	float current_roll_;
	float current_pitch_;
	float current_yaw_;
	double current_heading_; 
	float current_latitude_; // global position
	float current_longitude_;
	float current_altitude_;
	double heading;			 // for calculation inside loop

	VehicleLocalPosition local_pose{}; // local position -> subscribed
	TrajectorySetpoint pose{};  	   // local position -> to be published
	static geometry_msgs::msg::Point32  current_object_ned{}; // subscribed object ned

	int process = 0;
    int i = 0;
    
	bool control_mode[3] = {true, false, false}; // 0: position, 1: velocity, 2: attitude
    bool hold_flag = false;
    bool hold_flag2 = false;
	bool gps_flag = false;
	bool geo_node_control_flag = false;
	bool gimbal_node_control_flag = false;	
	bool mission_flag = false; 

    float step = MC_SPEED;
    double h_err = MC_HORIZONTAL_ERROR;
    double v_err = MC_VERTICAL_ERROR;
    double a_err = HEADING_ERROR;
    double DESC_RATE;

    std::vector<double> start;
    std::vector<double> middle;
    std::vector<double> end;
    std::vector<double> center;
    std::vector<double> local;
    std::vector<double> local3;
    std::vector<double> set = {0, 0, 0};
	std::vector<double> set_vel = {0, 0, 0};
	double remain;
	// std::vector<double> corr_alt;

	void request_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	void response_callback(rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedFuture future);
	void srv_callback_paramUpdate(
        const std::shared_ptr<guidance::srv::SetGuidanceParam::Request> request,
        std::shared_ptr<guidance::srv::SetGuidanceParam::Response> response);
	void timer_callback(void);

	void publish_offboard_control_mode(bool control_mode);
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);
	void publish_node_control_flag();
	
	void listener_callback_local_position(const VehicleLocalPosition::SharedPtr msg);
	void listener_callback_airspeed(const Airspeed::SharedPtr msg);
	void listener_callback_attitude(const VehicleAttitude::SharedPtr msg);
	void listener_callback_gps(const SensorGps::SharedPtr msg);
	void listener_callback_object_ned(const geometry_msgs::msg::Point32::SharedPtr msg);
};

/**
 * @brief Send a command to switch to offboard mode / arm & disarm / transition & back
 */
void OffboardControl::switch_to_offboard_mode(){
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
void guidance_param::callback_paramUpdate(
	const std::shared_ptr<guidance::srv::SetGuidanceParam::Request> request,
	std::shared_ptr<guidance::srv::SetGuidanceParam::Response> response)
  {
	uint8_t idx = request->index;
	double val = request->value;
	if (idx < 10) {
	  param[idx] = val;
	  response->success = true;
	  RCLCPP_INFO(node_->get_logger(), "[GUIDANCE] param[%u] set to %.3f", idx, val);
	} 
	else {
	  response->success = false;
	  RCLCPP_WARN(node_->get_logger(), "[GUIDANCE] invalid index %u", idx);
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
	current_timestamp_ = msg->timestamp;
	current_timestamp_sample_ = msg->timestamp_sample;
	current_indicated_airspeed_ = msg->indicated_airspeed_m_s;
	current_true_airspeed_ = msg->true_airspeed_m_s;
}

// void OffboardControl::listener_callback_attitude(const VehicleAttitude::SharedPtr msg)
// {
// 	float w = msg->q[0];
// 	float x = msg->q[1];
// 	float y = msg->q[2];
// 	float z = msg->q[3];

// 	current_roll_  = std::atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + y * y));
// 	current_pitch_ = std::asin(2.0f * (w * y - z * x));
// 	current_yaw_   = std::atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (y * y + z * z));

// 	current_heading_ = current_yaw_ * (180.0f / M_PI);
// 	if (current_heading_ < 0) {
// 		current_heading_ += 360.0f;
// 	}
// }

void OffboardControl::listener_callback_gps(const SensorGps::SharedPtr msg)
{
	current_latitude_  = msg->latitude_deg;
	current_longitude_ = msg->longitude_deg;
	current_altitude_  = msg->altitude_msl_m;

	/******************************
	*  	Coordinates setting		  *
	******************************/

	if(FRAME_MODE == LLH && gps_flag == false ){  // wait GPS data

		std::vector<double> home_local = {local_pose.y, local_pose.x, -local_pose.z}; // NED -> ENU
		std::vector<double> home_global = {current_latitude_, current_longitude_, current_altitude_};

		RCLCPP_INFO(this->get_logger(), "global: %f, %f, %f", home_global[0], home_global[1], home_global[2]);
		RCLCPP_INFO(this->get_logger(), "local: %f, %f, %f", home_local[0], home_local[1], home_local[2]);

		WPT[8][0] = home_global[0];  
		WPT[8][1] = home_global[1];  // change later

		for (int i=0; i<9; i++)
		{
			WPT[i][2] = WPT[i][2] + home_global[2];
		}

		mission_llh2enu(WPT, home_global);
		mission_calib_local(WPT, home_local);

		gps_flag = true;
	}
}

void OffboardControl::listener_callback_object_ned(const geometry_msgs::msg::Point32::SharedPtr msg)
{
	current_object_ned = *msg;
}

void OffboardControl::listener_callback_mission_flag(const std_msgs::msg::Bool::SharedPtr msg)
{
	mission_flag = msg->data;
}


/**
 * @brief Set control mode 
 */
 void OffboardControl::publish_offboard_control_mode(bool control_mode)
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
	 std_msgs::msg::Bool geo_msg{};
	 std_msgs::msg::Bool gimbal_msg{};
	 geo_msg.data = geo_node_control_flag;
	 gimbal_msg.data = gimbal_node_control_flag;
	 geo_node_control_flag_publisher_->publish(geo_msg);
	 gimbal_node_control_flag_publisher_->publish(gimbal_msg);
 }

/**
 * @brief Publish local setpoint & heading ===========================================
 */
void OffboardControl::publish_trajectory_setpoint()	
{
	/******************************
 	*   Local position Update	  *
 	******************************/
	local3 = {local_pose.y, local_pose.x, -local_pose.z}; // ENU
	set_position(pose, {local_pose.y, local_pose.x, -local_pose.z}); 

	/******************************
 	*   Calculate Setpoint  	  *
 	******************************/

	if(state_ == State::armed){ 
		switch(process)
		{
		case 0:
			// i=0; Takeoff at Home
			set = {local_pose.y, local_pose.x, WPT[i][2]};
			set_position(pose, set);
			set_heading(pose, current_yaw_);

			if(is_arrived_verti(local_pose, WPT[i], v_err)){
							
				if (hold_flag == false)
    				hold_flag = true;
				if(hold_flag){
					================  set heading towards WP1 ===================
					start = {local_pose.pose.position.x,local_pose.pose.position.y};
					end = {WPT[i][0], WPT[i][1]};
					heading = get_angle({WPT[i+1][0], WPT[i+1][1]}, {WPT[i+2][0], WPT[i+2][1]});
					set_heading(pose, heading);

					if(is_arrived_direc(local_pose, heading, a_err)){
					    if(hold_flag2 == false && hold(1))
					        hold_flag2 = true;
					    if(hold_flag2){
							RCLCPP_INFO(this->get_logger(), "Take off Complete");
							process++; // 다음 단계
							i++;

							hold_flag = false;
							hold_flag2 = false;
						}
					}
				}
			}

			break;
		

		case 1: // to WPT1 (mission point)
			// i=1; WPT#1
			geo_node_control_flag = true;
			gimbal_node_control_flag = true; // Start searching target

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(local, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			
			set_position(pose, set);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				RCLCPP_INFO(this->get_logger(), "================== Arrived at WPT2 ");
				process++;

				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 5: arrived at WPT2
				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 6: P turn start
			}
			break;
		

		case 2: // Precise Landing
			// i=1; WPT#1 
		#ifndef MISSION_TEST_MODE
			if(detection_flag == true){
				RCLCPP_INFO(this->get_logger(), "================== Detected Marker, Start Landing");
				
				control_mode = {false, true, false}; // position control off, velocity control on, attitude control off
				set = {NaN, NaN, NaN}; 
				set_vel = precise_landing_guidance(param, current_heading_, current_object_ned);
				set_position(pose, set);
				set_velocity(pose, set_vel);

				if(current_altitude_ < 0){  // Check landing altitude
					RCLCPP_INFO(this->get_logger(), "================== Landing Complete");
					disarm();
				}
			}
			else{
				if(current_altitude_ > 5.0){
					RCLCPP_INFO(this->get_logger(), "================== Searching for Marker..");
					if(log_counter_ % 30 == 0){
						set = {WPT[i][0], WPT[i][1], -local_pose.z + 1}; // descend 1m
						set_position(pose, set);
					}
				}
				else{
					RCLCPP_INFO(this->get_logger(), "================== Cannot find marker, Auto-Land");
					
					land();

					if(state_ == State::land_requested)
					{
						RCLCPP_INFO(this->get_logger(), "================ Auto Land enabled ");
						process++;
					}
				}
			}

			break;
		#else
			process++; //pass
		#endif


		case 3: // Rescue mission
			// i=1; WPT#1
			if(detection_flag == true){
				RCLCPP_INFO(this->get_logger(), "================== Detected Target, Approaching");
				
				control_mode = {false, true, false}; // position control off, velocity control on, attitude control off
				set = {NaN, NaN, NaN}; 
				set_vel = precise_landing_guidance(param, current_heading_, current_object_ned);
				set_position(pose, set);
				set_velocity(pose, set_vel);

				if(current_altitude_ <= 2){  // Check mission altitude
					RCLCPP_INFO(this->get_logger(), "================== Waiting for mission");
					set_position(pose, local3); // hover until mission done manually

					if(mission_flag == true){  // manual input - service call or topic
						RCLCPP_INFO(this->get_logger(), "================== Mission Complete, continue ..");
						
						process++;
						detection_flag = false;
						mission_flag = false;
						control_mode = {true, false, false};

					}
				}
			}
			
			else{
				if(current_altitude_ > 5.0){
					RCLCPP_INFO(this->get_logger(), "================== Searching for Target..");
					if(log_counter_ % 30 == 0){
						set = {WPT[i][0], WPT[i][1], -local_pose.z + 1}; // descend 1m
						set_position(pose, set);
					}
				}
				else{
					RCLCPP_INFO(this->get_logger(), "================== Cannot find target, hovering");
					set_position(pose, local3); // set position to current local position
					/*
						Emergency Manual maneuver?
					*/
				}
			}

			break;


		case 4: // Return to WPT1
			// i=1; WPT#1

			// start = {WPT[i-1][0], WPT[i-1][1]};
			// end = {WPT[i][0], WPT[i][1]};
			set = {local_pose.y, local_pose.x, WPT[i][2]};
			set_position(pose, set);

			// hold X
			if(is_arrived_verti(local_pose, WPT[i], v_err)){
				RCLCPP_INFO(this->get_logger(), "================= Move to release point "); // P턴 원 탈출 지점 도달

				process++;
				i++;
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 7: arrived at P end
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 8: move to WPT3
			}
			
			break;

			
		case 5: // to WPT2 (Release point)
			// i=2; WPT#2
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]}; 
			// set_vel = velocity_guidance(local, set);  // acceleration guidance??
			
			set_position(pose, set);
			// set_velocity(pose, set_vel);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				RCLCPP_INFO(this->get_logger(), "=================== Arrived at release point ");  // 고도 조건 추가??
				process++;
				//i++;

				// save_timestamp(FILE_PATH, log_index++, timestamp); // 13: arrived at WP4
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 14: move to WPT5
			}
			break;


		case 7: // Relase Target
			// i=2; WPT#2
			if(detection_flag == true){
				RCLCPP_INFO(this->get_logger(), "================== Detected Marker, Approaching");
				
				control_mode = {false, true, false};
				set = {NaN, NaN, NaN}; 
				set_vel = precise_landing_guidance(param, current_heading_, current_object_ned);
				set_position(pose, set);
				set_velocity(pose, set_vel);

				if(current_altitude_ <= 2){  // Check mission altitude
					RCLCPP_INFO(this->get_logger(), "================== Waiting to release");
					set_position(pose, local3); // hover until mission done manually

					if(mission_flag == true){  // manual input - service call or topic
						RCLCPP_INFO(this->get_logger(), "================== Mission Complete, continue ..");
						
						detection_flag = false;
						mission_flag = false;
						process++;
						control_mode = {true, false, false};
					}
				}
			}
			
			else{
				if(current_altitude_ > 5.0){
					RCLCPP_INFO(this->get_logger(), "================== Searching for Marker..");
					if(log_counter_ % 30 == 0){
						set = {WPT[i][0], WPT[i][1], -local_pose.z + 1}; // descend 1m
						set_position(pose, set);
					}
				}
				else{
					RCLCPP_INFO(this->get_logger(), "================== Cannot find Marker, hovering");
					set_position(pose, local3); // set position to current local position
					/*
						Emergency Manual maneuver?
					*/
				}
			}

			break;


		case 8: // Return to WPT2
			// i=2; WPT#2

			// start = {WPT[i-1][0], WPT[i-1][1]};
			// end = {WPT[i][0], WPT[i][1]};
			set = {local_pose.y, local_pose.x, WPT[i][2]};  // Alt 30m or Custom
			set_position(pose, set);

			// hold X
			if(is_arrived_verti(local_pose, WPT[i], v_err)){
				RCLCPP_INFO(this->get_logger(), "================= Return to Home "); 

				process++;
				i++;
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 7: arrived at P end
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 8: move to WPT3
			}
			
			break;


		case 9: // to WPT3 (Vertiport)
			// i=3; WPT#3
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]}; 
			
			set_position(pose, set);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				RCLCPP_INFO(this->get_logger(), "=================== Arrived at Vertiport "); 
				process++;

				// save_timestamp(FILE_PATH, log_index++, timestamp); // 13: arrived at WP4
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 14: move to WPT5
			}
			break;
			

		case 10: // Precise Landing
			// i=3; WPT#3

			if(detection_flag == true){
				RCLCPP_INFO(this->get_logger(), "================== Detected Marker, Start Landing");
				
				control_mode = {false, true, false};
				set = {NaN, NaN, NaN}; 
				set_vel = precise_landing_guidance(param, current_heading_, current_object_ned);
				set_position(pose, set);
				set_velocity(pose, set_vel);

				if(current_altitude_ < 0){  // Check landing altitude
					RCLCPP_INFO(this->get_logger(), "================== Landing Complete");
					disarm();
				}
			}
			else{
				if(current_altitude_ > 5.0){
					RCLCPP_INFO(this->get_logger(), "================== Searching for Marker..");
					if(log_counter_ % 30 == 0){
						set = {WPT[i][0], WPT[i][1], -local_pose.z + 1}; // descend 1m
						set_position(pose, set);
					}
				}
				else{
					RCLCPP_INFO(this->get_logger(), "================== Cannot find marker, Auto-Land");
					
					land();

					if(state_ == State::land_requested)
					{
						RCLCPP_INFO(this->get_logger(), "================ Auto Land enabled ");
						process++;
					}
				}
			}

			break;

		}
	}
	/******************************
 	*  Print & Log current info   *
 	******************************/
    remain = remain_dist(local_pose, WPT[i]);
	save_setpoint_local(SPT_FILE_PATH, current_timestamp_, i, set, local3);

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
	switch (state_)
	{
	case State::init :
		switch_to_offboard_mode();
		state_ = State::offboard_requested;
		break;
	case State::offboard_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
				state_ = State::wait_for_stable_offboard_mode;				
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	case State::wait_for_stable_offboard_mode :
		if (++num_of_steps>10){
			arm();
			state_ = State::arm_requested;
		}
		break;
	case State::arm_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "vehicle is armed");
				state_ = State::armed;
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
				rclcpp::shutdown();
			}
		}
		break;
	case State::transition_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Transition success");
					state_ = State::armed;
			}
			else{
				RCLCPP_ERROR(this->get_logger(), "Transition Failed, exiting");
				rclcpp::shutdown();
			}
		}
		break;

	default:
		break;
	}

	/******************************
 	*  	Publishes Data here       *
 	******************************/
	publish_offboard_control_mode();
	publish_trajectory_setpoint(); 	// offboard_control_mode needs to be paired with trajectory_setpoint
	publish_node_control_flag();	// publish all node control flags 

	/****************************
 	*  	Print Sensor Data       *
 	*****************************/
	log_counter_++;
	if (log_counter_ % 5 == 0) {
		RCLCPP_INFO(this->get_logger(),
					"\n========== CURRENT STATE ==========\n"
					"Timestamp: %ld\n"
					"Current: %.2f, %.2f %.2f\n"
					"Setpoint: %.2f, %.2f, %.2f\n"
					"Current heading: %.2f / Set heading: %.2f\n"
					"Velocity setpoint:  %.2f, %.2f, %.2f\n"
					"Remain Dist: %f, to WPT#%d\n"
					"Case #%d\n"
					// "Airspeed:\n"
					// "  Indicated: %.2f m/s, True: %.2f m/s\n"
					// "Attitude:\n"
					// "  Roll: %.2f, Pitch: %.2f, Heading: %.2f°\n"
					// "GPS:\n"
					// "  Latitude: %.7f, Longitude: %.7f, Altitude: %.2f m\n"
					"==================================\n",
					current_timestamp_, local_pose.x, local_pose.y, local_pose.z, 
					pose.position[0], pose.position[1], pose.position[2],
					current_heading_, DEF_R2D(heading),
					pose.velocity[0], pose.velocity[1], pose.velocity[2],
					remain, i, process
					// current_indicated_airspeed_, current_true_airspeed_,
					// current_roll_, current_pitch_, current_heading_,
					// current_latitude_, current_longitude_, current_altitude_
				);
	}
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

