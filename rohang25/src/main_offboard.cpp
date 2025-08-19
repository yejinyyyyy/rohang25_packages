/*
/ 2025 한국로봇항공기대회
/ 건국대학교 ASEC
*/

#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/srv/vehicle_command.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/airspeed.hpp>//
#include <px4_msgs/msg/vehicle_attitude.hpp>//
#include <px4_msgs/msg/sensor_gps.hpp>
#include <px4_msgs/msg/vehicle_status.hpp> 
#include <rclcpp/rclcpp.hpp>

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <chrono>
#include <iostream>

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
		service_result_{0}, service_done_{false}, nav_state_(0), arm_state_(1), allow_run_(false), latched_stop_(false), did_mode_arm_(false)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		qos_profile.depth = 20;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		subscription_local_position = this->create_subscription<VehicleLocalPosition>(
		    "/fmu/out/vehicle_local_position", qos,
		    [this](const VehicleLocalPosition::SharedPtr msg) { this->listener_callback_local_position(msg); });

		subscription_airspeed = this->create_subscription<Airspeed>(
		    "/fmu/out/airspeed", qos,
		    [this](const Airspeed::SharedPtr msg) { this->listener_callback_airspeed(msg); });

		subscription_vehicle_status = this->create_subscription<VehicleStatus>(
			"/fmu/out/vehicle_status", qos,
			[this](const px4_msgs::msg::VehicleStatus::SharedPtr msg) { this->listener_callback_vehicle_status(msg); });

		// subscription_vehicle_attitude = this->create_subscription<VehicleAttitude>(
		//     "/fmu/out/vehicle_attitude", qos,
		//     [this](const VehicleAttitude::SharedPtr msg) { this->listener_callback_attitude(msg); });

		subscription_sensor_gps = this->create_subscription<SensorGps>(
		    "/fmu/out/vehicle_gps_position", qos,
		    [this](const SensorGps::SharedPtr msg) { this->listener_callback_gps(msg); });

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

private:

	rclcpp::Client<px4_msgs::srv::VehicleCommand>::SharedPtr vehicle_command_client_;

	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<VehicleLocalPosition>::SharedPtr subscription_local_position;
	rclcpp::Subscription<Airspeed>::SharedPtr subscription_airspeed;
	// rclcpp::Subscription<VehicleAttitude>::SharedPtr subscription_vehicle_attitude;
	rclcpp::Subscription<SensorGps>::SharedPtr subscription_sensor_gps;
	rclcpp::Subscription<VehicleStatus>::SharedPtr subscription_vehicle_status;

	rclcpp::TimerBase::SharedPtr timer_;
	
	enum class State{  // only for arm & offboard mode at start
		init,
		offboard_requested,
		offboard_running,
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

	int nav_state_;
	int arm_state_;
	bool allow_run_;
	bool latched_stop_;
	bool did_mode_arm_;

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

	
	int process = 0;
    int i = 0;
    
	bool control_mode[3] = {true, false, false}; // 0: position, 1: velocity, 2: acceleration
    bool hold_flag = false;
    bool hold_flag2 = false;
	bool gps_flag = false;

    float step = MC_SPEED;
	double vel_step = FW_SPEED;  // initial value = cruise speed (FW_SPEED)
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
	void timer_callback(void);

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

	void listener_callback_local_position(const VehicleLocalPosition::SharedPtr msg);
	void listener_callback_airspeed(const Airspeed::SharedPtr msg);
	// void listener_callback_attitude(const VehicleAttitude::SharedPtr msg);
	void listener_callback_gps(const SensorGps::SharedPtr msg);
	void listener_callback_vehicle_status(const VehicleStatus::SharedPtr msg);
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

/**
 * @brief Set control mode (*position control)
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
 * @brief Publish vehicle commands
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
 * @brief Listener callback functions
 */

void OffboardControl::listener_callback_local_position(const VehicleLocalPosition::SharedPtr msg)
{
	local_pose = *msg;

	current_heading_ = msg->heading * (180.0f / M_PI);
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

		WPT[7][0] = home_global[0];  
		WPT[7][1] = home_global[1];  // save Vertiport coords

		for (int i=0; i<8; i++)
		{
			WPT[i][2] = WPT[i][2] + home_global[2];
		}

		mission_llh2enu(WPT, home_global);
		mission_calib_local(WPT, home_local);

		gps_flag = true;
	}
}

void OffboardControl::listener_callback_vehicle_status(const VehicleStatus::SharedPtr msg)
{
	// Offboard → 이탈 시 파일럿 개입으로 간주해 래치(재진입 금지)
	if (!latched_stop_ && nav_state_ == 14 && msg->nav_state != 14) {
		latched_stop_ = true;
		allow_run_ = false;
		RCLCPP_WARN(this->get_logger(), "Pilot override detected. Stop all commands.");
	} else if (!latched_stop_) {
		allow_run_ = (msg->nav_state == 14);  // is offboard?
	}
	nav_state_ = msg->nav_state;
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
	// set_position(pose, {local_pose.y, local_pose.x, -local_pose.z}); 

	/******************************
 	*   Calculate Setpoint  	  *
 	******************************/

	if(state_ == State::offboard_running){ 
		switch(process)
		{
		case 0:
			// i=0; Takeoff
			set = {local_pose.y, local_pose.x, WPT[i][2]};
			
			set_position(pose, set);
			set_heading(pose, current_yaw_);
			set_velocity(pose, {NAN, NAN, NAN});  // init

			if(is_arrived_verti(local_pose, WPT[i], v_err)){
							
				if (hold_flag == false)
    				hold_flag = true;
				if(hold_flag){
					// ================  set heading towards WP1 ===================
					heading = get_angle({local_pose.y, local_pose.x}, {WPT[i+1][0], WPT[i+1][1]});
					set_heading(pose, heading);

					if(is_arrived_direc(local_pose, heading, a_err)){
						if(hold_flag2 == false && hold(1))
							hold_flag2 = true;
						if(hold_flag2){
							RCLCPP_INFO(this->get_logger(), "Take off Complete");
							process++; // 다음 단계
							// i++;

							hold_flag = false;
							hold_flag2 = false;
						}
					}
				}
			}

			break;
		
		case 1: // WPT1 and Hover
			// i=0; WPT#1 MC
			
			// start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(local, end, local, 2*step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_position(pose, set);
			if( is_arrived_hori(local_pose, WPT[i], h_err) && is_arrived_verti(local_pose, WPT[i], v_err) || is_increase_dist(remain_dist(local_pose, WPT[i])) ){
				set_position(pose, WPT[i]);
				if(hold_flag == false)
					hold_flag = true;
				if(hold_flag){
					start = {WPT[i][0], WPT[i][1]};
					end = {WPT[i+1][0], WPT[i+1][1]};
					heading = get_angle(start, end);
					set_heading(pose, heading);
					
					if(is_arrived_direc(local_pose, heading, a_err)){
						// if(hold_flag2 == false && hold(1.5))
						// 	hold_flag2 = true;
						// if(hold_flag2){
							RCLCPP_INFO(this->get_logger(), "===================== Arrived at WPT1");
							process++;
							i++;

							hold_flag = false;
							hold_flag2 = false;
							// }
					}
				}  
			}
			break;

		case 2: // Transition to FW
		// i=1;
		#ifndef QUAD_MODE
			// ================ Transition while flying ===================	
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(local, end, local, 15);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_position(pose, set);
			set_vel = velocity_guidance(local, set, vel_step);

			set_position(pose, WPT[i]);
			set_velocity(pose, set_vel);

			if(hold_flag == false && hold(2))
				hold_flag = true;
			if(hold_flag){
				do_vtol_transition();	
				if(state_ == State::transition_requested){
					state_ = State::offboard_running;
					current_flight_mode = FW;
					step = FW_SPEED;
					h_err = FW_HORIZONTAL_ERROR;
					v_err = FW_VERTICAL_ERROR;

					RCLCPP_INFO(this->get_logger(), "Transition FW ===========");
					process++;
				}
			}
			break;
		#else
			process++; //pass
		#endif
		

		case 3: // to WPT2
			// ROS_INFO("fly to WPT2..");
			// i=1; WPT#2

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				RCLCPP_INFO(this->get_logger(), "================== Arrived at WPT2 ");
				process++;
				i++;

				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 5: arrived at WPT2
				//    save_timestamp(FILE_PATH, log_index++, timestamp); // 6: P turn start
			}
			break;

		case 4: // to WP3
			// ROS_INFO("fly to WP3..");
			// i=2; WPT#3
			// control_mode[0] = false; // position control off, velocity control on, attitude control off
			// control_mode[1] = true;	
			
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				
				RCLCPP_INFO(this->get_logger(), "================== Arrived at WPT3 ");
				process++;
				// i++;
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 9: arrived at WPT3
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 10: P turn start
			}
			break;


		case 5: // P turn
			// ROS_INFO("P turn start..");
			// i=2; WPT#3

			// start = {WPT[i-1][0], WPT[i-1][1]};
			// end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = Pturn_guidance(WPT[i-1], WPT[i], WPT[i+1], PTURN_RADIUS, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(hold_flag == false && hold(1)){ // 3번 경로점을 충분히 지나갈 때까지 대기
				hold_flag = true;
			}

			if(hold_flag == true){
				if(is_arrived_direc(local_pose, get_angle(WPT[i], WPT[i+1]), a_err*2)){
				RCLCPP_INFO(this->get_logger(), "================= Arrived at switch point "); // P턴 원 탈출 지점 도달

				process++;
				i++;
				hold_flag = false;

				// save_timestamp(FILE_PATH, log_index++, timestamp); // 7: arrived at P end
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 8: move to WPT3
				}
			}
			
			break;


		case 6: // to WPT4
			// ROS_INFO("fly to WPT4..");
			// i=3; WPT#4
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};  // 좌표로 고도 반영
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				RCLCPP_INFO(this->get_logger(), "=================== Arrived at WPT4 ");  // 고도 조건 추가??
				process++;
				//i++;

				// save_timestamp(FILE_PATH, log_index++, timestamp); // 13: arrived at WP4
				//save_timestamp(FILE_PATH, log_index++, timestamp); // 14: move to WPT5
			}
			break;

		case 7: // P turn

			// ROS_INFO("P turn start..");
			// i=3; WPT#4
			start = {WPT[i-1][0], WPT[i-1][1]};
			middle = {WPT[i][0], WPT[i][1]};
			end = {WPT[i+1][0], WPT[i+1][1]};

			local = {local_pose.y, local_pose.x};
			set = Pturn_guidance(start, middle, end, PTURN_RADIUS, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]} ; // set is NED, changed sign
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(hold_flag == false && hold(1)) 
				hold_flag = true;

			if(hold_flag == true){
				if(is_arrived_direc(local_pose, get_angle(WPT[i], WPT[i+1]), a_err*2)){
				RCLCPP_INFO(this->get_logger(), "================= Arrived at switch point "); // P턴 원 탈출 지점 도달

				process++;
				i++;
				hold_flag = false;
				hold_flag2 = false;

				// save_timestamp(FILE_PATH, log_index++, timestamp); // 7: arrived at P end
				// save_timestamp(FILE_PATH, log_index++, timestamp); // 8: move to WPT3
				}
			}
			break;




		case 8: // to WP5
			// ROS_INFO("fly to WPT5..");
			// i=4; WPT#5

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			vel_step = 16.5;
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(is_arrived_hori(local_pose, WPT[i], v_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				RCLCPP_INFO(this->get_logger(), "================== Arrived at WPT5 "); // 고도 조건 추가??
				process++;
				i++;

				// save_timestamp(FILE_PATH, log_index++, timestamp); // 15: arrived at WP5
				//save_timestamp(FILE_PATH, log_index++, timestamp); // 16: move to WP6
			}
			break;


		// case 9: // P turn

		// 	// ROS_INFO("P turn start..");
		// 	// i=4; WPT#5
		// 	start = {WPT[i-1][0], WPT[i-1][1]};
		// 	middle = {WPT[i][0], WPT[i][1]};
		// 	end = {WPT[i+1][0], WPT[i+1][1]};

		// 	local = {local_pose.y, local_pose.x};
		// 	set = Pturn_guidance(start, middle, end, PTURN_RADIUS, local, step);
		// 	set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]} ; // set is NED, changed sign
		// 	set_vel = velocity_guidance(local, set, 17);
			
		// 	set_position(pose, set);
		// 	set_velocity(pose, set_vel);

		// 	if(hold_flag == false && hold(1)) // 3번 경로점을 충분히 지나갈 때까지 대기
		// 		hold_flag = true;

		// 	if(hold_flag == true){
		// 		if(is_arrived_direc(local_pose, get_angle(WPT[i], WPT[i+1]), a_err)){
		// 		RCLCPP_INFO(this->get_logger(), "================= Arrived at switch point "); // P턴 원 탈출 지점 도달

		// 		process++;
		// 		i++;
		// 		hold_flag = false;
		// 		hold_flag2 = false;

		// 		// save_timestamp(FILE_PATH, log_index++, timestamp); // 7: arrived at P end
		// 		// save_timestamp(FILE_PATH, log_index++, timestamp); // 8: move to WPT3
		// 		}
		// 	}
		// 	break;

			

		case 9: // to WP6

			// ROS_INFO("fly to WPT6..");
			// i=5; WPT#6

			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			vel_step = vel_step - 0.07; // slow down before transition 1m/s^2 from 18m/s
			set_vel = velocity_guidance(local, set, vel_step);
			
			set_position(pose, set);
			set_velocity(pose, set_vel);

			if(remain_dist(local_pose, WPT[i]) < BACK_TRANSITION_DIST){
				RCLCPP_INFO(this->get_logger(), "================= Back Transition Start ");
				process++;
			}

			
			break;

		case 10: // Transition to MC /i=5
			#ifndef QUAD_MODE
				// ============ set heading towards WP6 =============
				start = {WPT[i-1][0], WPT[i-1][1]};
				end = {WPT[i][0], WPT[i][1]};
				local = {local_pose.y, local_pose.x};
				set = line_guidance(start, end, local, step);
				set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
				vel_step = vel_step - 0.05; // slow down before transition 17 -> 15m/s
				set_vel = velocity_guidance(local, set, vel_step);
				heading = get_angle(start, end);
				
				set_heading(pose, heading);   
				set_position(pose, set);
				// set_velocity(pose, {NAN, NAN, NAN});
				set_velocity(pose, set_vel);
				
				do_back_transition();

				if(state_ == State::transition_requested){
					state_ = State::offboard_running;
					current_flight_mode = MC;
					step = MC_SPEED;
					h_err = MC_HORIZONTAL_ERROR;
					v_err = MC_VERTICAL_ERROR;
	
					RCLCPP_INFO(this->get_logger(), "================ Transition to MC ");
					// set_position(pose, WPT[i]);
					process++;
					//i++;
				}


				break;
			#else
				process++; //pass
			#endif	

		case 11: 
			// ROS_INFO("fly to WPT6..");
			// i=5; WPT#6
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			vel_step = vel_step - 0.1; // slow down before transition 17 -> 15m/s
			set_vel = velocity_guidance(local, set, vel_step);
			heading = get_angle({local_pose.y, local_pose.x}, {WPT[i+1][0], WPT[i+1][1]});

			if(vel_step < 3.0) {
				vel_step = 3;
				set_vel = {NAN, NAN, NAN}; // stop before WPT6
			}

			set_position(pose, WPT[i]);
			set_velocity(pose, set_vel);
			set_heading(pose, heading);
			
			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				
				// if(hold_flag == false && hold(1))
    			// 	hold_flag = true;
				// if(hold_flag){

					if(is_arrived_direc(local_pose, heading, a_err)){
						if(hold_flag2 == false && hold(1))
							hold_flag2 = true;
						if(hold_flag2){
							RCLCPP_INFO(this->get_logger(), "=================== Arrived at WPT6 ");
							process++; 
							i++;

							hold_flag = false;
							hold_flag2 = false;
						}
					}
				// }
			}
			break;

		case 12: 
			// ROS_INFO("fly to WPT7..");
			// i=6; WPT#7
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_position(pose, set);
			
			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){
				
				if(hold_flag == false && hold(1))
    				hold_flag = true;
				if(hold_flag){
					heading = get_angle({local_pose.y, local_pose.x}, {WPT[i+1][0], WPT[i+1][1]});
					set_heading(pose, heading);

					if(is_arrived_direc(local_pose, heading, a_err)){
						if(hold_flag2 == false && hold(1))
							hold_flag2 = true;
						if(hold_flag2){
							RCLCPP_INFO(this->get_logger(), "=================== Arrived at WPT7 ");
							process++; 
							i++;

							hold_flag = false;
							hold_flag2 = false;
						}
					}
				}
			}
				
			break;




		case 13: // to WP8
			// ROS_INFO("fly to WP8..");
			// i=7; WPT#8
			start = {WPT[i-1][0], WPT[i-1][1]};
			end = {WPT[i][0], WPT[i][1]};
			local = {local_pose.y, local_pose.x};
			set = line_guidance(start, end, local, step);
			set = {local_pose.y + set[0], local_pose.x + set[1], WPT[i][2]};
			set_position(pose, set);
			
			if(is_arrived_hori(local_pose, WPT[i], h_err) || is_increase_dist(remain_dist(local_pose, WPT[i]))){

				if(hold_flag2 == false && hold(1))
					hold_flag2 = true;
				if(hold_flag2){
					RCLCPP_INFO(this->get_logger(), "=================== Arrived at Base ");
					process++; 
					// i++;

					hold_flag = false;
					hold_flag2 = false;
				}
			}
			
			break;
			

		case 14: 
			// Land
		
			land();

			if(state_ == State::land_requested)
			{
			RCLCPP_INFO(this->get_logger(), "================ Auto Land enabled ");
			process++;
			}
			
			break;
		
		}
	}
	/******************************
 	*  Print & Log current info   *
 	******************************/
    remain = remain_dist(local_pose, WPT[i]);
	save_setpoint_local(SPT_FILE_PATH, current_timestamp_, i, {pose.position[0], pose.position[1], pose.position[2]}, local3);

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
 
	if (latched_stop_) return; // ★ 파일럿 개입 후엔 완전 정지
	if (!allow_run_) return; // Offboard가 아니면 아무 것도 보내지 않음
	if (offboard_setpoint_counter_ < 11) {  // 카운터 갱신
		offboard_setpoint_counter_++;
	}

	/*************************************
 	*  	Service state (Arming sequence)  *
 	**************************************/		
	switch (state_)
	{
	case State::init :
		if (arm_state_ = 2) { // 1: disarmed, 2: armed,
			RCLCPP_INFO(this->get_logger(), "Vehicle is armed");
			state_ = State::armed;
		} 
	break;
	case State::armed :
		if (!did_mode_arm_ && allow_run_ && offboard_setpoint_counter_ == 10) { // ★ 최초 1회만 모드/ARM (기존 1초 지연 유지)
			did_mode_arm_ = true;
			RCLCPP_INFO(this->get_logger(), "Offboard mode initiated");
			state_ = State::offboard_running;
		}
	break;
	// case State::offboard_requested :
	// 	if(service_done_){
	// 		if (service_result_==0){
	// 			RCLCPP_INFO(this->get_logger(), "Entered offboard mode");
	// 			state_ = State::wait_for_stable_offboard_mode;				
	// 		}
	// 		else{
	// 			RCLCPP_ERROR(this->get_logger(), "Failed to enter offboard mode, exiting");
	// 			rclcpp::shutdown();
	// 		}
	// 	}
	// 	break;
	// case State::wait_for_stable_offboard_mode :
	// 	if (++num_of_steps>10){
	// 		arm();
	// 		state_ = State::arm_requested;
	// 	}
	// 	break;
	// case State::arm_requested :
	// 	if(service_done_){
	// 		if (service_result_==0){
	// 			RCLCPP_INFO(this->get_logger(), "vehicle is armed");
	// 			state_ = State::armed;
	// 		}
	// 		else{
	// 			RCLCPP_ERROR(this->get_logger(), "Failed to arm, exiting");
	// 			rclcpp::shutdown();
	// 		}
	// 	}
	// 	break;
	case State::transition_requested :
		if(service_done_){
			if (service_result_==0){
				RCLCPP_INFO(this->get_logger(), "Transition success");
					state_ = State::offboard_running;
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
 	*  	Publishes Setpoint here   *
 	******************************/
	publish_offboard_control_mode();
	publish_trajectory_setpoint(); 	// offboard_control_mode needs to be paired with trajectory_setpoint


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
					"Current velocity: %.2f m/s\n"
					"Velocity setpoint: %.2f, %.2f / %.1f m/s\n"
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
					sqrt(local_pose.vx*local_pose.vx+local_pose.vy*local_pose.vy),
					pose.velocity[0], pose.velocity[1], vel_step,
					remain, i+1, process
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

