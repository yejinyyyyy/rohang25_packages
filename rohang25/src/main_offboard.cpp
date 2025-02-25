#include <px4_msgs/msg/offboard_control_mode.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>
#include <px4_msgs/msg/vehicle_command.hpp>
#include <px4_msgs/msg/airspeed.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <px4_msgs/msg/sensor_gps.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cmath>
#include <chrono>
#include <iostream>

using namespace std::chrono;
using namespace std::chrono_literals;
using namespace px4_msgs::msg;

class OffboardControl : public rclcpp::Node
{
public:
	OffboardControl() : Node("offboard_control"),
		offboard_setpoint_counter_(0), log_counter_(0),
		current_confidence_(0.0f), current_timestamp_(0), current_timestamp_sample_(0),
		current_indicated_airspeed_(0.0f), current_true_airspeed_(0.0f),
		current_roll_(0.0f), current_pitch_(0.0f), current_yaw_(0.0f), current_heading_(0.0f),
		current_latitude_(0.0f), current_longitude_(0.0f), current_altitude_(0.0f)
	{
		rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		qos_profile.history = RMW_QOS_POLICY_HISTORY_KEEP_LAST;
		qos_profile.depth = 10;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, qos_profile.depth), qos_profile);

		offboard_control_mode_publisher_ = this->create_publisher<OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
		trajectory_setpoint_publisher_ = this->create_publisher<TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
		vehicle_command_publisher_ = this->create_publisher<VehicleCommand>("/fmu/in/vehicle_command", 10);

		subscription_airspeed = this->create_subscription<Airspeed>(
		    "/fmu/out/airspeed", qos,
		    [this](const Airspeed::SharedPtr msg) { this->listener_callback(msg); });

		subscription_vehicle_attitude = this->create_subscription<VehicleAttitude>(
		    "/fmu/out/vehicle_attitude", qos,
		    [this](const VehicleAttitude::SharedPtr msg) { this->listener_callback_attitude(msg); });

		subscription_sensor_gps = this->create_subscription<SensorGps>(
		    "/fmu/out/vehicle_gps_position", qos,
		    [this](const SensorGps::SharedPtr msg) { this->listener_callback_gps(msg); });

		timer_ = this->create_wall_timer(100ms, [this]() -> void {
			if (offboard_setpoint_counter_ == 10) {
				this->publish_vehicle_command(VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
				this->arm();
			}
			publish_offboard_control_mode();
			publish_trajectory_setpoint();
			if (offboard_setpoint_counter_ < 11) {
				offboard_setpoint_counter_++;
			}

			log_counter_++;
			if (log_counter_ % 10 == 0) {
				RCLCPP_INFO(this->get_logger(),
				            "\n========== SENSOR STATE ==========\n"
				            "Airspeed:\n"
				            "  Indicated: %.2f m/s, True: %.2f m/s\n"
				            "Attitude:\n"
				            "  Roll: %.2f, Pitch: %.2f, Heading: %.2f°\n"
				            "GPS:\n"
				            "  Latitude: %.7f, Longitude: %.7f, Altitude: %.2f m\n"
				            "==================================",
				            current_indicated_airspeed_, current_true_airspeed_,
				            current_roll_, current_pitch_, current_heading_,
				            current_latitude_, current_longitude_, current_altitude_);
			}
		});
	}

	void arm();
	void disarm();

private:
	rclcpp::Publisher<OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
	rclcpp::Publisher<TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
	rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;

	rclcpp::Subscription<Airspeed>::SharedPtr subscription_airspeed;
	rclcpp::Subscription<VehicleAttitude>::SharedPtr subscription_vehicle_attitude;
	rclcpp::Subscription<SensorGps>::SharedPtr subscription_sensor_gps;

	rclcpp::TimerBase::SharedPtr timer_;
	uint64_t offboard_setpoint_counter_;
	uint64_t log_counter_;

	float current_confidence_;
	uint64_t current_timestamp_;
	uint64_t current_timestamp_sample_;
	float current_indicated_airspeed_;
	float current_true_airspeed_;
	float current_roll_;
	float current_pitch_;
	float current_yaw_;
	float current_heading_;
	float current_latitude_;
	float current_longitude_;
	float current_altitude_;

	void publish_offboard_control_mode();
	void publish_trajectory_setpoint();
	void publish_vehicle_command(uint16_t command, float param1 = 0.0f, float param2 = 0.0f);

	void listener_callback(const Airspeed::SharedPtr msg);
	void listener_callback_attitude(const VehicleAttitude::SharedPtr msg);
	void listener_callback_gps(const SensorGps::SharedPtr msg);
};

void OffboardControl::arm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0f);
	RCLCPP_INFO(this->get_logger(), "Arm command sent");
}

void OffboardControl::disarm()
{
	publish_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0f);
	RCLCPP_INFO(this->get_logger(), "Disarm command sent");
}

void OffboardControl::publish_offboard_control_mode()
{
	OffboardControlMode msg{};
	msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void OffboardControl::publish_trajectory_setpoint()
{
	float radius = 10.0f;
	float angular_velocity = 0.1f;
	static float altitude = -5.0f;
	static float angle = 0.0f;
	static bool isFlying = false;
	static bool isDone = false;

	if (altitude < -5.0f) {
		altitude += 0.1f;
		isFlying = false;
	} else {
		isFlying = true;
	}

	TrajectorySetpoint msg{};
	if (isFlying && !isDone) {
		angle += angular_velocity * 0.1f;
		float x = radius * std::cos(angle);
		float y = radius * std::sin(angle);
		msg.position = {x, y, -5.0f};
		if (angle >= 2 * M_PI) {
			isDone = true;
		}
	} else {
		msg.position = {0.0f, 0.0f, altitude};
	}
	msg.yaw = -3.14f;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
}

void OffboardControl::publish_vehicle_command(uint16_t command, float param1, float param2)
{
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
	vehicle_command_publisher_->publish(msg);
}

void OffboardControl::listener_callback(const Airspeed::SharedPtr msg)
{
	current_confidence_ = msg->confidence;
	current_timestamp_ = msg->timestamp;
	current_timestamp_sample_ = msg->timestamp_sample;
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

	current_heading_ = current_yaw_ * (180.0f / M_PI);
	if (current_heading_ < 0) {
		current_heading_ += 360.0f;
	}
}

void OffboardControl::listener_callback_gps(const SensorGps::SharedPtr msg)
{
	current_latitude_  = msg->latitude_deg;
	current_longitude_ = msg->longitude_deg;
	current_altitude_  = msg->altitude_msl_m;
}

int main(int argc, char *argv[])
{
	std::cout << "Starting offboard control node..." << std::endl;
	setvbuf(stdout, NULL, _IONBF, BUFSIZ);
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<OffboardControl>());
	rclcpp::shutdown();
	return 0;
}

