#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <px4_msgs/msg/trajectory_setpoint.hpp>

#include "config.h"
#include "algebra.h"

extern int current_flight_mode;

void set_position(px4_msgs::msg::TrajectorySetpoint &pose, std::vector<double> coor);
void set_heading(px4_msgs::msg::TrajectorySetpoint &pose, double heading);
void set_altitude(px4_msgs::msg::TrajectorySetpoint &pose, double alt);
void set_velocity(px4_msgs::msg::TrajectorySetpoint &pose, std::vector<double> vel);
bool hold(double seconds);
bool is_arrived_hori(px4_msgs::msg::VehicleLocalPosition local, std::vector<double> set, double hori_err);
bool is_arrived_verti(px4_msgs::msg::VehicleLocalPosition local, std::vector<double> set, double verti_err);
// std::vector<double> quat2RPY(double x, double y, double z, double w);
// double get_current_heading(VehicleLocalPosition my_pos);
bool is_arrived_direc(px4_msgs::msg::VehicleLocalPosition my_pos, double angle, double a_err);
double next_waypoint_heading(std::vector<double> start, std::vector<double> end);
std::vector<double> next_waypoint_unitvector(std::vector<double> start, std::vector<double> end);
double remain_dist(px4_msgs::msg::VehicleLocalPosition my_pos, std::vector<double> wpt);
bool is_increase_dist(double dist);
// void update_posestamp_header(TrajectorySetpoint &pose);


#endif
