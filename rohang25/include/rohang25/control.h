#ifndef __CONTROL_H__
#define __CONTROL_H__

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "config.h"
#include "algebra.h"

extern int current_flight_mode;

void set_position(geometry_msgs::PoseStamped &pose, std::vector<double> coor);
void set_heading(geometry_msgs::PoseStamped &pose, double heading);
void set_altitude(geometry_msgs::PoseStamped &pose, double alt);
bool hold(double seconds);
bool is_arrived_hori(geometry_msgs::PoseStamped local, std::vector<double> set, double hori_err);
bool is_arrived_verti(geometry_msgs::PoseStamped local, std::vector<double> set, double verti_err);
std::vector<double> quat2RPY(double x, double y, double z, double w);
double get_current_heading(geometry_msgs::PoseStamped my_pos);
bool is_arrived_direc(geometry_msgs::PoseStamped my_pos, double angle, double a_err);
double next_waypoint_heading(std::vector<double> start, std::vector<double> end);
std::vector<double> next_waypoint_unitvector(std::vector<double> start, std::vector<double> end);
double remain_dist(geometry_msgs::PoseStamped my_pos, std::vector<double> wpt);
bool is_increase_dist(double dist);
void update_posestamp_header(geometry_msgs::PoseStamped &pose);


#endif