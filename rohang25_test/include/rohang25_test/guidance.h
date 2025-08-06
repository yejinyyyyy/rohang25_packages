#ifndef __GUIDANCE_H__
#define __GUIDANCE_H__

#include <geometry_msgs/msg/point32.hpp>
#include <math.h>
#include <algorithm> 

#include "global_def.h"
#include "config.h"
#include "algebra.h"

double get_angle(std::vector<double> start, std::vector<double> end);
std::vector<double> line_guidance(std::vector<double> start, std::vector<double> end, std::vector<double> local, double step);
std::vector<double> circle_guidance(std::vector<double> center, double radius, double direc, std::vector<double> local, double step);
std::vector<double> Pturn_guidance(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, double radius, std::vector<double> local, double step);
// std::vector<double> obstacle_avoidance(std::vector<double> start, std::vector<double> end, std::vector<double> local, double step, std::vector<double> obstacle, double radius=-1);
std::vector<double> corridor_alt(std::vector<double> start, std::vector<double> end, std::vector<double> local3);
std::vector<double> velocity_guidance(std::vector<double> local, std::vector<double> setpoint);
std::vector<double> vel_saturation(const geometry_msgs::msg::Point32 &in, double sat);
std::vector<double> precise_landing_guidance(double_t* param, double heading, geometry_msgs::msg::Point32 object_pos, std::vector<double> local_pos);
#endif
