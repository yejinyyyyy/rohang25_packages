#ifndef __ALGEBRA_H__
#define __ALGEBRA_H__

#include <math.h>
#include <vector>

double norm(std::vector<double> x1);
double dot(std::vector<double> v0, std::vector<double> v1);
std::vector<double> emult(std::vector<double> v0, std::vector<double> v1);
std::vector<double> mult_const(std::vector<double> v0, double v1);
std::vector<double> eplus(std::vector<double> v0, std::vector<double> v1);
std::vector<double> eminus(std::vector<double> v0, std::vector<double> v1);
std::vector<std::vector<double>> Multiply(std::vector<std::vector<double>> aMat,
                                         std::vector<std::vector<double>> bMat);
// double point2line_dist(std::vector<double> P1, std::vector<double> P2, std::vector<double> N1);
// double point2point_dist(std::vector<double> P1, geometry_msgs::PoseStamped P2);

#endif