#ifndef __LOG_H__
#define __LOG_H__

#include <mavros_msgs/srv/file_open.hpp>
#include <mavros_msgs/srv/file_close.hpp>
#include <mavros_msgs/srv/file_write.hpp>

#include <stdio.h>
// #include <tf/tf.h>

#define MODE_READ 0
#define MODE_WRITE 1
#define MODE_CREATE 2

int create_file(const char *path);
//void save_timestamp(const char *path, int wpt, unsigned long long time);
//void save_output(const char *path, int wpt, double data, double altitude, unsigned long long time);
void save_setpoint_local(const char *path, unsigned long long time, int wpt, std::vector<double> setpoint, std::vector<double> local);
// void save_log(const char *path, int autopilot, double *llh, int *datetime, int mission_index);

#endif
