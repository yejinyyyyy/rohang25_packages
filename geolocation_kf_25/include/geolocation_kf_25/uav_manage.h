#ifndef UAV_MANAGE
#define UAV_MANAGE

#include <vector>
#include <string>

#include <uav_status.h>
#include <console_print.h>
#include <stdint.h>

#include <std_msgs/Header.h>
#include <geometry_msgs/PoseStamped.h>

class uav_manage
{
public:
    uav_manage();
    ~uav_manage();

    uint8_t del(std::string uav_name);
    void add(uav_status* puav_status);
    void show_list();
    void communication_health_check();

public:
    std::vector<uav_status*> uav_list;
};



#endif
