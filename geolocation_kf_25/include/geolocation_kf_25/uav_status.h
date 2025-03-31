#ifndef UAV_STATUS
#define UAV_STATUS

#include <ros/ros.h>
#include <Eigen/Geometry>

#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/HomePosition.h>
#include <mavros_msgs/WaypointList.h>

#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Float64.h>
#include <geographic_msgs/GeoPoint.h> // ws add

#include <string>
#include <vector>
#include <stdint.h>
#include <cmath>

#include <console_print.h>


#define SUBSCRIBER_LIST {"/mavros/state", \
                        "/mavros/local_position/pose", \
                        "/mavros/local_position/velocity_local", \
                        "/mavros/local_position/velocity_body" , \
                        "/mavros/global_position/global", \
                        "/mavros/global_position/rel_alt", \
                        "/mavros/setpoint_raw/target_local", \
                        "/mavros/home_position/home", \
                        "/mavros/mission/waypoints"}

// Roll, Pitch Data Bug must be modified :(

class uav_status
{

private:
    void callback_state(const mavros_msgs::State::ConstPtr& msg); ////ws check
    void callback_local_position(const geometry_msgs::PoseStamped::ConstPtr& msg); ////ws check
    void callback_local_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg); ////ws check
    void callback_body_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg); ////ws check
    void callback_global_position(const sensor_msgs::NavSatFix::ConstPtr& msg); ////ws check
    void callback_global_rel_alt(const std_msgs::Float64::ConstPtr& msg);
    void callback_local_position_target(const mavros_msgs::PositionTarget::ConstPtr& msg); ////ws check
    void callback_home_position(const mavros_msgs::HomePosition::ConstPtr& msg);
    void callback_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg);

public:
    std::vector<std::string> subscriber_list;
    std::string name; 

public:
    uav_status(ros::NodeHandle& nh, std::string name, uint8_t own);
    ~uav_status();

    void set_velocity_ned(float Vn, float Ve, float Vd, float yaw_rate);////ws check
public:    
    ros::Subscriber sub_state;
    ros::Subscriber sub_local_position;
    ros::Subscriber sub_local_velocity;
    ros::Subscriber sub_body_velocity;
    ros::Subscriber sub_global_position;
    ros::Subscriber sub_global_rel_alt;
    ros::Subscriber sub_local_position_target;
    ros::Subscriber sub_home_position;
    ros::Subscriber sub_waypoints;

    ros::Publisher pub_set_velocity;


    mavros_msgs::State state[4];
	geometry_msgs::PoseStamped local_position;
    geometry_msgs::TwistStamped local_velocity;
    geometry_msgs::TwistStamped body_velocity;
    sensor_msgs::NavSatFix global_position;
    std_msgs::Float64 global_rel_alt;
    mavros_msgs::PositionTarget local_position_target;
    mavros_msgs::HomePosition home_position;
    mavros_msgs::WaypointList waypoints;

    Eigen::Vector3d position_ned; // [North East Down]
    Eigen::Vector3d velocity_ned; // [Vel_N Vel_E Vel_D]
    Eigen::Vector3d attitude_ned; // [roll pitch yaw]_ned
    Eigen::Vector3d global;
    Eigen::Vector3d rate;         // [p q r]_FRD
    Eigen::Vector3d rel_home;     // Initial difference of home between leader follower
    Eigen::Vector3d homepose;     // Ws add

    uint8_t own;
public:
    void print_subscriber_list(); ////ws check
    uint8_t topic_check(std::string name);     ////ws check
};





#endif
