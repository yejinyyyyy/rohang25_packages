#include <uav_status.h>
#include <stdexcept>

/// uav_status uav(nh, "", 1);///////// ws check
uav_status::uav_status(ros::NodeHandle& nh, std::string name, uint8_t own)
{
    
    this->own = own;

    std::vector<std::string> subscriber_list_temp = SUBSCRIBER_LIST;   /// Defined it is used SUBSCRIBER_LIST
    for(int i = 0; i < subscriber_list_temp.size(); i++)
    { //name= '' // ws check
        subscriber_list_temp.at(i) = name + subscriber_list_temp.at(i);
    }

    this->name = name;
    this->subscriber_list = subscriber_list_temp;

/*    for(int i = 0; i < this->subscriber_list.size(); i++)
    {
        if(!(this->topic_check(this->subscriber_list.at(i))))
        {
            throw std::domain_error(this->subscriber_list.at(i) + " Topic not found");
        }
    }
*/
//    this->sub_state = nh.subscribe<mavros_msgs::State>
//    (this->subscriber_list.at(0).c_str() , 1, &uav_status::callback_state, this);
    
    this->sub_local_position = nh.subscribe<geometry_msgs::PoseStamped>
    (this->subscriber_list.at(1).c_str() , 1, &uav_status::callback_local_position, this);

    this->sub_local_velocity = nh.subscribe<geometry_msgs::TwistStamped>
    (this->subscriber_list.at(2).c_str() , 1, &uav_status::callback_local_velocity, this);
// void uav_status::callback_local_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
//::ConstPtr&는 const pointer 로서 msg의 주소값을 저장하는 역할을 해준다

    this->sub_body_velocity = nh.subscribe<geometry_msgs::TwistStamped>
    (this->subscriber_list.at(3).c_str() , 1, &uav_status::callback_body_velocity, this);

    this->sub_global_position = nh.subscribe<sensor_msgs::NavSatFix> // sub_global_position is Subscriber
    (this->subscriber_list.at(4).c_str() , 1, &uav_status::callback_global_position, this);

    this->sub_global_rel_alt = nh.subscribe<std_msgs::Float64>
    (this->subscriber_list.at(5).c_str() , 1, &uav_status::callback_global_rel_alt, this);

//    this->sub_local_position_target = nh.subscribe<mavros_msgs::PositionTarget>
//    (this->subscriber_list.at(6).c_str() , 1, &uav_status::callback_local_position_target, this);

//ws soojung 0914
    this->sub_home_position = nh.subscribe<mavros_msgs::HomePosition>
    (this->subscriber_list.at(7).c_str() , 1, &uav_status::callback_home_position, this);

//    this->sub_waypoints = nh.subscribe<mavros_msgs::WaypointList>
//    (this->subscriber_list.at(8).c_str() , 1, &uav_status::callback_waypoints, this);

    this->print_subscriber_list();

    if(own == 1)
    {
	std::cout<<name<<std::endl;
        this->pub_set_velocity = nh.advertise<geometry_msgs::TwistStamped>
        (this->name + "/mavros/setpoint_velocity/cmd_vel", 30);  // publish -what mean 30?
    }
    
}

uav_status::~uav_status() {}

void uav_status::set_velocity_ned(float Vn, float Ve, float Vd, float yaw_rate)
{
    geometry_msgs::TwistStamped set_velocity;

    set_velocity.twist.linear.x = Ve;
    set_velocity.twist.linear.y = Vn;
    set_velocity.twist.linear.z = -Vd;
    set_velocity.twist.angular.z = -yaw_rate;
    this->pub_set_velocity.publish(set_velocity);
}

void uav_status::print_subscriber_list()
{

    notice_bold("-----------SBUSCRIBER LISTS-----------");
    for(int i = 0; i < this->subscriber_list.size(); i++)
    { notice(subscriber_list.at(i)); } std::cout<<std::endl;

}


uint8_t uav_status::topic_check(std::string name)
{

	ros::master::V_TopicInfo master_topics;
	ros::master::getTopics(master_topics);

    for (ros::master::V_TopicInfo::iterator it = master_topics.begin() ; it != master_topics.end(); it++)
    {
        const ros::master::TopicInfo& info = *it;
        int idx = info.name.find(name);
        if((idx == 0)||(idx == 1))
        {
            return 1;
        }                   
    }

    return 0;

}

	//////////////////////////////////////////////////////////////
	//                  Callback Function Lists                 //
	//////////////////////////////////////////////////////////////

void uav_status::callback_state(const mavros_msgs::State::ConstPtr& msg)
{
    for(int i = 0; i < 3; i++) { this->state[i+1] = this->state[i];}
    this->state[0] = *msg;
}

void uav_status::callback_local_position(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    //geometry_msgs::PoseStamped local_position_enu = *msg;
    //this->local_position = local_position_enu;
    this->local_position = *msg;
    this->position_ned[0] = this->local_position.pose.position.y;
    this->position_ned[1] = this->local_position.pose.position.x;
    this->position_ned[2] = -this->global_rel_alt.data;;

    Eigen::Quaterniond q(this->local_position.pose.orientation.w,
    this->local_position.pose.orientation.x,
    this->local_position.pose.orientation.y,
    this->local_position.pose.orientation.z);

    //this->attitude_ned[0] = atan2(2.0*(q.y()*q.z() + q.w()*q.x()), q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z());
    Eigen::Vector3d q_enu = q.toRotationMatrix().eulerAngles(0, 1, 2);

    if(abs(q_enu[0]) > 3.141592/2)
    {
        this->attitude_ned[0] = q_enu[1] - 3.141592;
        this->attitude_ned[1] = q_enu[0] - 3.141592;
        this->attitude_ned[2] = -q_enu[2] + 3.141592*3/2;
    }

    else
    {        
        this->attitude_ned[0] = q_enu[1];
        this->attitude_ned[1] = q_enu[0];
        this->attitude_ned[2] = -q_enu[2] + 3.141592/2;
    }


    if(this->attitude_ned[0]>3.141592)
        this->attitude_ned[0] = this->attitude_ned[0] - 2*3.141592;
    else if (this->attitude_ned[0]<-3.141592)
        this->attitude_ned[0] = -(this->attitude_ned[0] + 2*3.141592);

    if(this->attitude_ned[1]>3.141592)
        this->attitude_ned[1] = this->attitude_ned[1] - 2*3.141592;
    else if (this->attitude_ned[1]<-3.141592)
        this->attitude_ned[1] = this->attitude_ned[1] + 2*3.141592;

    if(this->attitude_ned[2]>3.141592)
        this->attitude_ned[2] = this->attitude_ned[2] - 2*3.141592;
    else if (this->attitude_ned[2]<-3.141592)
        this->attitude_ned[2] = this->attitude_ned[2] + 2*3.141592;
  

}

void uav_status::callback_local_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    /*this->local_velocity.twist.linear.x = local_velocity_enu.twist.linear.y;
    this->local_velocity.twist.linear.y = local_velocity_enu.twist.linear.x;
    this->local_velocity.twist.linear.z = -local_velocity_enu.twist.linear.z;*/
    this->local_velocity = *msg;
    this->velocity_ned[0] = this->local_velocity.twist.linear.y;
    this->velocity_ned[1] = this->local_velocity.twist.linear.x;
    this->velocity_ned[2] = -(this->local_velocity.twist.linear.z);
}

void uav_status::callback_body_velocity(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    this->body_velocity = *msg;
    this->rate[0] = this->body_velocity.twist.angular.x;
    this->rate[1] = -(this->body_velocity.twist.angular.y);
    this->rate[2] = -(this->body_velocity.twist.angular.z);
}

void uav_status::callback_global_position(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
    this->global_position = *msg; 
    this->global[0] = this->global_position.latitude;
    this->global[1] = this->global_position.longitude;
    this->global[2] = -this->global_rel_alt.data;
}

void uav_status::callback_global_rel_alt(const std_msgs::Float64::ConstPtr& msg)
{this->global_rel_alt = *msg;}
void uav_status::callback_local_position_target(const mavros_msgs::PositionTarget::ConstPtr& msg)
{this->local_position_target = *msg;}
void uav_status::callback_home_position(const mavros_msgs::HomePosition::ConstPtr& msg)
{   
    this->home_position = *msg; 
    this->homepose[0] = this->home_position.geo.latitude; //ws add
    this->homepose[1] = this->home_position.geo.longitude; // ws add
    this->homepose[2] = this->home_position.geo.altitude; // ws add
   
}

void uav_status::callback_waypoints(const mavros_msgs::WaypointList::ConstPtr& msg)
{this->waypoints = *msg; }
