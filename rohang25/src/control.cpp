#include "control.h"

void set_position(geometry_msgs::PoseStamped &pose, std::vector<double> coor)
{
    // 기체 자세에서 로컬 위치 지정
    pose.pose.position.x = coor[0];
    pose.pose.position.y = coor[1];
    pose.pose.position.z = coor[2];
}

void set_heading(geometry_msgs::PoseStamped &pose, double heading)
{
    // 기체 자세에서 헤딩 지정
    // Input : 
    // pose: 자세 저장할 변수, heading: 북쪽 기준으로 시계방향 각도 (radian)
    tf2::Quaternion quat;

    heading = - heading + (M_PI/2);
    quat.setRPY(0, 0, heading);

    pose.pose.orientation.x = quat.getX();
    pose.pose.orientation.y = quat.getY();
    pose.pose.orientation.z = quat.getZ();
    pose.pose.orientation.w = quat.getW();
}

void set_altitude(geometry_msgs::PoseStamped &pose, double alt)
{
    // 기체 고도만 지정
    pose.pose.position.z = alt;
}

bool hold(double seconds)
{
    // 지정된 시간 재주는 타이머. 시간 지나고 호출되면 true 반환
    static bool flag = false;
    static ros::Time start_time;

    if(seconds == 0){
        return true;
    }

    if(flag == false){
        flag = true;
        start_time = ros::Time::now();
        ROS_INFO("start holding.. : %f", seconds);
    }
    if(ros::Time::now() - start_time > ros::Duration(seconds)){
        flag = false;
        ROS_INFO("finish holding! : %f", seconds);
        return true;
    }
    return false;
}

bool is_arrived_hori(geometry_msgs::PoseStamped local, std::vector<double> set, double hori_err)
{
    // 지정된 수평 오차 이내로 목표점에 도달했는지 확인
    // double x_err = set_pos.pose.position.x - my_pos.pose.position.x;
    // double y_err = set_pos.pose.position.y - my_pos.pose.position.y;
    double x_err = set[0] - local.pose.position.x;
    double y_err = set[1] - local.pose.position.y;
    double d_err = norm({x_err, y_err});

    // double hori_err;
    // if(current_flight_mode == MC){
    //     hori_err = MC_HORIZONTAL_ERROR;
    // } else if(current_flight_mode == FW){
    //     hori_err = FW_HORIZONTAL_ERROR;
    // }

    if(d_err < hori_err){
        // ROS_INFO("arrived pos!");
        return true;
    } else {
        // ROS_INFO("moving..");
        return false;
    }
    
}

bool is_arrived_verti(geometry_msgs::PoseStamped local, std::vector<double> set, double verti_err)
{
    // 지정된 수직 오차 이내로 목표점에 도달했는지 확인
    // double z_err = abs(set_pos.pose.position.z - my_pos.pose.position.z);
    double z_err = abs(set[2] - local.pose.position.z);

    // double verti_err;
    // if(current_flight_mode == MC){
    //     verti_err = MC_VERTICAL_ERROR;
    // } else if(current_flight_mode == FW){
    //     verti_err = FW_VERTICAL_ERROR;
    // }

    if(z_err < verti_err){
        // ROS_INFO("arrived alt!");
        return true;
    } else {
        // ROS_INFO("moving..");
        return false;
    }
}

std::vector<double> quat2RPY(double x, double y, double z, double w)
{
    tf2::Quaternion quat(x,y,z,w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    return {roll, pitch, yaw};
}

double get_current_heading(geometry_msgs::PoseStamped my_pos)
{
    // 현재 자세의 방위각 리턴(radian)
    double my_yaw = quat2RPY(my_pos.pose.orientation.x,
                             my_pos.pose.orientation.y,
                             my_pos.pose.orientation.z,
                             my_pos.pose.orientation.w)[2];
    my_yaw = - my_yaw + (M_PI/2);
    while(my_yaw < 0)
        my_yaw += 2*M_PI;
    while(my_yaw > 2*M_PI)
        my_yaw -= 2*M_PI;

    return my_yaw;
}

bool is_arrived_direc(geometry_msgs::PoseStamped my_pos, double heading, double a_err)
{
    double my_yaw = get_current_heading(my_pos);
    double err = abs(my_yaw - heading);
    if(err > M_PI){
        err = 2*M_PI - err;
    }
    if(err < a_err){
        // ROS_INFO("arrived heading!");
        return true;
    } else {
        return false;
    }
}

double next_waypoint_heading(std::vector<double> start, std::vector<double> end)
{
    // 두 점을 지나는 직선이 y축과 이루는 각, CW 방향: +
    // Input : start, end : 시점과 종점
    // Output : -pi < angle < pi, 북쪽 기준, 시계방향
    double heading;
    std::vector<double> direction = eminus(end, start);
    heading = atan2(direction[0], direction[1]);
    return heading;
}

std::vector<double> next_waypoint_unitvector(std::vector<double> start, std::vector<double> end)
{
    // 두 점을 지나는 직선의 단위 방향 벡터
    std::vector<double> direction = eminus(end, start);
    return mult_const(direction, 1/norm(direction));
}

double remain_dist(geometry_msgs::PoseStamped my_pos, std::vector<double> wpt)
{
    // 현재 위치와 목표점 사이의 수평 거리
    std::vector<double> local = {my_pos.pose.position.x, my_pos.pose.position.y};
    wpt.pop_back();
    return norm(eminus(wpt, local));
}

bool is_increase_dist(double dist)
{
    // 거리가 지난 루프때와 비교해서 증가했으면 true 반환, 단 특정 거리 이내에서만 작동
    double range;
    if(current_flight_mode == FW){
        range = 40;
    } else{
        range = 20;
    }
    static double last_dist = range;
    static int count = 0;
    
    if(dist > range){
        last_dist = range;
        return false;
    }
    if(dist < last_dist){
        last_dist = dist;
        count = 0;
        return false;
    } else if(dist > last_dist){
        count++;
        last_dist = dist;
        if(count >= 3){
            count = 0;
            last_dist = range;
            return true;
        }
        return false;
    }
}

void update_posestamp_header(geometry_msgs::PoseStamped &pose)
{
    static unsigned long seq = 0;
    ros::Time time = ros::Time::now();
    pose.header.seq = seq;
    pose.header.stamp = time;
    seq++;

}
