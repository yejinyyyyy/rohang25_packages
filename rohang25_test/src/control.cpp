#include "control.h"

void set_position(px4_msgs::msg::TrajectorySetpoint &pose, std::vector<double> coor)
{
    // 기체 자세에서 로컬 위치 지정 (ENU -> NED)
    pose.position[0] = coor[1];
    pose.position[1] = coor[0];
    pose.position[2] = -coor[2];
}

void set_heading(px4_msgs::msg::TrajectorySetpoint &pose, double heading)
{
    // 기체 자세에서 헤딩 지정
    // Input : 
    // pose: 자세 저장할 변수, heading: 북쪽 기준으로 시계방향 각도 (radian)

    if(heading > M_PI){
        heading = heading - 2*M_PI;
    }

    pose.yaw = heading;
}

void set_altitude(px4_msgs::msg::TrajectorySetpoint &pose, double alt)
{
    // 기체 고도만 지정
    pose.position[2] = alt;
}

void set_velocity(px4_msgs::msg::TrajectorySetpoint &pose, std::vector<double> vel)
{
    pose.velocity[0] = vel[0];
    pose.velocity[1] = vel[1];
    pose.velocity[2] = vel[2];
}


bool hold(double seconds) 
{
    // 지정된 시간 재주는 타이머. 시간 지나고 호출되면 true 반환
    static bool flag = false;
    static rclcpp::Time start_time; 
    rclcpp::Clock clock(RCL_ROS_TIME);

    if(seconds == 0){
        return true;
    }

    if(flag == false){
        flag = true;
        start_time = clock.now();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "start holding.. : %f", seconds);
    }
    if(clock.now() - start_time > rclcpp::Duration::from_seconds(seconds)){
        flag = false;
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "finish holding! : %f", seconds);
        return true;
    }
    return false;
}

bool is_arrived_hori(px4_msgs::msg::VehicleLocalPosition local, std::vector<double> set, double hori_err)
{
    // 지정된 수평 오차 이내로 목표점에 도달했는지 확인

    double x_err = set[0] - local.y;  // E
    double y_err = set[1] - local.x;  // N
    double d_err = norm({x_err, y_err});         

    // double hori_err;
    // if(current_flight_mode == MC){
    //     hori_err = MC_HORIZONTAL_ERROR;
    // } else if(current_flight_mode == FW){
    //     hori_err = FW_HORIZONTAL_ERROR;
    // }

    if(d_err < hori_err){
        return true;
    } else {
        return false;
    }
    
}

bool is_arrived_verti(px4_msgs::msg::VehicleLocalPosition local, std::vector<double> set, double verti_err)
{
    // 지정된 수직 오차 이내로 목표점에 도달했는지 확인
    // double z_err = abs(set_pos.pose.position.z - my_pos.pose.position.z);
    double z_err = abs(set[2] + local.z); //* */

    // double verti_err;
    // if(current_flight_mode == MC){
    //     verti_err = MC_VERTICAL_ERROR;
    // } else if(current_flight_mode == FW){
    //     verti_err = FW_VERTICAL_ERROR;
    // }

    if(z_err < verti_err){
        return true;
    } else {
        return false;
    }
}

// std::vector<double> quat2RPY(double x, double y, double z, double w)
// {
//     tf2::Quaternion quat(x,y,z,w);
//     tf2::Matrix3x3 m(quat);
//     double roll, pitch, yaw;
//     m.getRPY(roll, pitch, yaw);

//     return {roll, pitch, yaw};
// }

// double get_current_heading(VehicleLocalPosition my_pos)
// {
//     // 현재 자세의 방위각 리턴(radian)
//     double my_yaw = quat2RPY(my_pos.pose.orientation.x,
//                              my_pos.pose.orientation.y,
//                              my_pos.pose.orientation.z,
//                              my_pos.pose.orientation.w)[2];
//     my_yaw = - my_yaw + (M_PI/2);
//     while(my_yaw < 0)
//         my_yaw += 2*M_PI;
//     while(my_yaw > 2*M_PI)
//         my_yaw -= 2*M_PI;

//     return my_yaw;
// } // ----------------------> VehicleLocalPosition.heading or listener_callback_attitude

bool is_arrived_direc(px4_msgs::msg::VehicleLocalPosition my_pos, double heading, double a_err)
{   
    if(heading > M_PI){
        heading = heading - 2*M_PI;
    }

    double err = abs(my_pos.heading - heading);
    if(err > M_PI){
        err = 2*M_PI - err;
    }
    if(err < a_err){
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

double remain_dist(px4_msgs::msg::VehicleLocalPosition my_pos, std::vector<double> wpt)
{
    // 현재 위치와 목표점 사이의 수평 거리
    std::vector<double> local = {my_pos.y, my_pos.x};
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

// void update_posestamp_header(TrajectorySetpoint &pose) //
// {
//     static unsigned long seq = 0;
//     ros::Time time = ros::Time::now();
//     pose.header.seq = seq;
//     pose.header.stamp = time;
//     seq++;

// }



