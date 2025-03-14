#include "guidance.h"

double get_angle(std::vector<double> start, std::vector<double> end)
{
    // +y축으로부터 직선에 대한 CW방향의 radian 각도 return(0~2*pi)
    // Input : 
    // start, end : 2차원 좌표

    std::vector<double> vec = eminus(start, end);
    double angle = atan2(vec[0], vec[1]) + M_PI;
    if(angle > 2*M_PI){
        angle -= 2*M_PI;
    }
    return angle;

}

std::vector<double> line_guidance(std::vector<double> start, std::vector<double> end, std::vector<double> local, double step)
{
    // 직선 경로 유도
    // Input 
    // start: 경로 시작점 좌표, end: 경로 끝점 좌표, local: 현재 위치 좌표, step: 이번 주기 setpoint 찍을 거리
    // Output
    // 이번 주기에서 목표로 할 setpoint 2차원 좌표

    double psi_path = get_angle(start, end);
    double psi_vehicle = get_angle(local, end);
    double dist_err = norm(eminus(local, end))*sin(psi_path - psi_vehicle);

    // double psi_err = - KP*atan(dist_err)*(M_PI/180);
    double psi_err = - KP*dist_err*(M_PI/180);
    // while(psi_err < -M_PI/2)
    //     psi_err += M_PI;
    // while(psi_err > M_PI/2)
    //     psi_err -= M_PI;

    double psi_control = psi_path + psi_err;

    // 0~360 deg to -180~180 deg
    if(psi_control > 2*M_PI) psi_control = psi_control - 2*M_PI;
    else if (psi_control < 0) psi_control = psi_control + 2*M_PI;
    if(psi_control >= M_PI) psi_control = psi_control - 2*M_PI;

    return {step*sin(psi_control), step*cos(psi_control)};
}

std::vector<double> circle_guidance(std::vector<double> center, double radius, double direc, std::vector<double> local, double step)
{
    // 원형 경로 유도
    // Input 
    // center: 선회 원 중심, radius: 선회 원 반경, direc: 선회방향(CW, CCW), local: 현재 위치 좌표, step: 이번 주기 setpoint 찍을 거리
    // Output
    // 이번 주기에서 목표로 할 setpoint 2차원 좌표

    double psi_vehicle = get_angle(local, center);
    double dist_center = norm(eminus(local, center));
    double psi_s;
    if(dist_center > radius){
        psi_s = atan2(radius, sqrt(abs(pow(dist_center, 2)-pow(radius, 2))));
    } else {
        psi_s = atan2(radius, -sqrt(abs(pow(dist_center, 2)-pow(radius, 2))));
    }
    if(direc == CW){
        psi_s = - psi_s;
    }
    double psi_control = psi_vehicle + psi_s;
    // RCLCPP_INFO(this->get_logger(), "guide angle: %f %f %f", psi_control, psi_vehicle, psi_s);

    // 0~360 deg to -180~180 deg
    if(psi_control > 2*M_PI) psi_control = psi_control - 2*M_PI;
    else if (psi_control < 0) psi_control = psi_control + 2*M_PI;
    if(psi_control >= M_PI) psi_control = psi_control - 2*M_PI;

    return {step*sin(psi_control), step*cos(psi_control)};
}

std::vector<double> Pturn_guidance(std::vector<double> p1, std::vector<double> p2, std::vector<double> p3, double radius, std::vector<double> local, double step)
{
    // P턴 경로 유도
    // Input 
    // p1: 출발점, p2: 선회점, p3: 다음 Wp, radius: 선회 원 반경, direc: 선회방향(CW, CCW), local: 현재 위치 좌표, step: 이번 주기 setpoint 찍을 거리
    // Output
    // 이번 주기에서 목표로 할 setpoint 2차원 좌표, P턴 탈출 지점 2차원 좌표
    int direc;
    
    std::vector<double> u1 = mult_const(eminus(p2, p1), 1/norm(eminus(p2, p1))); // p1, p2 unit vector
    std::vector<double> u2 = mult_const(eminus(p2, p3), 1/norm(eminus(p2, p3))); // p3, p2 unit vector
    double theta_circle = acos(dot(u1, u2));
    std::vector<double> u_center = mult_const(eplus(u1, u2), 1/norm(eplus(u1, u2))); // unit vector to center
    double dist_center = radius/sin(theta_circle/2);  // x
    std::vector<double> v_center = mult_const(u_center, dist_center);

    std::vector<double> center = eplus(p2, v_center);  // 원 중심

    double circle_direc = u1[0]*u2[1] - u1[1]*u2[0];

    if (circle_direc < 0)
        direc = 0;
    else
        direc = 1;

    std::vector<double> setpoint = circle_guidance(center, radius, direc, local, step);  // 원형 가이던스로 setpoint 반환

    return setpoint; 
}


// std::vector<double> obstacle_avoidance(std::vector<double> start, std::vector<double> end, std::vector<double> local, double step, std::vector<double> obstacle, double radius)
// {
//     // 직선 경로 유도 상 장애물 원형 회피
//     // Input
//     // start: 경로 시작점 좌표, end: 경로 끝점 좌표, local: 현재 위치 좌표, step: 이번 주기 setpoint 찍을 거리
//     // obstacle: 이번 주기 최근접 장애물 탐지 좌표, radius: 회피 반경(default=-1: 장애물 없음으로 인식)
//     // Output
//     // 이번 주기에서 목표로 할 setpoint 2차원 좌표

//     static bool first_run = false;
//     static int direction;
//     double psi_path = get_angle(start, end);

//     // 장애물이 없으면 직선 경로 유도
//     if(isnan(obstacle[0]) || radius == -1){
//         first_run = false;

//         ROS_INFO("ObstacleAvoid - LineGuide");
//         return line_guidance(start, end, local, step);
//     }

//     double dir;
//     // 장애물이 있는 첫 상황인 경우
//     if(first_run == false){
//         first_run = true;

//         std::vector<double> target_dir = eminus(end, local);
//         std::vector<double> obstacle_dir = eminus(obstacle, local);
//         dir  = target_dir[0]*obstacle_dir[1] - target_dir[1]*obstacle_dir[0]; // 외적 계산임
//         if(dir > 0){ // 장애물이 경로상 우측에 있으면 반시계 방향으로 회피
//             direction = CCW;
//         }else{ // 장애물이 경로상 좌측에 있으면 시계 방향으로 회피
//             direction = CW;
//         }
//     }
//     // 장애물 좌표 중심으로 원형 회피
//     ROS_INFO("ObstacleAvoid - CircleGuide");
//     return circle_guidance(obstacle, radius, direction, local, step);  
    
// }

std::vector<double> corridor_alt(std::vector<double> start, std::vector<double> end, std::vector<double> local3)
{
    std::vector<double> WPT_dist = eminus(end, start);  // a
    std::vector<double> start2curr = eminus(local3, start); // b
    std::vector<double> current_line = mult_const(WPT_dist, 1/(norm(WPT_dist)*norm(WPT_dist))*dot(WPT_dist, start2curr));

    return current_line;
}

std::vector<double> velocity_guidance(std::vector<double> local, std::vector<double> setpoint)
{
    // Input: local(2D), setpoint(3D)
    // Output: current position - setpoint unit vector * cruise speed 2D -> pose

    std::vector<double> setpoint_2d = {setpoint[0], setpoint[1]}; 
    double velocity = 17;  // cruise speed (assumed)
    std::vector<double> v = eminus(setpoint_2d, local);
    std::vector<double> unit = mult_const(v, 1/norm(v));  // u
    std::vector<double> vel_vect = mult_const(unit, velocity);

    return vel_vect;
}
