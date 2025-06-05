#ifndef __MISSION_H__
#define __MISSION_H__

#include "global_def.h"
#include "config.h"
#include "frame.h"


#if MISSION_NUMBER == KONKUK_BIG_QUAD // 대운동장
    #define MISSION_ALT1 15
    #define MISSION_ALT2 20
    #define MISSION_RADIUS 15
    #define CIRCLE_DIRECTION CCW
#elif MISSION_NUMBER == KONKUK_SMALL_QUAD // 공터
    #define MISSION_ALT1 50
    #define MISSION_ALT2 10
    #define MISSION_ALT3 30
    #define MISSION_RADIUS 30
    #define PTURN_RADIUS 15
    #define PTURN_DIRACTION CW
    #define CIRCLE_DIRECTION CCW
#elif MISSION_NUMBER == TAEAN
    #define MISSION_ALT1 50
    #define MISSION_ALT2 10
    #define MISSION_ALT3 30
    #define MISSION_RADIUS 50
    #define PTURN_RADIUS 30
    #define PTURN_DIRACTION CW
    #define CIRCLE_DIRECTION CCW
#elif MISSION_NUMBER == HANAM // 하남
    #define MISSION_ALT1 20
    #define MISSION_ALT2 40
    #define MISSION_RADIUS 75
    #define CIRCLE_DIRECTION CW
#elif MISSION_NUMBER == YANGPYEONG // 양평
    #define MISSION_ALT1 50
    #define MISSION_ALT2 70
    #define MISSION_RADIUS 75
    #define CIRCLE_DIRECTION CCW
#elif MISSION_NUMBER == GONGSA // 공군사관학교
    #define MISSION_ALT1 40
    #define MISSION_ALT2 90
    #define MISSION_RADIUS 72
    #define CIRCLE_DIRECTION 
#elif MISSION_NUMBER == INCHEON // 인천
    #define MISSION_ALT1 50
    #define MISSION_ALT2 10
    #define MISSION_ALT3 30
    #define MISSION_RADIUS 50
    #define PTURN_RADIUS 20
    #define PTURN_DIRACTION CW
    #define CIRCLE_DIRECTION CCW
#elif MISSION_NUMBER == TOECHON // 퇴촌
    #define MISSION_ALT1 50
    #define MISSION_ALT2 90
    #define MISSION_RADIUS 72
    #define CIRCLE_DIRECTION CW

#elif MISSION_NUMBER == GWANGNARU // 
    #define MISSION_ALT1 15
    #define MISSION_ALT2 20
    #define MISSION_RADIUS 15
    #define CIRCLE_DIRECTION CCW

#elif MISSION_NUMBER == LIDAR_TEST // lidar test in konkuk
    #define MISSION_ALT1 4.5
    #define MISSION_ALT2 2.5
    #define MISSION_RADIUS 0 // not use
    #define CIRCLE_DIRECTION 0 // not use

#elif MISSION_NUMBER == SACHEON // 
    #define MISSION_ALT1 50
    #define MISSION_ALT2 50
    #define MISSION_ALT3 50
    #define MISSION_RADIUS 50
    #define PTURN_RADIUS 30
    #define PTURN_DIRACTION CW
    #define CIRCLE_DIRECTION CCW

#else // 임시
    #define MISSION_ALT1 40
    #define MISSION_ALT2 90
    #define MISSION_RADIUS 75
    #define CIRCLE_DIRECTION CCW
#endif

#define HOLD_TIME 3


std::vector<std::vector<double>> WPT = {
#if FRAME_MODE == LLH
    // {lat, lon, alt}, -> {x, y, z} 로 변환됨
#if MISSION_NUMBER == KONKUK_BIG_QUAD
    // 001_대운동장_QUAD
    // ALT1=10, ALT2=15, RADIUS=15, CCW
    {37.5441845, 127.0778389, MISSION_ALT1}, // base
    {37.5442934, 127.0779950, MISSION_ALT1}, // WPT#1
    {37.5444859, 127.0778722, MISSION_ALT1}, // WPT#2
    {37.5445160, 127.0776061, MISSION_ALT1}, // WPT#3
    {37.5442460, 127.0775728, MISSION_ALT2}, // WPT#4
    {37.5441908, 127.0776869, MISSION_ALT2}, // WPT#5
    {37.5441845, 127.0778389, MISSION_ALT2}  // base
#elif MISSION_NUMBER == KONKUK_SMALL_QUAD
    // 002_공터_QUAD
    // ALT1=10, ALT2=15, RADIUS=10, CCW
    {37.5436891, 127.0783433, MISSION_ALT1}, // base
    {37.5441264, 127.0784313, MISSION_ALT1}, // WPT#1
    {37.5445300, 127.0784937, MISSION_ALT1}, // WPT#2
    {37.5443536, 127.0775839, MISSION_ALT1}, // WPT#3
    {37.5443075, 127.0780804, MISSION_ALT2}, // WPT#4
    {37.5442580, 127.0786142, MISSION_ALT2}, // WPT#5
    {37.5442580, 127.0786142, MISSION_ALT3}, // WPT#6
    {37.5440129, 127.0788729, MISSION_ALT3}, // WPT#7
    {37.5436891, 127.0783433, 5}  // base
#elif MISSION_NUMBER == HANAM
    // 003_하남
    // ALT1=40, ALT2=90, RADIUS=75, CW
    {37.5582762, 127.2160029, MISSION_ALT1}, // base
    {37.5582492, 127.2163312, MISSION_ALT1}, // WPT#1
    {37.5591730, 127.2159570, MISSION_ALT1}, // WPT#2
    {37.5595800, 127.2172369, MISSION_ALT1}, // WPT#3
    {37.5582495, 127.2174854, MISSION_ALT2}, // WPT#4
    {37.5580313, 127.2163267, MISSION_ALT2}, // WPT#5
    {37.5582762, 127.2160029, MISSION_ALT2}  // base
#elif MISSION_NUMBER == YANGPYEONG
    // 004_양평
    // ALT1=40, ALT2=90, RADIUS=75, CCW
    {37.4832987, 127.4873662, MISSION_ALT1}, // base
    {37.4835550, 127.4878116, MISSION_ALT1}, // WPT#1
    {37.4827489, 127.4881599, MISSION_ALT1}, // WPT#2
    {37.4823894, 127.4888480, MISSION_ALT1}, // WPT#3
    {37.4837077, 127.4892529, MISSION_ALT2}, // WPT#4
    {37.4838282, 127.4880289, MISSION_ALT2}, // WPT#5
    {37.4832987, 127.4873662, MISSION_ALT2}  // base
#elif MISSION_NUMBER == GONGSA
    // 005_공군사관학교
    // ALT1=40, ALT2=90, RADIUS=75, CCW
    {36.5805664, 127.5277605, MISSION_ALT1}, // base
    {36.5809714, 127.5279660, MISSION_ALT1}, // WPT#1
    {36.5815662, 127.5273548, MISSION_ALT1}, // WPT#2
    {36.5817026, 127.5261213, MISSION_ALT1}, // WPT#3
    {36.5803526, 127.5261307, MISSION_ALT2}, // WPT#4
    {36.5804089, 127.5270927, MISSION_ALT2}, // WPT#5
    {36.5805664, 127.5277605, MISSION_ALT2}  // base
#elif MISSION_NUMBER == INCHEON
    // 006_인천
    {37.5209312, 126.6088170, MISSION_ALT1}, // base
    {37.5213276, 126.6080395, MISSION_ALT1}, // WPT#1
    {37.5229366, 126.6081187, MISSION_ALT1}, // WPT#2
    {37.5218822, 126.6058976, MISSION_ALT1}, // WPT#3
    {37.5218320, 126.6069376, MISSION_ALT2}, // WPT#4
    {37.5217879, 126.6081406, MISSION_ALT2}, // WPT#5
    {37.5223781, 126.6092603, MISSION_ALT3}, // WPT#6
    {37.5216580, 126.6088892, MISSION_ALT3}, // WPT#7
    {37.5209312, 126.6088170, 5}  // base
#elif MISSION_NUMBER == TOECHON
    // 007_퇴촌
    {37.4688332, 127.3030513, MISSION_ALT1}, // base
    {37.4686110, 127.3029754, MISSION_ALT1}, // WPT#1
    {37.4679514, 127.3026637, MISSION_ALT1}, // WPT#2
    {37.4679514, 127.3011346, MISSION_ALT1}, // WPT#3
    {37.4693003, 127.3011822, MISSION_ALT2}, // WPT#4
    {37.4691060, 127.3024175, MISSION_ALT2}, // WPT#5
    {37.4688332, 127.3030513, MISSION_ALT2}  // base
#elif MISSION_NUMBER == GWANGNARU
    // 008 
    {37.5476438, 127.1197185, MISSION_ALT1}, // base
    {37.5476139, 127.1198793, MISSION_ALT1}, // WPT#1
    {37.5476910, 127.1200029, MISSION_ALT1}, // WPT#2
    {37.5479443, 127.1201196, MISSION_ALT1}, // WPT#3
    {37.5480263, 127.1197961, MISSION_ALT2}, // WPT#4
    {37.5477636, 127.1196963, MISSION_ALT2}, // WPT#5
    {37.5476438, 127.1197185, MISSION_ALT2}  // base

#elif MISSION_NUMBER == LIDAR_TEST // lidar test in konkuk
    {37.5440768, 127.0783976, MISSION_ALT1}, // start
    {37.5442659, 127.0784259, MISSION_ALT2} // end


#elif MISSION_NUMBER == TAEAN
    // 009
    {36.660854, 126.341925, MISSION_ALT1}, // base
    {36.661078, 126.342193, MISSION_ALT1}, // WPT#1
    {36.662081, 126.344977, MISSION_ALT1}, // WPT#2
    {36.663238, 126.341070, MISSION_ALT1}, // WPT#3
    {36.661950, 126.341837, MISSION_ALT2}, // WPT#4
    {36.660259, 126.342790, MISSION_ALT2}, // WPT#5
    {36.659412, 126.344157, MISSION_ALT3},  // WPT#6
    {36.660226, 126.342404, MISSION_ALT3},  // WPT#7
    {36.660854, 126.341925, 5} // base

#elif MISSION_NUMBER == SACHEON //사천
    // 012
    {35.0688773, 128.0864906, MISSION_ALT1}, // base
    {35.0687931, 128.0865277, MISSION_ALT1}, // WPT#1
    {35.0685205, 128.0869303, MISSION_ALT1}, // WPT#2
    {35.0675394, 128.0872670, MISSION_ALT1}, // WPT#3
    {35.0683686, 128.0881632, MISSION_ALT2}, // WPT#4
    {35.0685205, 128.0869303, MISSION_ALT2}, // WPT#5
    {35.0691603, 128.0869242, MISSION_ALT3},  // WPT#6
    {35.0684781, 128.0862252, MISSION_ALT3},  // WPT#7
    {35.0688773, 128.0864906, 5} // base

#else
    // 임시용
    {36.5805664, 127.5277605, MISSION_ALT1}, // base
    {36.5809714, 127.5279660, MISSION_ALT1}, // WPT#1
    {36.5815662, 127.5273548, MISSION_ALT1}, // WPT#2
    {36.5817026, 127.5261213, MISSION_ALT1}, // WPT#3
    {36.5803526, 127.5261307, MISSION_ALT2}, // WPT#4
    {36.5804089, 127.5270927, MISSION_ALT2}, // WPT#5
    {36.5805664, 127.5277605, MISSION_ALT2}  // base
#endif
#elif FRAME_MODE == ENU
    // {x, y, alt},
    {0,      0,     MISSION_ALT1}, // base
    {5,     0,     MISSION_ALT1}, // WPT#1
    {75,   0,     MISSION_ALT1}, // WPT#2
    {75,   80,   MISSION_ALT1}, // WPT#3
    {-75,   80,   MISSION_ALT2}, // WPT#4
    {-10,    0,     MISSION_ALT2}, // WPT#5
    {0,      0,     MISSION_ALT1}  // base
#endif
};

void mission_llh2enu(std::vector<std::vector<double>> &mission, std::vector<double> home_llh)
{
    std::vector<double> home_ecef = llh2ecef(home_llh);
    std::vector<double> point_ecef, point_enu;
    int i = 0;
    int size = mission.size();

    for(i=0; i<size; i++){
        point_ecef = llh2ecef(mission[i]);
        point_enu = ecef2enu(point_ecef, home_ecef);
        mission[i][0] = point_enu[0];
        mission[i][1] = point_enu[1];
        mission[i][2] = point_enu[2];
    }
}

void mission_llh2ned(std::vector<std::vector<double>> &mission, std::vector<double> home_llh)
{
    std::vector<double> home_ecef = llh2ecef(home_llh);
    std::vector<double> point_ecef, point_ned;
    int i = 0;
    int size = mission.size();

    for(i=0; i<size; i++){
        point_ecef = llh2ecef(mission[i]);
        point_ned = ecef2ned(point_ecef, home_ecef);
        mission[i][0] = point_ned[0];
        mission[i][1] = point_ned[1];
        mission[i][2] = point_ned[2];
    }
}

void mission_calib_local(std::vector<std::vector<double>> &mission, std::vector<double> home_local)
{
    int i = 0;
    int size = mission.size();

    for(i=0; i<size; i++){
        mission[i][0] += home_local[0];
        mission[i][1] += home_local[1];
        mission[i][2] += home_local[2];
    }
}


#endif
