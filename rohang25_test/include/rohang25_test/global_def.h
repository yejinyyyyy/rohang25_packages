#ifndef __GLOBAL_DEF_H__
#define __GLOBAL_DEF_H__

#include <iostream>
#include <math.h>
#include <vector>

#define DEF_D2R(x) ((x)*(M_PI/180))
#define DEF_R2D(x) ((x)*(180/M_PI))
#define D2R     3.14159265359 / 180.0
#define R2D     180/3.14159265359
#define SIGN(x) (((x) > 0) ? 1 : (((x) < 0) ? -1 : 0))
#define DEF_PI M_PI

#define KONKUK_BIG_QUAD     1
#define KONKUK_SMALL_QUAD   2
#define HANAM               3
#define YANGPYEONG          4
#define GONGSA              5
#define INCHEON             6
#define TOECHON             7
#define GWANGNARU           8
#define LIDAR_TEST          9
#define CORRIDOR_TEST       10
#define TAEAN               11
#define SACHEON             12
#define OTHER               -1

#define LLH 0
#define ECEF 1
#define ENU 2

#define MC 3
#define FW 4

#define CW 0
#define CCW 1



#endif