#ifndef __FRAME_H__
#define __FRAME_H__

#include "global_def.h"
#include "config.h"

std::vector<double> llh2ecef(std::vector<double> llh);
std::vector<double> ecef2llh(std::vector<double> ecef);
std::vector<double> ecef2enu(std::vector<double> ecef,
                            std::vector<double> ori_ecef);
std::vector<double> ecef2ned(std::vector<double> ecef,
                            std::vector<double> ori_ecef);
std::vector<double> ecef2en(std::vector<double> ecef,
                            std::vector<double> ori_ecef);


#endif
