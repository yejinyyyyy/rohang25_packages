#include <uav_manage.h>
#include <stdio.h>
#include <vector>

uav_manage::uav_manage()
{

}

uav_manage::~uav_manage()
{

}

uint8_t uav_manage::del(std::string del_uav_name)
{
    for(std::vector<uav_status*>::iterator it = uav_list.begin(); it < uav_list.end(); it++)
    {
        if((*it)->name.compare(del_uav_name))
        {
            std::cout<<"!!!"<<std::endl;
        }
    }
}

void uav_manage::communication_health_check()
{

    std::cout<<"++-------Communication Health Check---------++"<<std::endl;
    for(std::vector<uav_status*>::iterator it = uav_list.begin(); it < uav_list.end(); it++)
    {
        info(((*it)->name) + " Communication Health : ");

        double dur = ros::Time::now().toSec() - (*it)->state[0].header.stamp.toSec();

        if(dur < 1.5)         info(std::to_string(dur));
        else if(dur < 2.5)    warning(std::to_string(dur));            
        else 
        {            
            error(std::to_string(dur));
            //uav_list.erase(it);
        }
    }
    std::cout<<"++------------------------------------------++"<<std::endl;

}


void uav_manage::add(uav_status* puav_status)
{
    uav_list.push_back(puav_status);

}

void uav_manage::show_list()
{

}
