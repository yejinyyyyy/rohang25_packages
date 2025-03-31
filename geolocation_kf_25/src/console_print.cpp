#include <console_print.h>


void error(std::string msg)
{
    std::cout<<RED<<"[ERROR] "<<msg<<RESET<<std::endl;
}

void warning(std::string msg)
{
    std::cout<<YELLOW<<"[WANRN] "<<msg<<RESET<<std::endl;
}

void notice(std::string msg)
{
    std::cout<<CYAN<<"[NOTICE] "<<msg<<RESET<<std::endl;
}

void info(std::string msg)
{
    std::cout<<WHITE<<"[INFO] "<<msg<<RESET<<std::endl;
}

void error_bold(std::string msg)
{
    std::cout<<BOLDRED<<msg<<RESET<<std::endl;
}

void warning_bold(std::string msg)
{
    std::cout<<BOLDYELLOW<<msg<<RESET<<std::endl;
}

void notice_bold(std::string msg)
{
    std::cout<<BOLDCYAN<<msg<<RESET<<std::endl;
}
