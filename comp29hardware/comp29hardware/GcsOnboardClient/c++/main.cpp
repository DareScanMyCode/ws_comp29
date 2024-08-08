#include "include/UavOnboard.h"
#include <iostream>
int main()
{
    config_t cfg;
    UavOnboard uav(cfg);
    int a;
    std::cout << "Press any key to exit" << std::endl;
    std::cin >> a;
    uav.send_msg("123123");
    uav.send_msg("222222");
    std::cout << "Press any key to exit" << std::endl;
    std::cin >> a;
    return 0;
}
