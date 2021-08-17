#include <iostream>
#include "ros/ros.h"


int main(){
    ros::WallTime time = ros::WallTime::now();

    while(true){
        std::cout << time.toNSec() << std::endl;
        time = ros::WallTime::now();
    }

}
