/*
 * Copyright (c) 2014, RoboPeak
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
/*
 *  RoboPeak LIDAR System
 *  RPlidar ROS Node client test app
 *
 *  Copyright 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 * 
 */
#include <chrono>
#include <iostream>
#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <stdio.h>
#include <sys/types.h>
#include <dirent.h>

#include <typeinfo>
#define RAD2DEG(x) ((x)*180./M_PI)

class LidarProcessor
{
    int numFiles = -2; //avoids counting two hidden dir
    std::string filePath = "/home/xiaor/data/lidar/scan_data_";

    public:
        LidarProcessor();
        void updateFilePath();
        void scanCallback(const sensor_msgs::LaserScan::ConstPtr&);
};

LidarProcessor::LidarProcessor(){
//  count number of files in lidar directory
    DIR *dp;
    struct dirent *ep;
    dp = opendir("/home/xiaor/data/lidar/");

    if(dp != NULL)
    {
        while(ep = readdir(dp))
            numFiles++;
        (void)closedir(dp);
    }
    else
        perror("Couldn't open the directory");


    filePath += std::to_string(numFiles) + ".txt"; //initialize first file
}

void LidarProcessor::updateFilePath(){
    numFiles++;
    filePath = "/home/xiaor/data/lidar/scan_data_" + std::to_string(numFiles) + ".txt";
}

void LidarProcessor::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan){
    //auto lidarOutputFile = std::fstream("~/data/lidar/scan_data.txt",std::ios::out);
    std::ofstream lidarOutputFile;
    lidarOutputFile.open(filePath, std::ios::out | std::ios::app);
    updateFilePath();

    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    std::chrono::microseconds ms = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch());
    ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));
    lidarOutputFile << "Time: " <<  ms.count() << "\n";
    std::cout << "Time: " << ms.count() << std::endl; 

    for(int i = 0; i < count; i++) {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i);
        lidarOutputFile << "Th: " << degree << " D(m): " << scan->ranges[i] << "\n";
    }

    lidarOutputFile.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "rplidar_node_write_file");
    ros::NodeHandle n;
    LidarProcessor lidarProcessor;
    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, &LidarProcessor::scanCallback, &lidarProcessor);

    ros::spin();

    return 0;
}
