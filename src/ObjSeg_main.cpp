/*
 * File: ObjSeg_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * 
 */
 
//ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

//STL
#include <iostream>
#include <cstdlib>
#include <string>

#include "ObjSeg.h"

using namespace ros;
using namespace std;

int main(int argc, char **argv)
{
    init(argc, argv, "ObjSeg");
    
    ROS_INFO("Starting node\n");

    ObjSeg objSeg;
    NodeHandle nh;
    Publisher* mainsPub = objSeg.getPublisher();
    Subscriber sub = nh.subscribe<const PointCloud<PointXYZRGBA>::ConstPtr&>("/camera/depth_registered/points",
                                                        1,
                                                        &ObjSeg::callback,
                                                        &objSeg);

    *mainsPub = nh.advertise<sensor_msgs::Image>("/scooter/rgb/image_rect_color", 1);

    spin();
    
    return EXIT_SUCCESS;
}
