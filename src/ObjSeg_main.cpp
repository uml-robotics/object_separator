/*
 * File: ObjSeg_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: main funcion for class ObjSeg.  See ObjSeg.h for more details.
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
    Publisher* imagePub = objSeg.getImagePublisher();
    Publisher* cloudPub = objSeg.getCloudPublisher();
    Subscriber pointSub = nh.subscribe<const PointCloud<PointXYZRGB>::ConstPtr&>("/camera/depth_registered/points",
                                                                                  1,
                                                                                  &ObjSeg::cloudCallback,
                                                                                  &objSeg);
                                                        
    Subscriber imageSub = nh.subscribe<sensor_msgs::Image>("/camera/rgb/image_rect_color",
                                                             1,
                                                             &ObjSeg::imageCallback,
                                                             &objSeg);

    *imagePub = nh.advertise<sensor_msgs::Image>("/scooter/rgb/image_rect_color", 1);
    *cloudPub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB> /*sensor_msgs::PointCloud2*/>("/scooter/depth_registered/points", 1);

    spin();
    
    return EXIT_SUCCESS;
}
