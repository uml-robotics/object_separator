/*
 * File: ObjSeg_main.cpp
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: 
 *
 * 
 */
 
 
#include <ros/ros.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/io/io.h>
#include <pcl/common/time.h>
#include <pcl/common/angles.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/features/normal_3d.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/planar_region.h>
#include <pcl/segmentation/organized_multi_plane_segmentation.h>
#include <pcl/segmentation/organized_connected_component_segmentation.h>
#include <pcl/segmentation/euclidean_cluster_comparator.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/segmentation/edge_aware_plane_comparator.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>

#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


#include <math.h>
#include <stdlib.h>
#include <stdio.h>

#include <iostream>
#include <cstdlib>
#include <sstream>
#include <vector>
#include <string>
#include <assert.h>

#include "ObjSeg.h"

using namespace ros;
using namespace pcl;
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
