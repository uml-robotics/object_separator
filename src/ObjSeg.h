/*
 * File: ObjSeg.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class performs object segmentation.  It utilizes the most current unstable versions of PCL and PCL_ROS (1.8).  See the README file for more details.
 *
 * References: https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/openni_organized_multi_plane_segmentation.cpp
               https://github.com/DeepBlue14/raptor/blob/master/measurements/src/Raptor_Segmentation.cpp
               http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html
 *
 */

#ifndef OBJ_SEG_H
#define OBJ_SEG_H

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
#include <pcl/segmentation/supervoxel_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/segmentation/lccp_segmentation.h> //PCL version 1.8.0 or greater

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
#include <sstream>
#include <vector>
#include <string>
#include <assert.h>

using namespace ros;
using namespace pcl;
using namespace cv;
using namespace std;



class ObjSeg
{
    private:
        pcl::PointCloud<PointXYZRGBA>::ConstPtr pclCloud;
        pcl::PointCloud<pcl::Normal>::Ptr m_normalCloud;
        pcl::visualization::PCLVisualizer::Ptr viewer;
	    Publisher* pub;

    public:
	    ObjSeg();
	    pcl::visualization::PCLVisualizer::Ptr initViewer(pcl::PointCloud<PointXYZRGBA>::ConstPtr cloud);
	    void updateVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud);
	    void showVisualizer();
	    void callback(const PointCloud<PointXYZRGBA>::ConstPtr& input);
        void lccpSeg();
        void llcpViewSetup();
        //void displayPlanarRegions
        //void displayEuclideanClusters
        //void displayCurvature
        //void displayDistanceMap
        //void removePreviousDataFromScreen
        //...
        //...
	    Publisher* getPublisher();
	    ~ObjSeg();

};

#endif /* OBJ_SEG_H */
