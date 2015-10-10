/*
 * File: ObjSeg.h
 * Author: James Kuczynski
 * Email: jkuczyns@cs.uml.edu
 * File Description: This class performs object segmentation using PCL's LCCPSegmentation API.  It utilizes the most current unstable versions of PCL and PCL_ROS (1.8).  See the README file for more details.
 *
 * References: https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/openni_organized_multi_plane_segmentation.cpp
 *             https://github.com/DeepBlue14/raptor/blob/master/measurements/src/Raptor_Segmentation.cpp
 *             http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html
 *
 * See also: http://pointclouds.org/documentation/tutorials/region_growing_rgb_segmentation.php#region-growing-rgb-segmentation
 */

#ifndef OBJ_SEG_H
#define OBJ_SEG_H

// ROS
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/Point.h>

// ROS <--> PCL conversions
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>


// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PCLImage.h>
#include <pcl/io/io.h>
#include <pcl/io/point_cloud_image_extractors.h>
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
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/surface/convex_hull.h>

#include <pcl/segmentation/lccp_segmentation.h> //PCL version 1.8.0 or greater

// VTK
#include <vtkRenderWindow.h>
#include <vtkImageReader2Factory.h>
#include <vtkImageReader2.h>
#include <vtkImageData.h>
#include <vtkImageFlip.h>
#include <vtkPolyLine.h>

// OpenCV
#include <opencv/cv.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

// STL
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
        bool gotNewPoint; //flag.  This is turned on whenever a new point is recieved, and shut off after the point has been processed.
        pcl::PointCloud<PointXYZRGB>::ConstPtr pclCloud;
        geometry_msgs::PointConstPtr pixelPoint;
        geometry_msgs::Point realWorldCoorPoint;
        sensor_msgs::Image rosImage;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr resultCloud;
        cv::Mat m_image;
	    Publisher* imagePub;
	    Publisher* cloudPub;

    public:
        /**
         * Constructor.
         */
	    ObjSeg();
	    

        void pointCallback(const geometry_msgs::PointConstPtr& pixelPoint);


	    /**
	     * This method is the ROS callback.  It converts the ROS pointcloud to a PCL pointcloud,
	     * calls segmentation and visualization methods. TODO Publish data.
	     *
	     * @param input the ROS pointcloud.
	     */
	    void cloudCallback(const PointCloud<PointXYZRGB>::ConstPtr& input);
	    
	    /**
	     *
	     */
	     void imageCallback(const sensor_msgs::ImageConstPtr& rosImage);
	    
	    /**
	     * This method blacks out features more then 2 meters(?) away.
	     */
	    void distanceFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud);
	    
	    /**
	     * This method performs a PCL LCCP segmentation.
	     */
        void lccpSeg();
        
        /**
         * TODO implement.
         */
        void llcpViewSetup();
        
        /**
         * 
         */
         uint32_t computeMaxLabel(pcl::PointCloud<PointXYZL>::Ptr labeledCloud);
        
        /**
         * This method copies the rgb values from an OpenCV matrix to a PCL pointcloud.
         *
         * @param cvImage
         * @return 
         */
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr mapCvMat2PclCloud(Mat cvImage);
        
        /**
         * Copies the rgb values from a PCL pointcloud to a OpenCV matrix.
         *
         * @param cloud
         * @return 
         */
        cv::Mat mapPclCloud2CvMat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
        
        /**
         * Accessor method for the ROS publisher.
         *
         * @return a pointer to the ROS publisher.
         */
	    Publisher* getImagePublisher();
	    
	    /**
	     *
	     *
	     */
	    Publisher* getCloudPublisher();
	    
	    /**
	     * Destructor.
	     */
	    ~ObjSeg();

};

#endif /* OBJ_SEG_H */
