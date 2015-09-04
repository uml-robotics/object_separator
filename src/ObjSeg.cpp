#include "ObjSeg.h"

ObjSeg::ObjSeg()
{
    imagePub = new Publisher();
    cloudPub = new Publisher();
    pcl::PointCloud<PointXYZRGB>::Ptr init_cloud_ptr(new pcl::PointCloud<PointXYZRGB>);
}


void ObjSeg::cloudCallback(const PointCloud<PointXYZRGB>::ConstPtr& input)
{
    PointCloud<PointXYZRGB>::ConstPtr cloudCopy(new PointCloud<PointXYZRGB>);
    PointCloud<PointXYZRGB>::Ptr result(new PointCloud<PointXYZRGB>);
    cloudCopy = input;
    copyPointCloud(*cloudCopy, *result);

    pclCloud = cloudCopy;

    distanceFilter(pclCloud);

    lccpSeg();
    
    cv_bridge::CvImage out_msg;
    out_msg.header = rosImage.header;
    out_msg.encoding = sensor_msgs::image_encodings::BGR8;
    out_msg.image = m_image;
    imagePub->publish(out_msg.toImageMsg() );
    cloudPub->publish(resultCloud);
}


void ObjSeg::imageCallback(const sensor_msgs::ImageConstPtr& rosImage)
{
    this->rosImage = *rosImage;
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat cvImage;
    
    try
    {
        cv_ptr = cv_bridge::toCvCopy(rosImage, sensor_msgs::image_encodings::BGR8);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what() );
    }

    cvImage = cv_ptr->image;

    m_image = cvImage;
}


void ObjSeg::distanceFilter(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr& cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr resCloud(new PointCloud<PointXYZRGB>);
    *resCloud = *cloud;

    for(size_t i = 0; i < cloud->points.size(); i++)
    {
        if(!(cloud->points[i].z < 2.0) && !(cloud->points[i].z > 0.1) )
        {
            resCloud->points[i].r = 0;
            resCloud->points[i].g = 0;
            resCloud->points[i].b = 0;
        }
    }

    PointCloud<PointXYZRGB>::ConstPtr tmp(resCloud);
    pclCloud = tmp;
}


void ObjSeg::lccpSeg()
{
    uint k_factor = 0; //if you want to use extended convexity, set to 1

    // Values of parameters before parsing
    // Supervoxel Stuff
    float voxel_resolution = 0.0075f;
    float seed_resolution = 0.03f;
    float color_importance = 0.0f;
    float spatial_importance = 1.0f;
    float normal_importance = 4.0f;
    bool use_single_cam_transform = false;
    bool use_supervoxel_refinement = false;

    // LCCPSegmentation Stuff
    float concavity_tolerance_threshold = 10;
    float smoothness_threshold = 0.1;
    uint32_t min_segment_size = 0;
    bool use_extended_convexity = false;
    bool use_sanity_criterion = false;

    pcl::SupervoxelClustering<PointXYZRGB> superVox(voxel_resolution, seed_resolution);
    superVox.setUseSingleCameraTransform(use_single_cam_transform);
    superVox.setInputCloud(pclCloud);

    superVox.setColorImportance(color_importance);
    superVox.setSpatialImportance(spatial_importance);
    superVox.setNormalImportance(normal_importance);
    std::map<uint32_t, pcl::Supervoxel<PointXYZRGB>::Ptr> supervoxel_clusters;

    superVox.extract(supervoxel_clusters);
    
    if(use_supervoxel_refinement)
    {
        //PCL_INFO("About to refine supervoxels\n");
        superVox.refineSupervoxels(2, supervoxel_clusters);
    }
    
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    superVox.getSupervoxelAdjacency(supervoxel_adjacency);
    
    // Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointXYZRGB>::makeSupervoxelNormalCloud (supervoxel_clusters);

    // The Main Step: Perform LCCPSegmentation
    pcl::LCCPSegmentation<PointXYZRGB> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSanityCheck(use_sanity_criterion);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.segment(supervoxel_clusters, supervoxel_adjacency);
    
    if(min_segment_size > 0)
    {
        lccp.mergeSmallSegments(min_segment_size);
    }
    
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = superVox.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);

    pcl::PointCloud<PointXYZRGB>::Ptr convCloud(new PointCloud<PointXYZRGB>);
    *convCloud = *pclCloud;
    convCloud->width = lccp_labeled_cloud->width;
    convCloud->height = lccp_labeled_cloud->height;

    //color objects
    for(size_t i = 0; i < lccp_labeled_cloud->size(); i++)
    {
        convCloud->at(i).x = lccp_labeled_cloud->at(i).x;
        convCloud->at(i).y = lccp_labeled_cloud->at(i).y;
        convCloud->at(i).z = lccp_labeled_cloud->at(i).z;
        if(lccp_labeled_cloud->at(i).label > 2)
        {
            convCloud->at(i).r = 100;
            convCloud->at(i).g = 200;
            convCloud->at(i).b = 0;
        }
    }
     
    //this will be passed by Android
    int androidX = 100, androidY = 100;
     
    uint32_t theChosenLabel;
    int pclCount = 0;
    for(size_t y = 0; y < m_image.rows; y++)
    {
        for(size_t x = 0; x < m_image.cols; x++)
        {
            if(y == androidY && x == androidX)
            {
                realWorldCoorPoint.x = lccp_labeled_cloud->points[pclCount].x;
                realWorldCoorPoint.y = lccp_labeled_cloud->points[pclCount].y;
                realWorldCoorPoint.z = lccp_labeled_cloud->points[pclCount].z;
                theChosenLabel = lccp_labeled_cloud->points[pclCount].label;
                break;
            }
            pclCount++;
        } // end of outer for loop
    }
    
    vector<int> chosenLabelIndices;
    for(size_t i = 0; i < lccp_labeled_cloud->size(); i++)
    {
        if(lccp_labeled_cloud->at(i).label == theChosenLabel)
        {
            convCloud->at(i).r = 0;
            convCloud->at(i).g = 0;
            convCloud->at(i).b = 200;
            chosenLabelIndices.push_back(i);
        }
    }

    //map cloud colors --> image
    Point minPoint(1000, 1000);
    Point maxPoint(0, 0);

    for(size_t i = 0; i < chosenLabelIndices.size(); i++)
    {
        int currIndex = chosenLabelIndices.at(i);
        for(size_t currRowNum = 0; currRowNum < 479; currRowNum++)
        {
            if(currIndex >= 0 && currIndex < 640)
            {
                if(currIndex < minPoint.x)
                {
                    minPoint.x = currIndex;
                }
                else if(currIndex > maxPoint.x)
                {
                    maxPoint.x = currIndex;
                }
                
                if(currRowNum < minPoint.y)
                {
                    minPoint.y = currRowNum;
                }
                else if(currRowNum > maxPoint.y)
                {
                    maxPoint.y = currRowNum;
                }
                break;
            }
            else
            {
                currIndex -= 640;
            }
            
        }
        

    }

    const Point CAMERA_OFFSET(15, 15);
    rectangle(m_image, minPoint, maxPoint+CAMERA_OFFSET, Scalar(0, 255, 0), 14, 8);
    
    
    //imshow("result", m_image);
    //waitKey(3);
    
    //Update result to publish
    resultCloud = convCloud;
}


void ObjSeg::llcpViewSetup()
{
    ;//TODO implement this!!!
}


uint32_t ObjSeg::computeMaxLabel(pcl::PointCloud<PointXYZL>::Ptr labeledCloud)
{
    uint32_t maxLabel = 0;
    
    for(size_t i = 0; i < labeledCloud->size(); i++)
    {
        if(labeledCloud->at(i).label > maxLabel)
        {
            maxLabel = labeledCloud->at(i).label;
        }
    }
    
    return maxLabel;
}


pcl::PointCloud<pcl::PointXYZRGB>::Ptr ObjSeg::mapCvMat2PclCloud(Mat cvImage)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud;

    int pclCount = 0;
    for(size_t y = 0; y < cvImage.rows; y++)
    {
        for(size_t x = 0; x < cvImage.cols; x++)
        {
            Vec3b color = cvImage.at<Vec3b>(Point(x,y));
            cloud->points[pclCount].r = color.val[2];
            cloud->points[pclCount].g = color.val[1];
            cloud->points[pclCount].b = color.val[0];
            
            //cout << "(post) pixel z: " << cloud->points[pclCount].z << endl;
            pclCount++;
        } // end of outer for loop
    }
}


cv::Mat ObjSeg::mapPclCloud2CvMat(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud)
{
    Mat cvImage = m_image;
    
    int pclCount = 0;
    for(size_t y = 0; y < cvImage.rows; y++)
    {
        for(size_t x = 0; x < cvImage.cols; x++)
        {
            cvImage.at<Vec3b>(Point(x, y)).val[2] = cloud->points[pclCount].r;
            cvImage.at<Vec3b>(Point(x, y)).val[1] = cloud->points[pclCount].g;
            cvImage.at<Vec3b>(Point(x, y)).val[0] = cloud->points[pclCount].b;
            pclCount++;
        }
    }
    
    imshow("mapPclCloud2CvMat(...)", cvImage);
    waitKey(3);
    
    return cvImage;
}


Publisher* ObjSeg::getImagePublisher()
{
    return imagePub;
}


Publisher* ObjSeg::getCloudPublisher()
{
    return cloudPub;
}


ObjSeg::~ObjSeg()
{
    ;
}
