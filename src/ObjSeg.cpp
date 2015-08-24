#include "ObjSeg.h"

ObjSeg::ObjSeg()
{
    pub = new Publisher();
    pcl::PointCloud<PointXYZRGBA>::Ptr init_cloud_ptr (new pcl::PointCloud<PointXYZRGBA>);
    viewer = initViewer(init_cloud_ptr);
}


pcl::visualization::PCLVisualizer::Ptr ObjSeg::initViewer(pcl::PointCloud<PointXYZRGBA>::ConstPtr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("Viewer") );
    viewer->setBackgroundColor (0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<PointXYZRGBA> single_color(cloud, 0, 255, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_OPACITY, 0.15, "cloud");
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    
    return viewer;
}


void ObjSeg::callback(const PointCloud<PointXYZRGBA>::ConstPtr& input)
{
    PointCloud<PointXYZRGBA>::ConstPtr cloudCopy(new PointCloud<PointXYZRGBA>);
    PointCloud<PointXYZRGBA>::Ptr result(new PointCloud<PointXYZRGBA>);
    cloudCopy = input;
    copyPointCloud(*cloudCopy, *result);

    pclCloud = cloudCopy;

    lccpSeg();
    
    sensor_msgs::Image rosImage;
    pcl::toROSMsg(*result, rosImage);
    pub->publish(rosImage);
}


void ObjSeg::updateVisualizer(pcl::visualization::PCLVisualizer::Ptr viewer, pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr cloud)
{
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGBA> rgb(cloud);
    viewer->removePointCloud();
    viewer->addPointCloud(resultCloud);
    viewer->spinOnce(/*100*/);
}


void ObjSeg::showVisualizer()
{
    updateVisualizer(viewer, pclCloud);
}


void ObjSeg::lccpSeg()
{
    uint k_factor = 0; //if you want to use extended convexity, set to 1

    // Default values of parameters before parsing
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
    
    // Normals stuff (not implemented)
    /*pcl::PointCloud<pcl::Normal>::Ptr normals (new pcl::PointCLoud<pcl::Normal>);
    bool has_normals = false;
    if(pcl::getFieldIndex(pclCloud, "normal_x") >= 0)
    {
        
    }*/

    pcl::SupervoxelClustering<PointXYZRGBA> superVox(voxel_resolution, seed_resolution);
    superVox.setUseSingleCameraTransform(use_single_cam_transform); //!!!Not available in verion 1.7; requires >= 1.8.0!!!
    superVox.setInputCloud(pclCloud);
    
    //if(has_normals)
        //superVox.setNormalCloud(m_normalCloud);
    superVox.setColorImportance(color_importance);
    superVox.setSpatialImportance(spatial_importance);
    superVox.setNormalImportance(normal_importance);
    std::map<uint32_t, pcl::Supervoxel<PointXYZRGBA>::Ptr> supervoxel_clusters;
    
    //PCL_INFO("About to extract supervoxels\n");
    superVox.extract(supervoxel_clusters);
    
    if(use_supervoxel_refinement)
    {
        //PCL_INFO("About to refine supervoxels\n");
        superVox.refineSupervoxels(2, supervoxel_clusters);
    }
    
    std::stringstream temp;
    //temp << " Number of supervoxels: " << supervoxel_clusters.size() << "\n";
    //PCL_INFO(temp.str().c_str() );
    
    //PCL_INFO("About to process supervoxel adjacency\n");
    std::multimap<uint32_t, uint32_t> supervoxel_adjacency;
    superVox.getSupervoxelAdjacency(supervoxel_adjacency);
    
    // Get the cloud of supervoxel centroid with normals and the colored cloud with supervoxel coloring (this is used for visulization)
    pcl::PointCloud<pcl::PointNormal>::Ptr sv_centroid_normal_cloud = pcl::SupervoxelClustering<PointXYZRGBA>::makeSupervoxelNormalCloud (supervoxel_clusters);

    // The Main Step: Perform LCCPSegmentation
    //PCL_INFO("About to run LCCPSegmentation\n");
    pcl::LCCPSegmentation<PointXYZRGBA> lccp;
    lccp.setConcavityToleranceThreshold(concavity_tolerance_threshold);
    lccp.setSanityCheck(use_sanity_criterion);
    lccp.setSmoothnessCheck(true, voxel_resolution, seed_resolution, smoothness_threshold);
    lccp.setKFactor(k_factor);
    lccp.segment(supervoxel_clusters, supervoxel_adjacency);
    
    if(min_segment_size > 0)
    {
        //PCL_INFO("About to merge small segments\n");
        lccp.mergeSmallSegments(min_segment_size);
    }
    
    //PCL_INFO("Interpolation voxel cloud -> input cloud and relabeling\n");
    pcl::PointCloud<pcl::PointXYZL>::Ptr sv_labeled_cloud = superVox.getLabeledCloud();
    pcl::PointCloud<pcl::PointXYZL>::Ptr lccp_labeled_cloud = sv_labeled_cloud->makeShared();
    lccp.relabelCloud(*lccp_labeled_cloud);
    pcl::LCCPSegmentation<PointXYZRGBA>::SupervoxelAdjacencyList sv_adjacency_list;
    lccp.getSVAdjacencyList(sv_adjacency_list); // Needed for visualization
    
    //cout << "# of supervoxel clusters: " << supervoxel_clusters.size() << endl;
    
    //visualization stuff
    typedef LCCPSegmentation<PointXYZRGBA>::VertexIterator VertexIterator;
    typedef LCCPSegmentation<PointXYZRGBA>::AdjacencyIterator AdjacencyIterator;
    typedef LCCPSegmentation<PointXYZRGBA>::EdgeID EdgeID;
    
    std::set<EdgeID> edge_drawn;
    
    const unsigned char convex_color [3] = {255, 255, 255};
    const unsigned char concave_color [3] = {255, 0, 0};
    const unsigned char* color;
    
    //The vertices in the supervoxel adjacency list are the supervoxel centroids
    //This iterates through them, finding the edges
    std::pair<VertexIterator, VertexIterator> vertex_iterator_range;
    vertex_iterator_range = boost::vertices (sv_adjacency_list);
    
    
    // Create a cloud of the voxelcenters and map: VertexID in adjacency graph -> Point index in cloud
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New ();
    vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New ();
    vtkSmartPointer<vtkUnsignedCharArray> colors = vtkSmartPointer<vtkUnsignedCharArray>::New ();
    colors->SetNumberOfComponents (3);
    colors->SetName ("Colors");
    
    // Create a polydata to store everything in
    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New ();
    for (VertexIterator itr = vertex_iterator_range.first; itr != vertex_iterator_range.second; ++itr)
    {
      const uint32_t sv_label = sv_adjacency_list[*itr];
      std::pair<AdjacencyIterator, AdjacencyIterator> neighbors = boost::adjacent_vertices (*itr, sv_adjacency_list);

      for (AdjacencyIterator itr_neighbor = neighbors.first; itr_neighbor != neighbors.second; ++itr_neighbor)
      {
        EdgeID connecting_edge = boost::edge (*itr, *itr_neighbor, sv_adjacency_list).first; //Get the edge connecting these supervoxels
        if (sv_adjacency_list[connecting_edge].is_convex)
          color = convex_color;
        else
          color = concave_color;
        
        // two times since we add also two points per edge
        colors->InsertNextTupleValue (color);
        colors->InsertNextTupleValue (color);
        
        pcl::Supervoxel<PointXYZRGBA>::Ptr supervoxel = supervoxel_clusters.at (sv_label);
        pcl::PointXYZRGBA vert_curr = supervoxel->centroid_;
        
        
        const uint32_t sv_neighbor_label = sv_adjacency_list[*itr_neighbor];
        pcl::Supervoxel<PointXYZRGBA>::Ptr supervoxel_neigh = supervoxel_clusters.at (sv_neighbor_label);
        pcl::PointXYZRGBA vert_neigh = supervoxel_neigh->centroid_;
        
        points->InsertNextPoint (vert_curr.data);
        points->InsertNextPoint (vert_neigh.data);
          
        // Add the points to the dataset
        vtkSmartPointer<vtkPolyLine> polyLine = vtkSmartPointer<vtkPolyLine>::New ();
        polyLine->GetPointIds ()->SetNumberOfIds (2);
        polyLine->GetPointIds ()->SetId (0, points->GetNumberOfPoints ()-2);
        polyLine->GetPointIds ()->SetId (1, points->GetNumberOfPoints ()-1);
        cells->InsertNextCell (polyLine);
      }
    }
    
    polyData->SetPoints (points);
    // Add the lines to the dataset
    polyData->SetLines (cells);
    polyData->GetPointData ()->SetScalars (colors);

    //Update Visualizer
    resultCloud = lccp_labeled_cloud;
    showVisualizer();
}


void ObjSeg::llcpViewSetup()
{
    ;//TODO implement this!!!
}


Publisher* ObjSeg::getPublisher()
{
    return pub;
}


ObjSeg::~ObjSeg()
{
    ;
}
