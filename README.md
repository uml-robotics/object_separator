#object_separator

This is not not complete (or even fully functional yet), nor have I had time to clean up the files, split code into helper functions, etc, so...it's a bit of a mess.  However, it should be usefull.

###Build & Run

- Go to http://pointclouds.org/downloads/
- Install the newest **unstable** version of PCL, which is verion 1.8 (this is required for the stuff Andreas suggested).
- Install the newest version of pcl/ros stuff (I installed it to my /home directory, so that it wouldn't screw with my ros indigo version.
- You should now be able to ```catkin_make``` this node
- Now...finish implementing it, or use it for reference, etc.

References: https://github.com/PointCloudLibrary/pcl/blob/master/apps/src/openni_organized_multi_plane_segmentation.cpp
            https://github.com/DeepBlue14/raptor/blob/master/measurements/src/Raptor_Segmentation.cpp
            http://docs.pointclouds.org/trunk/classpcl_1_1_l_c_c_p_segmentation.html

**NOTE**: When the PCL Visualizer first starts, the screen will be filled with these green and red blobs of color.  However, once you move rotate the visualizer scene even a little, it will display the correct pointcloud.

