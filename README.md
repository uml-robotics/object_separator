#object_separator

###**Index**

- Project Synopsis
- Dependencies
- Build & Run
- Bugs & TODO


###**Project Synopsis**

This project uses PCL's new LCCP segmentation to segment the objects in an environment.


###**Dependencies**

- PCL >= 1.8.0
- pcl_ros jade unstable branch
- VTK (Ubuntu 14.04 default version)

###**Build & Run**

**Install Dependencies:**
- Go to http://pointclouds.org/downloads/.
- Install the newest **unstable** version of PCL, which is currently version 1.8.0.
- Install the newest unstable version of pcl/ros stuff (there is a link to it on the PCL Download page).
- You should now be able to ```catkin_make``` this node.

**Run**
```
roslaunch openni2_launch openni2.launch
rosrun object_separator object_separator
```

**NOTE**: When the PCL Visualizer first starts, the screen will be filled with these green and red blobs of color.  However, once you move rotate the visualizer scene even a little, it will display the correct pointcloud.


###**Bugs & TODO**

- ROS publishing has not been implemented yet.

