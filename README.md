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
- pcl_ros Jade unstable branch

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

to test that it is working properly, create a simple publisher to "chatter" geometry_msgs::Point objects to tablet/geometry_msgs/point.  Here is a sample one:
```
git clone future_ros_ip_redirection_node_repo
rosrun ros_ip_redirection chatterbot
```


