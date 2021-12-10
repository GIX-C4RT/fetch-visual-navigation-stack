# fetch-visual-navigation-stack
This package handles the realsense t265 based visual slam navigation stack

#Setup
build the package as a standard ROS package via 
```
catkin_make
```

Execute this command on host computer(fetch in this case) connected with t265 tracking camera
```
roslaunch realsense2_camera rs_t265.launch
```

Execute the following command on anyother computer connected to the same ros master
```
roslaunch fetch_visual_navigation fetch_visual_navigation.launch
```

The command above will launching all the required node that are being used by navigation stack. It also brings up a rviz window so it might took a couple seconds to fully load up and ready to go
