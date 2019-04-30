# vicon_nodes
Package for experimenting in vicon motion capture with turtlesim

### Note: Not a part of the duckietown project

------------------------------------------------------------------------
# Implementation 
------------------------------------------------------------------------

### Note: Early Prototype of Development --- Not Finalized

### Instructions on how to run:

1. Create a vicon object called "duckiecar_1" and make sure "vicon/duckiecar_1/duckiecar_1" is advertising on your system

1. ```catkin build```

2. ```roslaunch vicon_nodes vicon_turtle.launch```

3. Move your vicon object

------------------------------------------------------------------------
# Problems that need to be solved
------------------------------------------------------------------------
1. Pretty Buggy - Not that accurate with tracking movement of vicon object
2. Angle from Quaternion is the most inaccurate with the current system
3. ***If vicon arena is coordinated differently, alter DesPose values in the callback function: vs_cb at the bottom.***


------------------------------------------------------------------------
