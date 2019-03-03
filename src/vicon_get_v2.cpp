// Gabe Petersen
// vicon_get_v2.cpp - 28 Feb 2019
// Purpose: Try to convert a simulated duckiecar's position in vicon to
// 			geometry_msgs/Twist to be displayed by turtlesim_node
// *** Based of of tf2 listener tutorial ***

#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <cmath>

int main(int argc, char** argv) {
  ros::init(argc, argv, "vicon_get_v2");

  ros::NodeHandle nh;

  ros::Publisher tv = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  ros::Rate rate(10.0);

  while(nh.ok()) {
    geometry_msgs::TransformStamped transformStamped;

    try {
    	transformStamped = tfBuffer.lookupTransform("/turtle1/cmd_vel", "/vicon/duckiecar_1/duckiecar_1", 
				ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.5).sleep();
      continue;
    }

    geometry_msgs::Twist newPlace;

    newPlace.angular.z = 4.0 * atan2(transformStamped.transform.translation.y,
                                    transformStamped.transform.translation.x);
    newPlace.linear.x = 0.5 * sqrt(pow(transformStamped.transform.translation.x, 2) +
                                  pow(transformStamped.transform.translation.y, 2));
    tv.publish(newPlace);

    rate.sleep();
  }
  return 0;
};

