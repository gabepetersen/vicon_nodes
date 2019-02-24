#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Spawn.h>
#include <cmath>

void tfs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);


class ViconTurtle {
	public:
		ViconTurtle() {
			tfs = nh.subscribe("/vicon/duckiecar_1/duckiecar_1", 100, &ViconTurtle::tfs_cb, this);
			tpsh = nh.advertise<geometry_msgs::Twist>("turtle1/cmd_vel", 100);
		}

		void tfs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);

	private:
		ros::NodeHandle nh;
		ros::Subscriber tfs;
		ros::Publisher tpsh;
};

int main(int argc, char** argv) {

  	ros::init(argc, argv, "vicon_get");

	// instantiate object - call def constructor
	ViconTurtle VTurtle;  	
	
  	ros::spin();
  	return 0;
};

void ViconTurtle::tfs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	// get the message
	geometry_msgs::TransformStamped tfs_data = *msg;
	// print out data for error checking
	ROS_INFO("Translation Values: <%f, %f, %f>", tfs_data.transform.translation.x, 
																tfs_data.transform.translation.y, 	
																tfs_data.transform.translation.z);
   ROS_INFO("Quaternion Values = <x,y,z,w> = <%f, %f, %f, %f>", tfs_data.transform.rotation.x,
																					 tfs_data.transform.rotation.x, 
																					 tfs_data.transform.rotation.x,
	  																				 tfs_data.transform.rotation.x );
	
	geometry_msgs::Twist newPlace;
	// Translation Vector - Varies Velocity and Placement in Space
	newPlace.linear = tfs_data.transform.translation;
	// Measure twist around only z-axis since car will operate on xy plane
	float qw = tfs_data.transform.rotation.w;
	float qz = tfs_data.transform.rotation.z;
	// Get angle (radians) from quarternion and print it for error checking (if NaN)
   float z_angle = 2 * acos(qw);
	ROS_INFO("Angle on z-axis is: %f", z_angle);
	
	// set z-angle and publish geometry_msg
	newPlace.angular.z = z_angle;
	tpsh.publish(newPlace);
}
