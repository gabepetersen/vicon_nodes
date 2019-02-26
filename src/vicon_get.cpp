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
		geometry_msgs::TransformStamped old_msg;
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
	
	geometry_msgs::Twist newPlace;
	// Translation Vector - Varies Velocity and Placement in Space - get change
	
	/*
	if(tfs_data.transform.translation.x != old_msg.transform.translation.x) {
		newPlace.linear.x = tfs_data.transform.translation.x - old_msg.transform.translation.x;
	}
	if(tfs_data.transform.translation.y != old_msg.transform.translation.y) {
		newPlace.linear.y = tfs_data.transform.translation.y - old_msg.transform.translation.y;
	}
	*/	

	// Measure twist around only z-axis since car will operate on xy plane
	float qz;
	float qw = (tfs_data.transform.rotation.w - old_msg.transform.rotation.w);
	if(tfs_data.transform.rotation.z > 0) {
		qz = (tfs_data.transform.rotation.z - old_msg.transform.rotation.z);
	} else {
		qz = (old_msg.transform.rotation.z - tfs_data.transform.rotation.z);
	}

	// qauternion stuff 
	if(qz < 0.008 && qz > -0.008) {
		qw = 1.0;	
	}
	
	// Get angle (radians) from quarternion and print it for error checking (if NaN)
   float z_angle = 2 * acos(qw);
	
	// set z-angle and publish geometry_msg
	newPlace.angular.z = z_angle;
	newPlace.angular.y = 0;
	newPlace.angular.x = 0;

	// Redo old msg
	old_msg = tfs_data;
	tpsh.publish(newPlace);
}
