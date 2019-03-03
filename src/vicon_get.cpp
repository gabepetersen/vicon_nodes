#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2/LinearMath/Quaternion.h>
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
		geometry_msgs::Twist old_msg;
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

	// Translation Vector - Varies Velocity and Placement in Space - get change
	float x = tfs_data.transform.translation.x;
	float y = tfs_data.transform.translation.y;

	float x_vel = ( 0.5 * std::sqrt(pow(x, 2) + pow(y, 2)) );
	
	if( (x_vel - old_msg.linear.x) > 2 ) {
		old_msg.linear.x = x_vel - old_msg.linear.x;
	} else {
		old_msg.linear.x = 0;
	}
	
	/// convert the quaternion message with tf2
	tf2::Quaternion myRotation;
	tf2::convert(tfs_data.transform.rotation, myRotation);
	myRotation.normalize();

	float z_angle = myRotation.getAngle();

	// print out stuff for error checking
	ROS_INFO("Here is the angle of z: %f", z_angle);
	ROS_INFO("Here is the linear x: %f", old_msg.linear.x);

	// set z-angle and publish geometry_msg
	if( (z_angle - old_msg.angular.z) > 2 ) {
		old_msg.angular.z = z_angle - old_msg.angular.z;
	} else {
		old_msg.angular.z = 0;
	}

	tpsh.publish(old_msg);
}
