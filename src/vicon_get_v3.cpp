// Gabe Petersen
// vicon_get_v3.cpp - 2 Mar 2019
// Purpose: to get pose from vicon object and translate to pose of turtle in turtlesim

// *** Based off of code in:
// https://github.com/utari/UTARI_ROSTutorials/wiki/TurtlesimPositionController-Tutorial 

#include "ros/ros.h"
#include <math.h>
#include "geometry_msgs/Pose2D.h"									
#include "turtlesim/Pose.h"
#include "turtlesim/Spawn.h"										
#include "geometry_msgs/Twist.h"		
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"					

class ViconTurtle {
	public:
		// Default constructor to connect topics
		ViconTurtle() {
			// used to get the pose of the duckiebot to transfer to turtlesim
			vicon_stream = nh.subscribe("/vicon/duckiecar_1/duckiecar_1", 5, &ViconTurtle::vs_cb, this);
			// register sub to get current position/pose
			CurPose_sub = nh.subscribe("/turtle1/pose", 5, &ViconTurtle::cp_cb, this);
			// register pub to send twist velocity (cmd_vel)
			Twist_pub = nh.advertise<geometry_msgs::Twist>("/turtle1/cmd_vel", 100);
			STOP = true;
		}
		// Function declarations
		void vs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg);
		void cp_cb(const turtlesim::Pose::ConstPtr& msg);
		float difAngleVel();
		float difLinearVel();
		turtlesim::Pose getCurPose();
		geometry_msgs::Pose2D getDesPose();
		bool getStop();

		// publisher is public to be accessed easily out of object
		ros::Publisher Twist_pub;
		ros::NodeHandle nh;

	private:
		// ros variables
		ros::Subscriber vicon_stream;
		ros::Subscriber CurPose_sub;
		// controller that controls new movement
		bool STOP;					
		// helper variables to track movement	
		turtlesim::Pose CurPose;									
		geometry_msgs::Pose2D DesPose;	
};

int main(int argc, char **argv) {
	ros::init(argc, argv, "vicon_get_v3");

	// intantiate class
	ViconTurtle turtleCmd;

	// set loop rate to 10HZ
	ros::Rate loop_rate(10);	/// ---- changed from 10hz to 100hz
	// helper variables								
	float ErrorLin = 0;									
	float ErrorAng = 0;													
	geometry_msgs::Twist CmdVel;								
			
	while (ros::ok() && turtleCmd.nh.ok()) {

		ros::spinOnce();
		if (turtleCmd.getStop() == false) {
			// Get errors
			ErrorLin = turtleCmd.difLinearVel();				
			ErrorAng = turtleCmd.difAngleVel();			
			ROS_INFO("Position Difference: %f, Angle Difference: %f\n", ErrorLin, ErrorAng);

			// publish new velocity based on the given linear/angular velocities
			CmdVel.linear.x = 0.5 * ErrorLin;
			CmdVel.angular.z = 0.2 * ErrorAng;	
			turtleCmd.Twist_pub.publish(CmdVel);		
		
		} else {
			ROS_INFO("Waiting...\n");	
		}
		loop_rate.sleep();
	}
}

// call back function to update pose of vicon object
void ViconTurtle::vs_cb(const geometry_msgs::TransformStamped::ConstPtr& msg) {
	// tell main loop that there is new data
	STOP = false;
	// get message in TransformStamped
	geometry_msgs::TransformStamped tfs_data = *msg;

	/*******************************************************************************/
	/********* Bug with quaternion values not getting correct angle ****************/
	/*******************************************************************************/

	// get quaternion from TransformStamped and normalize
	tf2::Quaternion myRotation;
	tf2::convert(tfs_data.transform.rotation, myRotation);
	myRotation.normalize();
	// get z-angle and x and y positions of transform
	tf2::Matrix3x3 m(myRotation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	DesPose.theta = yaw;
	/*******************************************************************************/
	/********* Adjust additions for current vicon configuration ********************/
	/************* x and y are flipped for easier visualization ********************/
	/*******************************************************************************/
	DesPose.x = tfs_data.transform.translation.x + 7.5;
	DesPose.y = tfs_data.transform.translation.y + 5.5;
}
// call back function to update turtle sim pose
void ViconTurtle::cp_cb(const turtlesim::Pose::ConstPtr& msg)			
{
	CurPose.x = msg->x;
	CurPose.y = msg->y;
	CurPose.theta = msg->theta;								
	return;
}

// function to get angular error between facing direction of the turtle and direction to desired pose
float ViconTurtle::difAngleVel() {
	// create error vector
	float Ex = DesPose.x - CurPose.x;		
	float Ey = DesPose.y - CurPose.y;			
	
	// get desire angle
	float dest = atan2f(Ey, Ex); 		
	
	// get angle error
	float Et = dest - CurPose.theta;  // BUG with angle switching back and forth
	// float Et = DesPose.theta - CurPose.theta;
	
	return Et;
}

// function to get linear error from the turtles perspective. Error only along turtle X axis
float ViconTurtle::difLinearVel() {
	// create error vector
	float Ex = (DesPose.x - CurPose.x);	
	float Ey = (DesPose.y - CurPose.y);
	float Et = difAngleVel();							// get angle between vectors
	
	// project error onto turtle x axis
	// float Etx =  pow( pow(Ex,2.0) + pow(Ey,2.0), 0.5 )*cos(Et);
	float Etx = hypotf(Ex, Ey) * cos(Et); // improved c function
	
	return Etx;
}
turtlesim::Pose ViconTurtle::getCurPose() {
	return CurPose;
}
geometry_msgs::Pose2D ViconTurtle::getDesPose() {
	return DesPose;
}
bool ViconTurtle::getStop() {
	return STOP;
}
