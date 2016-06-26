/*
* Author: Olalelan Ogunmolu
* Boston, MA
* June 2016
* Licensed under the MIT License
*/

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class odom
{
private:
	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;

public:
	odom(){}
	~odom(){}

	void readPose(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::Pose pose);
	void readTwist(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::Twist twist);

	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		// ROS_INFO_STREAM("Odom message aarived: " << odom_msg);
		readPose(odom_msg, pose);
		readTwist(odom_msg, twist);
	}
	
};

void odom::readPose(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::Pose pose)
{
	//x, y, z, and quarternion
	// pose = odom_msg.pose;
	// ROS_INFO_STREAM("odom pose " << odom_msg->pose);
}

void odom::readTwist(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::Twist twist)
{
	// twist = odom_msg.twist;
	// ROS_INFO_STREAM("odom twist " << odom_msg->twist);

}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom_node");
	ros::NodeHandle n_odom;
	odom od;
	ros::Subscriber sub = n_odom.subscribe("/RosAria/pose", 1000, &odom::odomCallback, &od);

	ros::spin();
	return EXIT_SUCCESS; 
}