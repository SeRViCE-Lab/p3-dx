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
#include <std_msgs/Float64.h>

class odom
{
private:
	geometry_msgs::PoseWithCovariance pose;
	geometry_msgs::TwistWithCovariance twist;
	std_msgs::Float64 poseCov;
	std_msgs::Float64 twistCov;

public:
	odom(){}
	~odom(){}

	void readPose(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::PoseWithCovariance pose);
	void readTwist(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::TwistWithCovariance twist);

	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		// ROS_INFO_STREAM("Odom message aarived: " << odom_msg);
		readPose(odom_msg, pose);
		readTwist(odom_msg, twist);
	}
	
};

void odom::readPose(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::PoseWithCovariance pose)
{
	//x, y, z, and quarternion
	pose = odom_msg->pose;
	geometry_msgs::Point pose_posit = pose.pose.position;
	geometry_msgs::Quaternion pose_orient = pose.pose.orientation;
	// poseCov = pose.covariance;
	// ROS_INFO("pose linear: (%f, %f, %f)", pose_posit.x, pose_posit.y, pose_posit.z);
	// ROS_INFO("pose orient: (%f, %f, %f)", pose_orient.x, pose_orient.y, pose_orient.z);
	// std::cout << "\n" << std::endl;
}

void odom::readTwist(const nav_msgs::Odometry::ConstPtr& odom_msg, geometry_msgs::TwistWithCovariance twist)
{
	twist = odom_msg->twist;
	geometry_msgs::Vector3 twist_linear = twist.twist.linear;
	geometry_msgs::Vector3 twist_orient = twist.twist.angular;
	// twistCov = twist.covariance;
	// ROS_INFO("twist linear: (%f, %f, %f)", twist_linear.x, twist_linear.y, twist_linear.z);
	// ROS_INFO("twist orient: (%f, %f, %f)", twist_orient.x, twist_orient.y, twist_orient.z);

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