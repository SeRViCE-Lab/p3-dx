#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>

class odom
{
	odom();
	~odom(){}
private:
	geometry_msgs::Pose pose;
	geometry_msgs::Twist twist;

public:
	void odomCallback(const nav_msgs::Odometry::ConstPtr& odom_msg)
	{
		ROS_INFO_STREAM("Odom message aarived: " << msg);
		readPose(msg, pose);
		readTwist(msg, twist);
	}
	
	void readPose(msg, Pose);
	{

	}

	void readTwist(msg, twist)
	{

	}
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, "odom node");
	ros::NodeHandle n_odom;
	odom od;
	ros::Subscriber sub = n_odom.sub("/RosAria/pose", 1000, odom::odomCallback, &od);
}