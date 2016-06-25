#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "jog_mover");
	ros::NodeHandle njog;

	ros::Publisher pub = njog.advertise<geometry_msgs::Pose2D>("/rosarnl_node/jog_position_simple/goal", 1000);
	geometry_msgs::Pose2D msg;

	uint count = 0;
	ros::Duration five_sec = ros::Duration(5);
	ros::Duration ten_sec = ros::Duration(10);
	ros::Duration twenty_sec = ros::Duration(20);

	//move the robot to the origin
	msg.x = 0;
	msg.y = 0;
	msg.theta = 0;
	pub.publish(msg);
	five_sec.sleep();

	for(size_t i = 0; i < 3; ++i)
	{
		msg.x = 0.5;  //move forward 1.5meters
		msg.y = 0;
		msg.theta = 0; //move 90 deg
		pub.publish(msg);
		ROS_INFO("Robot moved in style by (x: %fm, y: %fm, theta: %frads)", msg.x, msg.y, msg.theta);
		ten_sec.sleep();
	}

	for(size_t i = 0; i < 3; ++i)
	{
		msg.x = -0.5;  //move forward 1.5meters
		msg.y = 0;
		msg.theta = 1.57; //move 90 deg
		pub.publish(msg);		
		ROS_INFO("Robot moved in style by (x: %fm, y: %fm, theta: %frads)", msg.x, msg.y, msg.theta);
		ten_sec.sleep();
	}

	// Guard, make sure the robot stops.
	msg.x = 0;
	msg.y = 0;
	msg.theta = 0;
	pub.publish(msg);
	ros::spin();
}