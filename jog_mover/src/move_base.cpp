#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "move_base");
	ros::NodeHandle nbase;

	ros::Publisher pub = nbase.advertise<geometry_msgs::PoseStamped>("/rosarnl_node/move_base_simple/goal", 1000);
	geometry_msgs::PoseStamped msg;

	int count = 0;

	ros::Duration five_sec = ros::Duration(5);
	ros::Duration twenty_sec = ros::Duration(20);

	msg.header.frame_id = "simple_pose";
	msg.header.stamp = ros::Time::now();
	msg.header.seq = ++count;

	//move the robot to the origin
	//linear
	msg.pose.position.x = 0;
	msg.pose.position.y = 0;
	msg.pose.position.z = 0;

	//quarternions
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = 0;
	msg.pose.orientation.w = 0;
	pub.publish(msg);
	five_sec.sleep();


	for(size_t i = 0; i < 2; ++i)
	{
		//linear
		msg.pose.position.x = 0.4;
		msg.pose.position.y =  0.4;
		msg.pose.position.z = 0.4;

		//quarternions
		msg.pose.orientation.x = 0.2;
		msg.pose.orientation.y = 0.2;
		msg.pose.orientation.z = 0.2;
		msg.pose.orientation.w = 0.3;

		pub.publish(msg);
		ROS_INFO_STREAM("Robot moved in style by: " << msg.pose.position << ", \n" << msg.pose.orientation);
		twenty_sec.sleep();
	}

	for(size_t i = 0; i < 2; ++i)
	{
		//linear
		msg.pose.position.x = 0.2;
		msg.pose.position.y = 0.2;
		msg.pose.position.z = 0.2;

		//quarternions
		msg.pose.orientation.x = 0.2;
		msg.pose.orientation.y = 0.2;
		msg.pose.orientation.z = 0.2;
		msg.pose.orientation.w = 0.3;

		pub.publish(msg);		
		ROS_INFO_STREAM("Robot moved in style by: " << msg.pose.position << ", \n" << msg.pose.orientation);
		twenty_sec.sleep();
	}

	// Guard, make sure the robot stops.
	//linear
	msg.pose.position.x = 0;
	msg.pose.position.y = 0;
	msg.pose.position.z = 0;

	//quarternions
	msg.pose.orientation.x = 0;
	msg.pose.orientation.y = 0;
	msg.pose.orientation.z = 0;
	msg.pose.orientation.w = 0;
	pub.publish(msg);
	five_sec.sleep();

	ros::spin();
	return 0;
}