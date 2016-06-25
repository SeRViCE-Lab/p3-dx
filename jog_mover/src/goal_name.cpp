#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <vector>

int main(int argc, char** argv)
{
	ros::init(argc, argv, "goal_name");
	ros::NodeHandle ngoal;

	ros::Publisher pub = ngoal.advertise<std_msgs::String>("/rosarnl_node/goalname", 1000);
	std_msgs::String msg;

	std::vector<std::string> mappedLocs;
	mappedLocs.push_back("Lekan's Desk");
	mappedLocs.push_back("Andy's Desk");
	mappedLocs.push_back("Robot Arm1");
	mappedLocs.push_back("Teddy's Desk");
	mappedLocs.push_back("Robot Arm 1");
	mappedLocs.push_back("Machining Table 1");
	mappedLocs.push_back("Machining Table 2");
	mappedLocs.push_back("Lekan's Other Desk");
	mappedLocs.push_back("Teddy's Desk");
	mappedLocs.push_back("Carlos' Desk");
	mappedLocs.push_back("Negin's Desk");
	mappedLocs.push_back("Greg's Desk");
	mappedLocs.push_back("Robot Arm 2");

	ros::Duration twenty_sec = ros::Duration(20);
	
	while(ros::ok())
	{
		for (std::vector<std::string>::iterator it = mappedLocs.begin(); it !=mappedLocs.end(); ++it)
		{
			msg.data = *it;
			std::cout << "Now moving to " << msg.data << std::endl;
			pub.publish(msg);
			twenty_sec.sleep();
		}
	}

	ros::spin();
	return 0;
}