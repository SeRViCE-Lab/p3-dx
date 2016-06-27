/*
* Author: Olalelan Ogunmolu
* Boston, MA
* June 2016
* Licensed under the MIT License
*/

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf2_msgs/TFMessage.h>

class tf_listener
{
private:
	tf::TransformListener listener;
	std::string target_frame;
	std::string source_frame;
	ros::Duration timeout, polling_duration;
	tf::StampedTransform transform;

	ros::NodeHandle n_tf_;
	bool pirouette;

public:
	tf_listener(ros::NodeHandle n_tf, bool pirouette) 
	: n_tf_(n_tf), pirouette(pirouette)	
	{}
	~tf_listener() {}
	void gen_vel();

	void tfCallback(const tf2_msgs::TFMessage& tf_msg)
	{		
		for(auto it = tf_msg.transforms.begin(); it != tf_msg.transforms.end(); ++it)
		{		
			target_frame = (*it).header.frame_id;
			source_frame = (*it).child_frame_id;
			timeout = ros::Duration(0.2);
			polling_duration = ros::Duration(0.01);
		}

/*		if(!listener.waitForTransform(target_frame, \
		              source_frame, ros::Time(0), polling_duration));
		{
		   return;
		}*/
		try
		{
			listener.lookupTransform(target_frame, source_frame, \
				ros::Time(0), transform); 
		}
		catch (tf::TransformException &ex) 
		{
		      ROS_ERROR("%s",ex.what());
		      ros::Duration(1.0).sleep();	    
		}

		//resulting transform now in the transform object
		if(pirouette)
		{
			gen_vel(); //move linearly along x and orientation along
		}  
	}
};

//calculate linear and angular velocities for the robot based on the transform
void tf_listener::gen_vel()
{
	geometry_msgs::Twist vel_msg;
	vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
	                                transform.getOrigin().x());
	vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
	                              pow(transform.getOrigin().y(), 2));
	ros::Publisher rosaria_vel = n_tf_.advertise<geometry_msgs::Twist>("/RosAria/cmd_vel", 10);
	rosaria_vel.publish(vel_msg);
	ros::Rate rate(5.0);
	rate.sleep();
}

int main (int argc, char** argv)
{
	ros::init(argc, argv, "tf_listener_node");
	ros::NodeHandle n_tf;
	ros::Subscriber sub;
	bool pirouette;

	std::vector<std::string> args(argv, argv+argc);
	for (size_t i = 1; i < args.size(); ++i) 
	{
	   if (args[i] == "-p" || args[i] == "-pirouette" ) 
			pirouette = true;
	}

	tf_listener tf(n_tf, pirouette);
	sub = n_tf.subscribe("/tf", 1000, &tf_listener::tfCallback, &tf);
	ros::spin();

	return EXIT_SUCCESS;
}