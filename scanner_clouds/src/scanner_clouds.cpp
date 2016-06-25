#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <iostream>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

void laserCallback(const sensor_msgs::PointCloud::ConstPtr& msg )
{
	ROS_INFO_STREAM(" arriving " << msg->header.frame_id <<  " channels= ");
}


boost::shared_ptr<pcl::visualization::PCLVisualizer> simpleVis (pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addPointCloud<pcl::PointXYZ> (cloud, "profile cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);    //don't want no cylinder
  viewer->setSize(640, 480);
  viewer->initCameraParameters ();
  return (viewer);
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_clouds");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/RosAria/lms5XX_1_pointcloud", 1000, laserCallback);

	ros::spin();

	return 0;
}