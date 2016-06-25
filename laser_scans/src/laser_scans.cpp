#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>
#include <mutex>

#include <pcl/features/normal_3d.h>
#include <pcl/common/common_headers.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <tf/transform_listener.h>
#include <laser_geometry/laser_geometry.h>

class laser_clouds
{
private:
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud;
  ros::AsyncSpinner spinner;

  std::mutex lock;
  bool updateCloud;

public:
  laser_clouds();
  ~laser_clouds(){}
  void init();
  //high fidelity projection
  void laserCallback(const sensor_msgs::LaserScan::ConstPtr& msg )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZ>);
    if(!listener_.waitForTransform(msg->header.frame_id, \
                  "/base_link", msg->header.stamp +\
                  ros::Duration().fromSec(msg->ranges.size()* \
                  msg->time_increment), ros::Duration(1.0)))
    {
       return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_link",*msg,
            cloud,listener_);

    // lock.lock();
    readCloud(cloud, pclCloud);    
    this->pclCloud = pclCloud;
    updateCloud = false;
    // lock.unlock();
  }

  void readCloud(const sensor_msgs::PointCloud sensorCloud, pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud)
  {
    pcl::PointXYZ points;

    for(auto it=sensorCloud.points.begin(); it!=sensorCloud.points.end(); ++it )
    {      
      points.x = (*it).x;
      points.y = (*it).y;
      points.z = (*it).z;
      pclCloud->points.push_back(points);

      ROS_INFO_STREAM("sensorCloud: " << *it);
    }

    // ROS_INFO_STREAM("sensorCloud: " << sensorCloud);
    // ROS_INFO_STREAM("pclCloud " << *pclCloud);
    ROS_INFO("updateCloud status: %d .", updateCloud);

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("Laser_Cloud_Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (pclCloud, "laser_cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "laser_cloud");
    viewer->addCoordinateSystem (1.0);    
    viewer->setSize(640, 480);
    viewer->initCameraParameters ();

    while(!viewer->wasStopped() && ros::ok())
    {
      if(updateCloud)
      {
        // lock.lock();
        pclCloud = this->pclCloud;
        updateCloud = false;
        // lock.unlock();

        viewer->setSize(640, 480);
        viewer->updatePointCloud(pclCloud, "laser cloud");
      }
      viewer->spinOnce(100);
    }
    viewer->close();
  }
};

laser_clouds::laser_clouds()
  : updateCloud(true), spinner(0) {}

void laser_clouds::init()
{
  spinner.start();
}

int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_clouds");

	ros::NodeHandle n_laser;
  laser_clouds ls;
	ros::Subscriber sub = n_laser.subscribe("/RosAria/lms5XX_1_laserscan", 1000, &laser_clouds::laserCallback, &ls);

	ros::spin();

	return 0;
}