#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

class scanner_clouds
{
private:
  bool updateCloud;
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr laser_cloud_ptr; 

public:
  scanner_clouds()
  : updateCloud(false)
  {
    viewer = scanner_clouds::viewerCreator();
  }
  ~scanner_clouds() {}

  void laserCallback(const sensor_msgs::PointCloud& msg )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    readCloud(msg, cloud);

    if(ros::ok() && !viewer->wasStopped())
    {
      viewer->setSize(400, 400);
      viewer->addPointCloud<pcl::PointXYZ> (this->laser_cloud_ptr, "laser_ptcloud");
      viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "laser_ptcloud");
      viewer->spinOnce(10);
      boost::this_thread::sleep(boost::posix_time::microseconds(100));

      if(updateCloud)
      {      
        viewer->removePointCloud("laser_ptcloud");
        viewer->updatePointCloud(this->laser_cloud_ptr, "laser_ptcloud");
      }
      updateCloud = true;
    }
  }

  void readCloud(const sensor_msgs::PointCloud& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ptr)
  {
    sensor_msgs::PointCloud2 msg_pc2;
    pcl::PCLPointCloud2 pcl_pc2;
    sensor_msgs::convertPointCloudToPointCloud2(msg, msg_pc2); 
    pcl_conversions::toPCL(msg_pc2, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_ptr);
    this->laser_cloud_ptr = cloud_ptr;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerCreator()
  {        
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Laser Clouds 3D"));
    viewer->setBackgroundColor (0.2, 0.3, 0.3);
    viewer->addCoordinateSystem (1.0);    //don't want no cylinder
    viewer->setSize(400, 400);
    viewer->initCameraParameters ();    
    return viewer;
  }

};


int main(int argc, char** argv)
{
	ros::init(argc, argv, "laser_clouds");

	ros::NodeHandle n;

  scanner_clouds sc;
	ros::Subscriber sub = n.subscribe("/RosAria/lms5XX_1_pointcloud", 1000, &scanner_clouds::laserCallback, &sc);

	ros::spin();

	return 0;
}