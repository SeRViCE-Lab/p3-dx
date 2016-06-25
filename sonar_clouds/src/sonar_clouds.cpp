#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
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

class sonar_clouds
{
private:
  typedef pcl::PCLPointCloud2 pcl2;
  typedef pcl::PointCloud<pcl::PointXYZ> pointXYZ;

  pcl2 pcl_pc2;
  pointXYZ pcl_cloud;

  std::mutex lock;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud ;
  bool updateCloud;

public:
  sonar_clouds()
  : updateCloud(false) {}
  ~sonar_clouds(){}

  void sonarCallback(const sensor_msgs::PointCloud2& msg )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    readCloud(msg, cloud);

    lock.lock();
    this->cloud = cloud;
    updateCloud=true;
    lock.unlock();
  }

  void readCloud(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl2 pcl_pc2;
    pointXYZ pcl_cloud;
    pcl::PCLPointCloud2 pcl_pc;
    pcl::fromPCLPointCloud2(pcl_pc, *cloud);

 //all of these work but vizer doesn't like it
  //so I am commenting 'em out till we figure out a better way of minimally
  //converting rosmsgs to pcl' 
  /*  
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl_conversions::copyPointCloud2MetaData (msg, pcl_pc2);
    pcl_conversions::toPCL(msg, pcl_pc);
    pcl::fromROSMsg (msg, pcl_cloud);
    this->pcl_pc2 = pcl_pc2;
    this->pcl_cloud = pcl_cloud;*/

    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Sonar Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud, "sonar cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sonar cloud");
    viewer->addCoordinateSystem (1.0);    //don't want no cylinder
    viewer->setSize(640, 480);
    viewer->initCameraParameters ();

    while(!viewer->wasStopped() && ros::ok())
    {
      if(updateCloud)
      {
        lock.lock();
        cloud = this->cloud;
        updateCloud = false;
        lock.unlock();

        viewer->setSize(640, 480);
        viewer->updatePointCloud(cloud, "sonar cloud");
      }
      viewer->spinOnce(100);
    }
    viewer->close();
  }

  void show_cloud()
  {

  }

  void keyboardEventOccurred (const pcl::visualization::KeyboardEvent &event,
                              void* viewer_void)
  {    
    unsigned int text_id = 0;
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer = *static_cast<boost::shared_ptr<pcl::visualization::PCLVisualizer> *> (viewer_void);
    if (event.getKeySym () == "r" && event.keyDown ())
    {
      std::cout << "r was pressed => removing all text" << std::endl;

      char str[512];
      for (unsigned int i = 0; i < text_id; ++i)
      {
        sprintf (str, "text#%03d", i);
        viewer->removeShape (str);
      }
      text_id = 0;
    }
  }

};




int main(int argc, char** argv)
{
	ros::init(argc, argv, "sonar_clouds");

	ros::NodeHandle n_son;
  sonar_clouds sc;
	ros::Subscriber sub = n_son.subscribe("/RosAria/sonar_pointcloud2", 1000, &sonar_clouds::sonarCallback, &sc);

	ros::spin();

	return 0;
}