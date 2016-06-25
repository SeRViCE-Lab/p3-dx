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

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;

public:
  sonar_clouds()
  : updateCloud(false) 
  {    
    viewer = sonar_clouds::viewerCreator();
  }
  ~sonar_clouds(){}

  void sonarCallback(const sensor_msgs::PointCloud2& msg )
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    readCloud(msg, cloud);

    if(ros::ok())
    {     
     viewer->addPointCloud<pcl::PointXYZ> (this->cloud, "sonar_cloud");     
     viewer->setSize(400, 400);
     viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sonar_cloud");
     viewer->spinOnce(10);
     boost::this_thread::sleep(boost::posix_time::microseconds(100));

     if(msg.header.seq >=2)
     {      
       viewer->removePointCloud("sonar_cloud");
       viewer->updatePointCloud(this->cloud, "sonar_cloud");
     }
    }

  }

  void readCloud(const sensor_msgs::PointCloud2& msg, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
  {
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    this->cloud = cloud;
  }

  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewerCreator()
  {    
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("Sonar Clouds 3D"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addCoordinateSystem (1.0);    //don't want no cylinder
    viewer->setSize(400, 400);
    viewer->initCameraParameters ();
    viewer->registerKeyboardCallback (&sonar_clouds::keyboardEventOccurred, *this);
    return viewer;
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