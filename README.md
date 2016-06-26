[Table of Contents](table-of-contents)

###Intro

This package supports indoor localization and dynamic SLAM via an adaptive Monte Carlo Localization for mobile robots as described by Thrun, Dieter fox and colleagues in their book, Probabilistic Robotics. To aid faster implementation time, we have developed the code in ROS. This repo also has a set of clients that pushes velocity commands to the P3_DX robot from [adept mobile robots](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx). In addition to sending velocity commands, and performing dynamic SLAM based on LIDAR data, it subscribes to the [RosAria package's](wiki.ros.org/rosaria)sonar scans, laser scans, and projected 3D laser scans(point clouds) and provides a <i>400 X 400</i> pixels window to visualize these topics in real-time. Example point clouds from the sonars and laser scanners are provided below:

![P3_dx and Sick LIDAR](http://www.mobilerobots.com/Libraries/Site_Images/P3-DXwith_ball_2.sflb.ashx)
[![Laser 3D Point Clouds](https://i.ytimg.com/vi/lYgp8qZjvks/2.jpg?time=1466973717005)](https://youtu.be/lYgp8qZjvks)
[![Laser 2D Point Clouds](https://i.ytimg.com/vi/B871f3qa1p4/2.jpg?time=1466973686757)](https://youtu.be/B871f3qa1p4)
[![Sonar 3D Point Clouds](https://i.ytimg.com/vi/PYT4FCIVYgw/1.jpg?time=1466973658634)](https://youtu.be/PYT4FCIVYgw)

[Aria](http://www.mobilerobots.com/Software/ARIA.aspx) package and [Arnl](http://www.mobilerobots.com/Software/NavigationSoftware.aspx) package from Adept. 


Here is an example video of the navigation of the robot based on velocities commands that are sent to the robot after receiving the `TF` transforms broadcaster from the `rosaria` package:

[![Twist commands to the p3_dx robot](https://i.ytimg.com/vi/yczG8CUbK2M/1.jpg?time=1466972319359)](https://youtu.be/yczG8CUbK2M) 

To be able to compile these codes, you would want to pull the files from the links indicated above. 

### Dependencies

Please install [Aria](http://robots.mobilerobots.com/wiki/ARIA#Download_Aria) and [Arnl](http://robots.mobilerobots.com/wiki/ARNL,_SONARNL_and_MOGS#Download) packages by following the intructions on the links. Also, you would want to install the ros wrappers to the Aria and Arnl packages namely: [rosarnl](https://github.com/MobileRobots/ros-arnl) and [rosaria](http://wiki.ros.org/ROSARIA). 

After you are all set, you can clone this repo to your catkin workspace `src` folder and build with 

```bash
	catkin_make --source src/p3_dx
```

When the compilation finishes, you could run each individual executable as follows:

### 1. Stream Sonar Point Clouds

```bash
	rosrun sonar_clouds sonar_clouds
```

Remember to click on the PCL window and zoom out the clouds for visibility.

### 2. Stream Laser 2D clouds (sensor_msgs/PointCloud)

```bash
	rosrun laser_scans laser_scans
```

### 3. Stream Laser 3D Point clouds

```bash
	rosrun scanner_clouds scanner_clouds
```

### 4. Publish dynamic `tf transform` Twist messages to the `RosAria` topic /RosAria/cmd_vel

```bash
	rosrun tf_listener tf_listener
```

### 5. Start all the nodes above simultaneously

```bash
	roslaunch p3dx_2dnav p3_dx.launch
```

### 6. Navigate the environment intelligently 
 This uses the adaptive monte carlo localization algorithm as thoroughly discussed by Dieter Fox, Thrun, and colleagues in their book, <i>probabilistic roboticsM </i>. 

```bash
	roslaunch p3dx_2dnav move_base.launch
```

A static map of the environment used for development is provided in the directory [p3dx_2dnav/map/map.pgm](p3dx_2dnav/map/map.pgm). Feel free to create your own map and feed it to the robot by following the [Ros Map Server Tutorial](wiki.ros.org/map_server). Also provided along with the map are the global costmap, local costmap, base local planner and costmap common parameters that are used in setting up the ROS navigation stack when you bring up the robot 

 This will navigate the environment with all the robot's sensors and will dynamically update map of the environment based on real-time acquired sensor information. 


### Questions

Please use the issues page. Thanks!

