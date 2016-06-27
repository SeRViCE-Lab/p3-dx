[Table of Contents](#table-of-contents)

[Introduction](#introduction)

[Dependencies](#dependencies)

[Bring up the Robot](#bring-up-the-robot)

	- [Stream Sonar Point Clouds](#stream-sonar-point-clouds)

	- [Stream Laser 2D clouds (sensor_msgs/PointCloud)](#stream-laser-2D-clouds-(sensor_msgs/pointCloud))

	- [Stream Laser 3D Point clouds](#stream-laser-3d-point-clouds)

	- [Publish dynamic tf transform Twist messages to robot](#publish-dynamic-tf-transform-twist-messages-to-robot)

	- [Start all the nodes above simultaneously](#start-all-the-nodes-above-simultaneously)

	- [Navigate the environment using ROS' navigation stack](#Navigate-the-environment-using-ROS'-navigation-stack)

[Questions](#questions)


###Introduction

This repo also has a set of clients that pushes velocity commands to the P3_DX robot from [adept mobile robots](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx). The package supports indoor localization and dynamic SLAM via an adaptive Monte Carlo Localization (AMCL) for mobile robots as described by Sebastian Thrun, Wolfram Burgard, and Dieter Fox in their book, <i>Probabilistic Robotics, Intelligent Robotics and Autonomous Agent</i>. To aid faster implementation time, we have developed the code in ROS. In addition to sending velocity commands, and performing dynamic SLAM based on LIDAR data, it subscribes to the [RosAria package's](wiki.ros.org/rosaria) sonar scans, laser scans, and projected 3D laser scans(point clouds) and provides a <i>400 X 400</i> pixels window to visualize these topics in real-time. Example point clouds from the sonars and laser scanners are provided below:

<!-- <figure>
<img src="http://www.mobilerobots.com/Libraries/Site_Images/P3-DXwith_ball_2.sflb.ashx" height="250px" width= "250px">
<figcaption>P3-DX Robot</figcaption>
</figure>
<figure>
<a href="https://youtu.be/lYgp8qZjvks">
	<img src="https://i.ytimg.com/vi/lYgp8qZjvks/2.jpg?time=1466973717005" height="250px" width="250px"></a>
	<figcaption>Laser 3D Point Clouds</figcaption>
</figure> -->
<div class="fig figcenter fighighlight">
<a href="">
	<img src="http://www.mobilerobots.com/Libraries/Site_Images/P3-DXwith_ball_2.sflb.ashx" height="250px" width="250px" align="left"></a>
<a href="https://youtu.be/lYgp8qZjvks">
	<img src="https://i.ytimg.com/vi/lYgp8qZjvks/2.jpg?time=1466973717005" height="250px" width="250px" alight="right"></a>
	<div class="figcaption" align="left">P3-DX Robot
	<div class="figcaption" align="middle">Laser 3D Point Clouds</div></div>
</div>

<br></br>
<div class="fig figcenter fighighlight">
<a href="https://youtu.be/B871f3qa1p4">
	<img src="https://i.ytimg.com/vi/B871f3qa1p4/2.jpg?time=1466973686757" height="250px" width="250px" align="left"></a>
	<img src="https://i.ytimg.com/vi/PYT4FCIVYgw/1.jpg?time=14669736586347" height="250px" width="250px" alight="right"></a>
	<div class="figcaption" align="left">Laser 2D Point Clouds
	<div class="figcaption" align="middle">Sonar 3D Point Clouds</div></div>
</div>
<!-- <figure>
<a href="https://youtu.be/PYT4FCIVYgw">
	<img src="https://i.ytimg.com/vi/PYT4FCIVYgw/1.jpg?time=14669736586347" height="250px" width="250px"></a>
	<figcaption>Sonar 3D Point Clouds</figcaption>
</figure> -->

[Aria](http://www.mobilerobots.com/Software/ARIA.aspx) package and [Arnl](http://www.mobilerobots.com/Software/NavigationSoftware.aspx) package from Adept. 


Here is an example video of the navigation of the robot based on velocities commands that are sent to the robot after receiving the `TF` transforms broadcaster from the `rosaria` package:

<div class="fig figcenter fighighlight">
<a href="https://youtu.be/yczG8CUbK2M">
	<img src="https://i.ytimg.com/vi/yczG8CUbK2M/1.jpg?time=1466972319359" height="300px" width="400px"></a>
	</br>
	<div class="figcaption" align="middle">Twist commands to the p3_dx robot</div>
</div>
<!-- [![Twist commands to the p3_dx robot](https://i.ytimg.com/vi/yczG8CUbK2M/1.jpg?time=1466972319359)](https://youtu.be/yczG8CUbK2M)  -->

To be able to compile these codes, you would want to pull the files from the links indicated above. 

### Dependencies

Please install [Aria](http://robots.mobilerobots.com/wiki/ARIA#Download_Aria) and [Arnl](http://robots.mobilerobots.com/wiki/ARNL,_SONARNL_and_MOGS#Download) packages by following the intructions on the links. Also, you would want to install the ros wrappers to the Aria and Arnl packages namely: [rosarnl](https://github.com/MobileRobots/ros-arnl) and [rosaria](http://wiki.ros.org/ROSARIA). 

In addition to the above dependencies, you would preferrably want to compile the code using c++11. On Linux, ensure you have at least g++ 4.8 and pass the `-std=c++11` to the CMakeLists.txt files (this is already done by default).

When you are all set, you can clone this repo to your catkin workspace `src` folder and build with 

```bash
	catkin_make --source src/p3_dx
```
### Bring up the Robot

When the compilation finishes, you could run each individual executable as follows:

#### 1. Stream Sonar Point Clouds

```bash
	rosrun sonar_clouds sonar_clouds
```

Remember to click on the PCL window and zoom out the clouds for visibility.

#### 2. Stream Laser 2D clouds (sensor_msgs/PointCloud)

```bash
	rosrun laser_scans laser_scans
```

#### 3. Stream Laser 3D Point clouds

```bash
	rosrun scanner_clouds scanner_clouds
```

#### 4. Publish dynamic `tf transform` Twist messages to the `RosAria` topic: /RosAria/cmd_vel

After retrieving RosAria's latest published transforms, from the `odometry` frame -> `base_link` -> `laser frame`, we generate the transform from the origin to a new pose at time t_1 and we move linearly along x according to the following relation:

```lua
vel_msg.linear.x = 0.5 * sqrt(pow(transform.getOrigin().x(), 2) +
                              pow(transform.getOrigin().y(), 2));
``` 
and orient the robot along `z` according to:

```lua
vel_msg.angular.z = 4.0 * atan2(transform.getOrigin().y(),
                                transform.getOrigin().x());

```

```bash
	rosrun tf_listener tf_listener
```

#### 5. Start all the nodes above simultaneously

```bash
	roslaunch p3dx_2dnav p3_dx.launch
```

#### 6. Navigate the environment using ROS' navigation stack 
 This uses the adaptive monte carlo localization algorithm as thoroughly discussed by Dieter Fox, Thrun, and colleagues in their book, <i>probabilistic roboticsM </i>. 

```bash
	roslaunch p3dx_2dnav move_base.launch
```

A static map of the environment (generated with [openslam's gmapping](http://openslam.org/gmapping.html)) that was used for development is provided in the directory [p3dx_2dnav/map/map.pgm](/p3dx_2dnav/map/map.pgm). Feel free to create your own map and feed it to the robot by following the [Ros Map Server Tutorial](http://wiki.ros.org/map_server). Also provided along with the map are the global costmap, local costmap, base local planner and costmap common parameters that are used in setting up the ROS navigation stack when you bring up the robot 

 This will navigate the environment with all the robot's sensors and will dynamically update map of the environment based on real-time acquired sensor information. 


### Questions

Please use the issues page. Thanks!

