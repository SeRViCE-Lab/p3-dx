[Table of Contents](table-of-contents)

###Intro

This package supports indoor localization and dynamic SLAM via an adaptive Monte Carlo Localization for mobile robots as described by Thrun, Dieter fox and colleagues in their book, Probabilistic Robotics. To aid faster implementation time, we have developed the code in ROS. This repo also has a set of clients that pushes velocity commands to the P3_DX robot from [adept mobile robots](http://www.mobilerobots.com/ResearchRobots/PioneerP3DX.aspx). In addition to sending velocity commands, and performing dynamic SLAM based on LIDAR data, it subscribes to the [RosAria package's](wiki.ros.org/rosaria)sonar scans, laser scans, and projected 3D laser scans(point clouds) and provides a <i>400 X 400</i> pixels window to visualize these topics in real-time. Example point clouds from the sonars and laser scanners are provided below:

![P3_dx and Sick LIDAR](/p3_dx_2dnav/map_data/p3_dx.jpg). 

[Aria](http://www.mobilerobots.com/Software/ARIA.aspx) package and [Arnl](http://www.mobilerobots.com/Software/NavigationSoftware.aspx) package from Adept. 

[![Laser 3D Point Clouds](/p3_dx_2dnav/map_data/laser3d.png)](https://youtu.be/lYgp8qZjvks)

[![Laser 2D Point Clouds](/p3_dx_2dnav/map_data/laser2d.png)](https://youtu.be/B871f3qa1p4)

[![Sonar 3D Point Clouds](/p3_dx_2dnav/map_data/sonar3d.png)](https://youtu.be/PYT4FCIVYgw)

Here is an example video of the navigation of the robot based on velocities commands that are sent to the robot after receiving the `TF` transforms broadcaster from the `rosaria` package:

[![Twist commands to the p3_dx robot](https://i.ytimg.com/vi/yczG8CUbK2M/1.jpg?time=1466972319359)](https://youtu.be/yczG8CUbK2M) 

To be able to compile these codes, you would want to pull the files from the links indicated above. 