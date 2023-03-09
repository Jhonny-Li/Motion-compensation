# Event-based Real-time Moving Object Detection Based On IMU Egomotion Compensation

The improved nonlinear motion compensation algorithm from the ICRA contributed paper “Event-based Real-time Moving Object Detection Based On IMU Egomotion Compensation”is presented here.

#Disclaimer:

Please note that only the nonlinear motion compensation code mentioned in the paper is open sourced here, and we will provide the source code and corresponding demo to ensure the reproducibility of the algorithm.

#Requirement

1.The code was tested successfully on Ubuntu 18.04 and 20.04. And We recommend Ubuntu 20.04, because it can avoid some potential problems.

2.Running the code requires the support of ROS, please install the ROS corresponding to the Ubuntu version first.
 - [ROS melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu "Read this to install ROS melodic on your system")
 - [ROS Noetic installation](http://wiki.ros.org/noetic#Installation "Read this to install ROS Noetic on your system")
 - [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials "ROS tutorials")


#Driver Installation

To run the code,Please make sure that the event camera driver is installed on your computer.
 - [ROS packages for DVS](https://github.com/uzh-rpg/rpg_dvs_ros "Read this to install Event Camera Driver")
 
