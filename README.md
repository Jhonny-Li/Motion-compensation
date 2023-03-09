# Event-based Real-time Moving Object Detection Based On IMU Egomotion Compensation

The improved nonlinear motion compensation algorithm from the ICRA contributed paper “Event-based Real-time Moving Object Detection Based On IMU Egomotion Compensation”is presented here.

## Disclaimer:

Please note that only the nonlinear motion compensation code mentioned in the paper is open sourced here, and we will provide the source code and corresponding demo to ensure the reproducibility of the algorithm.

## Requirement

1.The code was tested successfully on Ubuntu 18.04 and 20.04. And We recommend Ubuntu 20.04, because it can avoid some potential problems.

2.Running the code requires the support of ROS, please install the ROS corresponding to the Ubuntu version first.
 - [ROS melodic installation](http://wiki.ros.org/melodic/Installation/Ubuntu "Read this to install ROS melodic on your system")
 - [ROS Noetic installation](http://wiki.ros.org/noetic#Installation "Read this to install ROS Noetic on your system")
 - [ROS Tutorials](http://wiki.ros.org/ROS/Tutorials "ROS tutorials")


## Driver Installation

To run the code, Please make sure that the event camera ros driver is installed on your computer.
 - [ROS packages for DVS](https://github.com/uzh-rpg/rpg_dvs_ros "Read this to install Event Camera Driver")
 
## Download the project and Running the Code

1.Download the project:
```
cd ~/catkin_ws/src
git clone https://github.com/Jhonny-Li/Motion-compensation.git
```

2.Build the project:
```
cd ~/catkin_ws
catkin build
```

3.Play the demo_bag

```
roscore #open a new terminal
```

play the bag in a new terminal: 
```
cd ~/catkin_ws/src/bags
rosbag play demo_bag
```

4.Run the Motion Compensation node by a launch file:
```
source ~/catkin_ws/devel/setup.bash
roslaunch datasync motion_compensation.launch
```

5.Open a image viewer to display the motion compensation results:
* `rosrun rqt_image_viewer rqt_image_viewer`  (select topic：count_image)

## Parameters
