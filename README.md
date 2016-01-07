## WAMpy ##
======

WAMpy is a python library for interacting with a simulated model of the Barrett WAM manipulator using OpenRAVE.
WAMpy extends the robot-agnostic helper library [PRpy](https://github.com/personalrobotics/prpy) by adding robot-specific functionality for the the WAM robot.
It is based on similiar robot-specific packages like [HERBpy](https://github.com/personalrobotics/herbpy) and [PR2py](https://github.com/personalrobotics/pr2py).

This software was created by the [Personal Robotics Lab] (https://personalrobotics.ri.cmu.edu) at [Carnegie Mellon University] (http://www.cmu.edu). 


### Usage examples ###

Load the Barrett WAM robot model and display it in Rviz viewer.  
```
$ roscore
$ rosrun wampy 01_WAM_arm.py
```
![ex1_screenshot](http://raw.github.com/personalrobotics/wampy/master/screenshots/01_WAM_arm.png)

.

Initialize an environment with the Barrett WAM arm on a Segway base. 
Grasp a Fuze drink bottle and lift it from the table.
This simulation tests the functionality of planning and grasping using object-specific TSRs. The actual grasp is performed by attaching the object kinbody to the gripper kinbody.
```
$ roscore
$ rosrun wampy 02_WAM_Segway_grasp_object.py
```
![ex2_screenshot](http://raw.github.com/personalrobotics/wampy/master/screenshots/02_WAM_Segway_grasp_object.png)

.

Test the OpenRAVE physics engine. 
The Segway base is set as a static link, so it doesn't fall over due to gravity.
The Barrett WAM arm should go limp because there is no joint controller configured.
```
$ roscore
$ rosrun wampy 03_WAM_Segway_physics.py
```
![ex3_screenshot](http://raw.github.com/personalrobotics/wampy/master/screenshots/03_WAM_Segway_physics.png)

