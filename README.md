# ros_erle_pattern_follower
A ROS package to control a copter so that it follows ArUco patterns with the bottom camera.

# To do list
1. Connect this simulator to a FOPD controller
1.1 Let this node publish a topic called `\matlab\err`;
1.2 Modify the ArUco model to make it not move;
1.3 Test if Simulink can get the positon error (Topic `\matlab\err`) as expected;
1.4 Add a P controller to make a close loop;
1.5 Add a PID controller to make a close loop;
1.6 Send data to Terminal and copy them to text file;
1.7 Add FOPD controller and log data;
1.8 Plot curve to compare the captured data.

Quick setup:
```bash
# Within a catkin directory
cd src
git clone https://github.com/erlerobot/ros_erle_pattern_follower
cd ..; 
catkin_make --pkg ros_erle_pattern_follower

source devel/setup.bash
rosrun ros_erle_pattern_follower ros_erle_pattern_follower
```

Simulating this package
------------------------
YouTube video:

[![YouTube video](http://img.youtube.com/vi/xNengdC0_8s/0.jpg)](http://www.youtube.com/watch?v=xNengdC0_8s)

Learn how to make this work on Gazebo simulator [here](http://docs.erlerobotics.com/simulation/vehicles/erle_copter/tutorial_5).

Run related Simulink file
---------------------
File name: cmd_vel.slx
Launch MATLAB;
On the MATLAB command line, execute the following:
```matlab
rosinit
```
Open this Simulink file and run the model.

Learn more [here](https://www.mathworks.com/help/robotics/examples/get-started-with-ros-in-simulink.html)

Support forums and chats
------------------------

Please ask your questions not related to bugs/feauture requests on:
- [Erle Robotics Forum](http://forum.erlerobotics.com/)


Links
-----

  - [Erle Robotics](www.erlerobotics.com)
