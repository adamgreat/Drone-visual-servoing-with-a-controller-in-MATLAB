# Drone-visual-servoing-with-a-controller-in-MATLAB
A ROS package to control a copter so that it follows ArUco patterns with the bottom camera. When using in Loiter mode, the system uses a controller in ROS. When the drone is in Guided mode, the system is connected to a controller in MATLAB/Simulink. 

# Video demo
[![Video demo](https://img.youtube.com/vi/e9FENWM6F0Q/0.jpg)](https://www.youtube.com/watch?v=e9FENWM6F0Q)

# To do list
* Test FOPD controller;
* Add a PID controller to make a close loop;
* Send data to Terminal and copy them to text file;
* Add FOPD controller and log data;
* Plot curve to compare the captured data.

# Setup steps:

Please read this page:
https://github.com/cnpcshangbo/Drone-visual-servoing-with-a-controller-in-MATLAB/wiki/Setting-up

# Start the simulation

Please read this page:
https://github.com/cnpcshangbo/Drone-visual-servoing-with-a-controller-in-MATLAB/wiki/Start-the-simulation

Support forums and chats
------------------------

Please ask your questions not related to bugs/feauture requests on:
- [Erle Robotics Forum](http://forum.erlerobotics.com/)

References:
-----

  - [Erle Robotics](www.erlerobotics.com)

  - YouTube video for Simulating the package 
  
  [![YouTube video](http://img.youtube.com/vi/xNengdC0_8s/0.jpg)](http://www.youtube.com/watch?v=xNengdC0_8s)

  - [Learn how to make this work on Gazebo simulator](http://docs.erlerobotics.com/simulation/vehicles/erle_copter/tutorial_5).
  
  - [ros_erle_pattern_follower project](https://github.com/erlerobot/ros_erle_pattern_follower)

  - [Get started with ROS in Simulink](https://www.mathworks.com/help/robotics/examples/get-started-with-ros-in-simulink.html)

  - [Custom message type in MATLAB](https://www.mathworks.com/matlabcentral/answers/355617-robot-system-toolbox-doesn-t-support-the-message-type-mavros_msgs-positiontarget)
