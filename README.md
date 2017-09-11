# Drone-visual-servoing-with-a-controller-in-MATLAB
A ROS package to control a copter so that it follows ArUco patterns with the bottom camera. When using in Loiter mode, the system uses a controller in ROS. When the drone is in Guided mode, the system is connected to a controller in MATLAB/Simulink. 

# Video demo
[![Video demo](https://img.youtube.com/vi/e9FENWM6F0Q/0.jpg)](https://www.youtube.com/watch?v=e9FENWM6F0Q)

# To do list
* Modify the ArUco model to make it move in a square slowly;
* Add a PID controller to make a close loop;
* Send data to Terminal and copy them to text file;
* Add FOPD controller and log data;
* Plot curve to compare the captured data.

# Setup steps:

* Set up development environment according to 
http://docs.erlerobotics.com/simulation/configuring_your_environment

* Run the following commands to set up this project:

```bash
# Within a catkin directory
cd src
git clone https://github.com/cnpcshangbo/ros_erle_pattern_follower
cd ..; 
catkin_make --pkg ros_erle_pattern_follower
```

* Install MATLAB Robotics System Toolbox
https://www.mathworks.com/help/robotics/index.html?s_cid=doc_flyout

* Install the package to support custom message type and add the custom message to MATLAB
https://www.mathworks.com/matlabcentral/answers/355617-robot-system-toolbox-doesn-t-support-the-message-type-mavros_msgs-positiontarget

# Start the simulation
* Launch Ardupilot & Gazebo

```bash
Terminal 1:

source ~/simulation/ros_catkin_ws/devel/setup.bash
cd ~/simulation/ardupilot/ArduCopter
../Tools/autotest/sim_vehicle.sh -j 4 -f Gazebo
# Once MAVProxy has launched completely, load the parameters
param load /home/bshang2/simulation/ardupilot/Tools/Frame_params/Erle-Copter.param
# Change the "bshang2" to your username.

Terminal 2:
source ~/simulation/ros_catkin_ws/devel/setup.bash
roslaunch ardupilot_sitl_gazebo_plugin erlecopter_mark.launch

Terminal 1:
mode GUIDED
arm throttle
takeoff 2
param set SYSID_MYGCS 1
```

* Run this project

```bash
source ~/simulation/ros_catkin_ws/devel/setup.bash
rosrun ros_erle_pattern_follower ros_erle_pattern_follower
```
* Launch MATLAB and type command: "rosinit"
* Run the Simulink file

Simulating this package
------------------------
YouTube video:

[![YouTube video](http://img.youtube.com/vi/xNengdC0_8s/0.jpg)](http://www.youtube.com/watch?v=xNengdC0_8s)

Learn how to make this work on Gazebo simulator [here](http://docs.erlerobotics.com/simulation/vehicles/erle_copter/tutorial_5).

Run related Simulink file
---------------------
Launch MATLAB;
On the MATLAB command line, execute the following:
```matlab
rosinit
```
Open this Simulink file (pattern_follow_body_frame.slx) and run the model.

Learn more:

https://www.mathworks.com/help/robotics/examples/get-started-with-ros-in-simulink.html

https://www.mathworks.com/matlabcentral/answers/355617-robot-system-toolbox-doesn-t-support-the-message-type-mavros_msgs-positiontarget

Support forums and chats
------------------------

Please ask your questions not related to bugs/feauture requests on:
- [Erle Robotics Forum](http://forum.erlerobotics.com/)


Links
-----

  - [Erle Robotics](www.erlerobotics.com)
  
Acknowledgment
-----------------
  - [ros_erle_pattern_follower project](https://github.com/erlerobot/ros_erle_pattern_follower)
