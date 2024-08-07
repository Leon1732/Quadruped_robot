Before running the code, compile by typing the following command in the terminal：

```sh
cd ~/robotic00_ws
catkin_make


Then execute the following commands in sequence：

```sh
roslaunch urdf_gazebo four_leg_gazebo.launch
roslaunch MYROBOT_control MYROBOT_control.launch
rosrun MYROBOT_control MYROBOT_control.py

