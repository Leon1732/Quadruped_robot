This project models a quadruped robot in ROS and controls it to walk.

<img src="https://https://github.com/Leon1732/Quadruped_robot/blob/main/robotwalk.gif" width="640" alt="EKF pic">

Before running the code, compile by typing the following command in the terminal：

```terminal
cd ~/robotic00_ws
catkin_make
```

Then execute the following commands in sequence：

```terminal
roslaunch urdf_gazebo four_leg_gazebo.launch
roslaunch MYROBOT_control MYROBOT_control.launch
rosrun MYROBOT_control MYROBOT_control.py
```
