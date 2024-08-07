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
