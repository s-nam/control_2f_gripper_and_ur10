# Guidelines to control UR10 and Robotiq 2F gripper
This package contains Python codes to control UR10 (CB series) and Robotiq 2F gripper.

## 1. Install this package
In `catkin_ws/src` folder,
```consol
$ git clone https://github.com/s-nam/control_2f_gripper_and_ur10.git
```
Next, go to `catkin_ws` folder,
```consol
$ catkin_make
```

A ROS-noetic package to control Robotiq 2F gripper and UR10 (CB series)


## 2. Running environments
- UR10 CB-series ([Installation guide](https://github.com/s-nam/UniversalRobots/blob/main/Installation_guides/01_UR10_on_ROS-noetic/README.md))
- Robotiq 2F-85 or 2F-140 Gripper ([Installation guide](https://github.com/s-nam/UniversalRobots/blob/main/Installation_guides/02_Robotiq_2F_gripper/README.md))
- ROS-noetic
- Ubuntu 20.04.5 LTS 64-bit

## 3. Running a Python code
Open 6 windows in Terminal, and execute the following commands in turn.
```console
(Window#1)$ roscore
```
```console
(Window#2)$ rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0
```
```console
(Window#3)$ roslaunch ur_robot_driver ur10_bringup.launch
```
Go to the teach pandent of UR10, load "ExternalControl.urp" on the teach pandent, and click `play` button. Then, you should double-check the following meassage in the terminal, such as
```console
(Window#3)$ ...
...
[INFO] [1673442528.019679]: Started controllers: joint_state_controller, scaled_pos_joint_traj_controller, speed_scaling_state_controller, force_torque_sensor_controller
[ INFO] [1673442534.449626052]: Robot requested program
[ INFO] [1673442534.449684350]: Sent program to robot
[ INFO] [1673442534.723437911]: Robot connected to reverse interface. Ready to receive control commands.
```

```console
(Window#4)$ roslaunch ur10_moveit_config moveit_planning_execution.launch
```

```console
(Window#5)$ roslaunch ur10_moveit_config moveit_rviz.launch rviz_config:=$(rospack find ur10_moveit_config)/launch/moveit.rviz
```
Please make sure that RViz presents similar robot posture to that of the real UR10.

     Important!! Before running the following Python code, please double check the values of joint angles in the code.

```console
(Windows#6)$ rosrun control_2f_gripper_and_ur10 gripper_ur10_demo.py
```


## 4. Issue - Gripper is not working
Currently, please run the following code first and run the section 3 again.

```console
$ rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py
```

You can also check [the official guideline of this gripper](http://wiki.ros.org/robotiq/Tutorials/Control%20of%20a%202-Finger%20Gripper%20using%20the%20Modbus%20RTU%20protocol%20%28ros%20kinetic%20and%20newer%20releases%29).

