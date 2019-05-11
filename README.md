# JumpingRobot_ROCO507# Jumping robot

[TOC]

## Introduction

The aim of this project is to build a robot with four jumping legs (which can fold) and four mecanum wheels, operated by a human via the Robot Operating System (ROS).

The system is distributed across:

 - Raspberry Pi - which main task is to steer the robot's motors.

 - Laptop with Linux operating system - enables human's interaction.



## Features

The robot can:

- jump,

- drive in multiple directions.

## Installation

Laptop with Linux: 

1. Install ROS kinetic - follow steps from: http://wiki.ros.org/kinetic/Installation/Ubuntu

2. Create a catkin workspace - follow steps from: 

   1. http://wiki.ros.org/catkin/Tutorials/create_a_workspace
   2. http://wiki.ros.org/catkin/Tutorials/CreatingPackage <instead of calling package "beginner_tutorials" call it "my_dynamixel">

3. Upload files to my_dynamixel package:

   1. upload the file "robot.py" to: ~/catkin_ws/src/my_dynamixel/scripts

   2. ```bash
      $ cd ~/catkin_ws/
      ```

   3. ```bash
      $ catkin_make
      ```


Raspberry Pi:

1. Follow steps 1 and 2 from the laptop's instruction <instead of calling package "my_dynamixel" call it "my_dynamixel_tutorial">

2. Upload files to my_dynamixel_tutorial package:

   1. upload content of the "launch" folder to: ~/catkin_ws/src/my_dynamixel_tutorial/launch/

   2. upload content of the "src" folder to: ~/catkin_ws/src/my_dynamixel_tutorial/

   3. ```bash
      $ cd ~/catkin_ws/
      ```

   4. ```bash
      $ catkin_make
      ```

## Running the system

In order to run the system, following steps are to be executed:

1. Run roscore in a terminal on the Linux laptop

   ```bash
   $ roscore
   ```

2. Connect the Raspberry Pi to a power

3. Connect to the Raspberry Pi from the second terminal on the laptop and run ROS nodes on the former

   ```bash
   $ ssh pi@<Raspberry_IP>
   ```

   ```bash
   $ roslaunch my_dynamixel_tutorial controller_manager.launch
   ```

4. Connect to the Raspberry Pi from the third terminal on the laptop and run ROS nodes on the former

   ```bash
   $ ssh pi@<Raspberry_IP>
   ```

   ```bash
   $ roslaunch my_dynamixel_tutorial start_pan_controller.launch
   ```

   ```bash
   $ roslaunch my_dynamixel_tutorial start_tilt_controller.launch
   ```

5. To enable human-control over the robot, run the following node in the fourth terminal on the laptop

   ```bash
   $ rosrun my_dynamixel robot.py
   ```


## Human-control commands

Keyboard commands (to be pressed in the robot.py node window):

- W - full speed forward
- S - full speed backwards
- T - stop
- A - full speed left
- D - full speed right
- Q - full speed diagonal forward left
- E - full speed diagonal forward right
- Z - full speed diagonal backward left
- C - full speed diagonal backward right
- X - rotate at a full speed anti-clockwise
- V - rotate at a full speed clockwise
- T - increase current speed by 7% of its maximum value
- G - reduce current speed by 6% of its maximum value
- 

## Authors

Michal Bogoryja-Zakrzewski - 10634856

Tengxiao He - 10609178

Haiwei Xu - 10623214
