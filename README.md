# JumpingRobot_ROCO507

[TOC]

## Introduction

The aim of this project is to build a robot with four jumping legs (which can fold) and four mecanum wheels, operated by a human via the Robot Operating System (ROS).

The system is distributed across:

 - Raspberry Pi - which main task is to steer the robot's motors.

 - Laptop with Linux operating system - enables human's interaction.



## Features

The robot can:

- jump,
- fold its legs,
- drive in multiple directions.

The robot's behavior can be controlled by a human operating the "robot.py" node on the laptop.

## Installation

Laptop with Linux: 

1. Install ROS kinetic - follow steps from: http://wiki.ros.org/kinetic/Installation/Ubuntu

2. Create a catkin workspace - follow steps from: 

   1. http://wiki.ros.org/catkin/Tutorials/create_a_workspace
   2. http://wiki.ros.org/catkin/Tutorials/CreatingPackage <instead of calling package "beginner_tutorials" call it "my_dynamixel">

3. Upload files to my_dynamixel package:

   1. upload the file "robot.py" to: ~/catkin_ws/src/my_dynamixel/

4. Build packages in catkin workspace

   1. Go to catkin workspace directory

      ```bash
      $ cd ~/catkin_ws/
      ```

   2. Build packages

      ```bash
      $ catkin_make
      ```

5. Configure .bashrc file 

   1. Open .bashrc file

      ```bash
      $ gedit .bashrc
      ```

   2. Add at the bottom of the file following lines:

      ```
      export ROS_MASTER_URI=http://localhost:11311/
      export ROS_HOSTNAME=<the laptop's IP>
      export ROS_IP=<the laptop's IP>
      ```

   3. Source .bashrc file

      ```bash
      $ source .bashrc
      ```


Raspberry Pi:

1. Follow steps 1 and 2 from the laptop's instruction <instead of calling package "my_dynamixel" call it "my_dynamixel_tutorial">

2. Install Dynamixel Controllers driver

   ```bash
   $ sudo apt-get install ros-<distro>-dynamixel-controllers
   ```

3. Upload files to my_dynamixel_tutorial package:

   1. upload content of the "launch" folder to: ~/catkin_ws/src/my_dynamixel_tutorial/launch/
   2. upload content of the "src" folder to: ~/catkin_ws/src/my_dynamixel_tutorial/

4. Build packages in catkin workspace

   1. Go to catkin workspace directory

      ```bash
      $ cd ~/catkin_ws/
      ```

   2. Build packages

      ```bash
      $ catkin_make
      ```

5. Configure .bashrc file 

   1. Open .bashrc file

      ```bash
      $ gedit .bashrc
      ```

   2. Add at the bottom of the file following lines:

      ```
      export ROS_MASTER_URI=http://the laptop's IP:11311/
      export ROS_HOSTNAME=<the Raspberry's IP>
      export ROS_IP=<the Raspberry's IP>
      ```

   3. Source .bashrc file

      ```bash
      $ source .bashrc
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
- Y - increase current speed by 7% of its maximum value
- H - reduce current speed by 7% of its maximum value
- O - setting the robot's leg to 0 position
- K - setting the robot's legs to the first starting position
- N - setting the robot's legs to the second starting position
- J - changing the robot's legs position by -0.05 (rad)
- L - changing the robot's legs position by +0.05 (rad)
- M - jumping (from the first starting position)
- O - jumping (from the second starting position)
- 0 - rotating legs up
- 1 - rotating legs down by 0.1 (rad)
- 2 - rotating legs up by 0.1 (rad)
- 3 - rotating legs down 
- 5 - preparing legs to down-folding (when initial position is ~0)
- 5 - preparing legs to down-folding (when initial position is ~5.2)

## Links to videos

Evaluation of the robot's driving behavior:

1. Early stages - <https://youtu.be/BNNEmNGqUxc>
2. Final stage - <https://youtu.be/ybAbjyJAmC4>

Evaluation of the robot's jumping behavior:

1. Just one leg - https://youtu.be/G1J8bCmxgGU
3. Early stages with all four legs - https://youtu.be/6cJKMZFHYcE
4. Four legs after some improvements - https://youtu.be/etr4STEtW9g
5. Final stage - https://youtu.be/HQ1hYwN6pAw

Height of a jump experiment - <https://youtu.be/KC6PH26aIuQ>

Legs folding - https://youtu.be/vKZoalooXHU

The robot's behavior animation - https://youtu.be/MrXAhyIWOSs

Final demonstration of all behaviors - https://youtu.be/TGW2EMyFp7g

## Authors

Michal Bogoryja-Zakrzewski 

Tengxiao He 

Haiwei Xu 