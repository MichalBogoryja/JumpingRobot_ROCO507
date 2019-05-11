#!/usr/bin/env python

import rospy
import sys, termios, tty, os, time
from std_msgs.msg import Int32, Float64

#declaration of global variables
motor_fr_speed = 0.0
motor_fl_speed = 0.0
motor_br_speed = 0.0
motor_bl_speed = 0.0

button_delay = 0.2


#3rd party function for getting keyboard commands
def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == '__main__':

    rospy.init_node('robot_controller', anonymous=True)

    rate = rospy.Rate(10)  # 10hz

    #declaration of ros publisher and assigning topics and data to be messaged
    speed_fr_publisher = rospy.Publisher('wheel_front_right_controller/command', Float64, queue_size=10)
    speed_fl_publisher = rospy.Publisher('wheel_front_left_controller/command', Float64, queue_size=10)
    speed_br_publisher = rospy.Publisher('wheel_back_right_controller/command', Float64, queue_size=10)
    speed_bl_publisher = rospy.Publisher('wheel_back_left_controller/command', Float64, queue_size=10)

    while not rospy.is_shutdown():
        char = getch()

        if char == "p":     #closes the program
            print("Stop program!")
            exit(0)

        if char == "w":     #setting the robot's speed to maximum and direction as forward
            print("Forward")
            motor_fr_speed = -7.0
            motor_fl_speed = 7.0
            motor_br_speed = -7.0
            motor_bl_speed = 7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "s":     #setting the robot's speed to maximum and direction as backward
            print("Backward")
            motor_fr_speed = 7.0
            motor_fl_speed = -7.0
            motor_br_speed = 7.0
            motor_bl_speed = -7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "t":     #stop
            print("Stop")
            motor_fr_speed = 0.0
            motor_fl_speed = 0.0
            motor_br_speed = 0.0
            motor_bl_speed = 0.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "a":     #setting the robot's speed to maximum and direction as left
            print("left")
            motor_fr_speed = -7.0
            motor_fl_speed = -7.0
            motor_br_speed = 7.0
            motor_bl_speed = 7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "d":     #setting the robot's speed to maximum and direction as right
            print("Right")
            motor_fr_speed = 7.0
            motor_fl_speed = 7.0
            motor_br_speed = -7.0
            motor_bl_speed = -7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "e":     #setting the robot's speed to maximum and direction as diagonal forward right
            print("Diagonal forward right")
            motor_fr_speed = 0.0
            motor_fl_speed = 7.0
            motor_br_speed = -7.0
            motor_bl_speed = 0.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "q":     #setting the robot's speed to maximum and direction as diagonal forward left
            print("Diagonal forward left")
            motor_fr_speed = -7.0
            motor_fl_speed = 0.0
            motor_br_speed = 0.0
            motor_bl_speed = 7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "z":     #setting the robot's speed to maximum and direction as diagonal backward left
            print("Diagonal backward left")
            motor_fr_speed = 0.0
            motor_fl_speed = -7.0
            motor_br_speed = 7.0
            motor_bl_speed = 0.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "c":     #setting the robot's speed to maximum and direction as diagonal backward right
            print("Diagonal backward right")
            motor_fr_speed = 7.0
            motor_fl_speed = 0.0
            motor_br_speed = 0.0
            motor_bl_speed = -7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "x":     #setting the robot's speed to maximum and rotate anti-clockwise
            print("Rotate anti-clockwise")
            motor_fr_speed = -7.0
            motor_fl_speed = -7.0
            motor_br_speed = -7.0
            motor_bl_speed = -7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "v":     #setting the robot's speed to maximum and rotate clockwise
            print("Rotate clockwise")
            motor_fr_speed = 7.0
            motor_fl_speed = 7.0
            motor_br_speed = 7.0
            motor_bl_speed = 7.0
            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "t":     #increase current speed by 7% of its maximum value
            print("Speed up")
            if 0.0 < motor_fr_speed < 7.0:
                motor_fr_speed += 0.5
            if 0.0 < motor_fl_speed < 7.0:
                motor_fl_speed += 0.5
            if 0.0 < motor_br_speed < 7.0:
                motor_br_speed += 0.5
            if 0.0 < motor_bl_speed < 7.0:
                motor_bl_speed += 0.5

            if -7.0 < motor_fr_speed < 0.0:
                motor_fr_speed -= 0.5
            if -7.0 < motor_fl_speed < 0.0:
                motor_fl_speed -= 0.5
            if -7.0 < motor_br_speed < 0.0:
                motor_br_speed -= 0.5
            if -7.0 < motor_bl_speed < 0.0:
                motor_bl_speed -= 0.5

            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "g":  # decrease current speed by 7% of its maximum value
            print("Speed down")
            if 0.0 < motor_fr_speed < 7.0:
                motor_fr_speed -= 0.5
            if 0.0 < motor_fl_speed < 7.0:
                motor_fl_speed -= 0.5
            if 0.0 < motor_br_speed < 7.0:
                motor_br_speed -= 0.5
            if 0.0 < motor_bl_speed < 7.0:
                motor_bl_speed -= 0.5

            if -7.0 < motor_fr_speed < 0.0:
                motor_fr_speed += 0.5
            if -7.0 < motor_fl_speed < 0.0:
                motor_fl_speed += 0.5
            if -7.0 < motor_br_speed < 0.0:
                motor_br_speed += 0.5
            if -7.0 < motor_bl_speed < 0.0:
                motor_bl_speed += 0.5

            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)
