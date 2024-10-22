#!/usr/bin/env python

import rospy
import sys, termios, tty, os, time
from std_msgs.msg import Int32, Float64

#declaration of global variables
motor_fr_speed = 0.0
motor_fl_speed = 0.0
motor_br_speed = 0.0
motor_bl_speed = 0.0

position = 0.0
position_bl = 0.0
position_fr = 0.0
position_fl = 0.0
position_br = 0.0

folding_fl_position = 0.0
folding_bl_position = 0.0
folding_fr_position = 0.0
folding_br_position = 0.0

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

    position_bl_publisher = rospy.Publisher('leg_back_left_controller/command', Float64, queue_size=10)
    position_fr_publisher = rospy.Publisher('leg_front_right_controller/command', Float64, queue_size=10)
    position_fl_publisher = rospy.Publisher('leg_front_left_controller/command', Float64, queue_size=10)
    position_br_publisher = rospy.Publisher('leg_back_right_controller/command', Float64, queue_size=10)

    folding_fl_publisher = rospy.Publisher('front_left_legs_folding_controller/command', Float64, queue_size=10)
    folding_bl_publisher = rospy.Publisher('back_left_legs_folding_controller/command', Float64, queue_size=10)
    folding_fr_publisher = rospy.Publisher('front_right_legs_folding_controller/command', Float64, queue_size=10)
    folding_br_publisher = rospy.Publisher('back_right_legs_folding_controller/command', Float64, queue_size=10)

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

        if char == "y":     #increase current speed by 7% of its maximum value
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

        if char == "h":  # decrease current speed by 7% of its maximum value
            print("Speed down")
            if 0.0 < motor_fr_speed <= 7.0:
                motor_fr_speed -= 0.5
            if 0.0 < motor_fl_speed <= 7.0:
                motor_fl_speed -= 0.5
            if 0.0 < motor_br_speed <= 7.0:
                motor_br_speed -= 0.5
            if 0.0 < motor_bl_speed <= 7.0:
                motor_bl_speed -= 0.5

            if -7.0 <= motor_fr_speed < 0.0:
                motor_fr_speed += 0.5
            if -7.0 <= motor_fl_speed < 0.0:
                motor_fl_speed += 0.5
            if -7.0 <= motor_br_speed < 0.0:
                motor_br_speed += 0.5
            if -7.0 <= motor_bl_speed < 0.0:
                motor_bl_speed += 0.5

            speed_fr_publisher.publish(motor_fr_speed)
            speed_fl_publisher.publish(motor_fl_speed)
            speed_br_publisher.publish(motor_br_speed)
            speed_bl_publisher.publish(motor_bl_speed)
            time.sleep(button_delay)

        if char == "o":  # setting the robot's legs to 0 position (Jump if before in starting position ~4.9)
            position_bl = 0.0
            position_fr = 0.0
            position_fl = 0.0
            position_br = 0.0
            position = 0.0
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "k":  # setting the robot's legs to the starting position ~0
            print("Starting position ~0")
            position_bl = 0.04
            position_fr = 0.05
            position_fl = 0.12
            position_br = 0.05
            position = 0.0
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "l":  # changing the robot's legs position by 0.05
            print("Position +0.05")
            position_bl += 0.05
            position_fr += 0.05
            position_fl += 0.05
            position_br += 0.05
            position += 0.05
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "j":  # changing the robot's legs position by -0.05
            print("Position -0.05")
            position_bl -= 0.05
            position_fr -= 0.05
            position_fl -= 0.05
            position_br -= 0.05
            position -= 0.05
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "m":  # setting the robot's legs to 5.2 position (Jump if before in starting position ~0)
            print("Position 5.2")
            position_bl = 5.2
            position_fr = 5.2
            position_fl = 5.2
            position_br = 5.2
            position = 5.2
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "n":  # setting the robot's legs to the jumping position ~4.9
            print("Jump2")
            position_bl = 4.85
            position_fr = 4.9
            position_fl = 4.9
            position_br = 4.935
            position = 4.9
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "5":  # setting the robot's legs to folding (down) position if legs in position 0
            print("Folding preparation")
            position_bl = 4
            position_fr = 4
            position_fl = 4
            position_br = 4
            position = 0.0
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "6":  # setting the robot's legs to folding (down) position if legs in position 5.2
            print("Folding preparation")
            position_bl = 1
            position_fr = 1
            position_fl = 1
            position_br = 1
            position = 0.0
            print(position)
            position_bl_publisher.publish(position_bl)
            position_fr_publisher.publish(position_fr)
            position_fl_publisher.publish(position_fl)
            position_br_publisher.publish(position_br)
            time.sleep(button_delay)

        if char == "0":  # setting the robot's legs up
            print("legs up")
            folding_bl_position = 0
            folding_fl_position = 5.2
            folding_br_position = 5.2
            folding_fr_position = 0
            print(folding_bl_position)
            folding_bl_publisher.publish(folding_bl_position)
            folding_fl_publisher.publish(folding_fl_position)
            folding_br_publisher.publish(folding_br_position)
            folding_fr_publisher.publish(folding_fr_position)
            time.sleep(button_delay)

        if char == "1":  # changing the robot's legs folding position down
            print("Legs down by 0.1")
            folding_bl_position += 0.1
            folding_fl_position -= 0.1
            folding_br_position -= 0.1
            folding_fr_position += 0.1
            print(folding_bl_position)
            folding_bl_publisher.publish(folding_bl_position)
            folding_fl_publisher.publish(folding_fl_position)
            folding_br_publisher.publish(folding_br_position)
            folding_fr_publisher.publish(folding_fr_position)
            time.sleep(button_delay)

        if char == "2":  # changing the robot's legs folding position up
            print("Legs up by 0.1")
            folding_bl_position -= 0.1
            folding_fl_position += 0.1
            folding_br_position += 0.1
            folding_fr_position -= 0.1
            print(folding_bl_position)
            folding_bl_publisher.publish(folding_bl_position)
            folding_fl_publisher.publish(folding_fl_position)
            folding_br_publisher.publish(folding_br_position)
            folding_fr_publisher.publish(folding_fr_position)
            time.sleep(button_delay)

        if char == "3":  # setting the robot's legs down
            print("Legs down")
            folding_bl_position = 3.14
            folding_fl_position = 2.06
            folding_br_position = 2.06
            folding_fr_position = 3.14
            print(folding_bl_position)
            folding_bl_publisher.publish(folding_bl_position)
            folding_fl_publisher.publish(folding_fl_position)
            folding_br_publisher.publish(folding_br_position)
            folding_fr_publisher.publish(folding_fr_position)
            time.sleep(button_delay)
