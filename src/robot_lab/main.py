# -*- coding: utf-8 -*- 
import sys
import json
import math

import rospy
from std_msgs.msg import String
from face_tracking.msg import Dist
from sensor_msgs.msg import Joy
from robot_lab.msg import MotorCommand

from enum import Enum

import MeCab

motor = rospy.Publisher('/motor', MotorCommand, queue_size=1)

def clamp_range(min_value, max_value, val):
    return max(min_value, min(max_value, val))

MAX_MOTOR_SPEED = 3000.0

def drive(right_speed, left_speed):
    right_speed = clamp_range(-1.0, 1.0, right_speed) * MAX_MOTOR_SPEED
    left_speed = clamp_range(-1.0, 1.0, left_speed) * MAX_MOTOR_SPEED
    motor.publish(MotorCommand(right=right_speed, left=left_speed, hardStop=False))

Stop = Enum('Stop', 'Soft Hard')

def stop(stop_type):
    motor.publish(MotorCommand(left=0.0, right=0.0, hardStop=stop_type==Stop.Hard))

def align_diff(distance_diff, angle_diff):
    distance_weight = 0.8
    angle_weight = 1.0

    v_right = distance_weight * distance_diff + angle_weight * angle_diff
    v_left  = distance_weight * distance_diff - angle_weight * angle_diff
    rospy.loginfo("distance_diff: %f, angle_diff: %f, v_right: %f, v_left: %f", distance_diff, angle_diff, v_right, v_left)

    drive(right_speed=v_right, left_speed=v_left)

def face_chaser():
    def callback(data):
        if data.dist == 0.0:
            stop(Stop.Soft)
            return
        target_distance = 70.0
        target_angle = 0.0

        distance_diff = (data.dist - target_distance) / 100
        angle_diff = (data.angle - target_angle) / math.pi


        align_diff(distance_diff, angle_diff)

    rospy.Subscriber("/dist", Dist, callback)
    rospy.spin()

def turn(clockwise):
    if clockwise:
        drive(right_speed=-0.5, left_speed=0.5)
    else:
        drive(right_speed=0.5, left_speed=-0.5)

def ps3_controller():
    def callback(data):
        l1 = data.buttons[10] == 1
        r1 = data.buttons[11] == 1
        x = clamp_range(-1.0, 1.0, -data.axes[0])
        y = clamp_range(-1.0, 1.0, data.axes[1])
        if y != 0.0:
            distance_diff = y * 10
            angle_diff = math.atan(x/y)
            align_diff(distance_diff, angle_diff)
        elif l1:
            turn(clockwise=False)
        elif r1:
            turn(clockwise=True)
        else:
            stop(Stop.Soft)

    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

def voice_controller():
    def parse(text):
        m = MeCab.Tagger("-Ochasen")
        node = m.parseToNode(text)
        words_list = []
        while node:
            word = node.surface
            wclass = node.feature.split(',')
            if wclass[0] != 'BOS/EOS':
                words_list.append(word)
            node = node.next
        return words_list

    def callback(msg):
        words = parse(msg.data)
        for word in words:
            if word == "止まれ":
                stop(Stop.Soft)
                return
            elif word == "前":
                drive(Motor.Right, 0.5)
                drive(Motor.Left, 0.5)
                return
            elif word == "後ろ":
                drive(Motor.Right, -0.5)
                drive(Motor.Left, 0.5)
                return
            elif word == "右":
                turn(True)
                return
            elif word == "左":
                turn(False)
                return
    rospy.Subscriber("/speech", String, callback) 
    rospy.spin()           

def main():
    rospy.init_node('main_control')
    ps3_controller()
    # face_chaser()
    # voice_controller()

#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        drive(Motor.Left, 1.0)
#        drive(Motor.Right, 1.0)
#        rate.sleep()

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
