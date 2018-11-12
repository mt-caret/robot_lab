import sys
import json
import math

import rospy
from std_msgs.msg import String
from face_tracking.msg import Dist
from sensor_msgs.msg import Joy

from enum import Enum

Motor = Enum('Motor', 'Right Left')

motors = {
    Motor.Right: rospy.Publisher('/motor/right', String, queue_size=1),
    Motor.Left: rospy.Publisher('/motor/left', String, queue_size=1)
}

def clamp_range(min_value, max_value, val):
    return max(min_value, min(max_value, val))

MAX_MOTOR_SPEED = 200.0

def generate_message(motor, speed):
    clamped_speed = clamp_range(-1.0, 1.0, speed)
    regularized_speed = abs(clamped_speed * MAX_MOTOR_SPEED)
    direction = 'clockwise' if (motor == Motor.Right) == (speed >= 0.0) else 'counter-clockwise'
    message = {
        'command': 'run',
        'parameters': {
            'direction': direction,
            'speed': regularized_speed
            }
        }
    return json.dumps(message)

def drive(motor, speed):
    message = generate_message(motor, speed)
    motors[motor].publish(message)

Stop = Enum('Stop', 'Soft Hard')

def stop(stop_type):
    stop_type_str = 'soft' if stop_type == Stop.Soft else 'hard'
    stop_message = {
        'command': 'stop',
        'parameters': {
            'type': stop_type_str
        }
    }
    stop_message_json = json.dumps(stop_message)
    motors[Motor.Right].publish(stop_message_json)
    motors[Motor.Left].publish(stop_message_json)

def face_chaser():
    def callback(data):
        if data.dist == 0.0:
            stop(Stop.Soft)
            return
        target_distance = 70.0
        target_angle = 0.0

        distance_diff = (data.dist - target_distance) / 100
        angle_diff = (data.angle - target_angle) / math.pi

        distance_weight = 0.8
        angle_weight = 1.0

        v_right = distance_weight * distance_diff + angle_weight * angle_diff
        v_left  = distance_weight * distance_diff - angle_weight * angle_diff
        rospy.loginfo("distance_diff: %f, angle_diff: %f, v_right: %f, v_left: %f", distance_diff, angle_diff, v_right, v_left)

        drive(Motor.Right, v_right)
        drive(Motor.Left, v_left)

    rospy.Subscriber("/dist", Dist, callback)
    rospy.spin()

def turn(clockwise):
    if clockwise:
        drive(Motor.Right, -0.5)
        drive(Motor.Left, 0.5)
    else:
        drive(Motor.Right, 0.5)
        drive(Motor.Left, -0.5)

def ps3_controller():
    def callback(data):
        l1 = data.buttons[10] == 1
        r1 = data.buttons[11] == 1
        #left_x_axis = clamp_range(-1.0, 1.0, data.axes[0])
        if l1:
            turn(clockwise=false)
        elif l2:
            turn(clockwise=true)
        else:
            stop(Stop.Soft)

    rospy.Subscriber("/joy", Joy, callback)
    rospy.spin()

def main():
    rospy.init_node('main_control')
    ps3_controller()
    #face_chaser()

#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        drive(Motor.Left, 1.0)
#        drive(Motor.Right, 1.0)
#        rate.sleep()

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
