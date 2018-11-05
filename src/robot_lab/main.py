import sys
import json
import math

import rospy
from std_msgs.msg import String
from face_tracking.msg import Dist

from enum import Enum

Motor = Enum('Motor', 'Right Left')

def clamp_range(min_value, max_value, val):
    return max(min_value, min(max_value, val))

def generate_message(motor, speed):
    scaling_constant = 200.0
    clamped_speed = clamp_range(-1.0, 1.0, speed)
    regularized_speed = abs(clamped_speed * scaling_constant)
    direction = 'clockwise' if (motor == Motor.Right) == (speed >= 0.0) else 'counter-clockwise'
    message = {
        'command': 'run',
        'parameters': {
            'direction': direction,
            'speed': regularized_speed
            }
        }
    return json.dumps(message)

def main():
    motors = {
        Motor.Right: rospy.Publisher('/motor/right', String, queue_size=1),
        Motor.Left: rospy.Publisher('/motor/left', String, queue_size=1)
    }

    def drive(motor, speed):
        message = generate_message(motor, speed)
        motors[motor].publish(message)

    def soft_stop():
        stop_message = json.dumps({ 'command': 'stop', 'parameters': { 'type': 'soft' }})
        motors[Motor.Right].publish(stop_message)
        motors[Motor.Left].publish(stop_message)

    def callback(data):
        if data.dist == 0.0:
            soft_stop()
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

    rospy.init_node('main_control')
    rospy.Subscriber("/dist", Dist, callback)
    rospy.spin()

#    rate = rospy.Rate(10)
#    while not rospy.is_shutdown():
#        drive(Motor.Left, 1.0)
#        drive(Motor.Right, 1.0)
#        rate.sleep()

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
