import sys
import json

import rospy
from std_msgs.msg import String

from enum import Enum

Motor = Enum('Motor', 'Right Left')

def generate_message(motor, speed):
    scaling_constant = 100.0
    clamped_speed = max(-1.0, min(1.0, speed))
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

    rospy.init_node('main_control')
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        drive(Motor.Left, 1.0)
        drive(Motor.Right, 1.0)
        rate.sleep()

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
