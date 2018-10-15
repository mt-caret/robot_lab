import sys
import json

import rospy
from std_msgs.msg import String

def main():
    motor_right = rospy.Publisher('/motor/right', String)
    motor_left = rospy.Publisher('/motor/right', String)

    move_clockwise_message = json.dumps(
            { 'command': 'move'
            , 'parameters':
                { 'direction': 'clockwise'
                , 'speed': 300.0
                }
            })
    motor_right.publish(move_clockwise_message)

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
