import sys
import json

import rospy
from std_msgs.msg import String

def main():
    motor_right = rospy.Publisher('/motor/right', String)
    motor_left = rospy.Publisher('/motor/left', String)

    rospy.init_node('main_control')

    move_clockwise_message = json.dumps(
            { 'command': 'run'
            , 'parameters':
                { 'direction': 'clockwise'
                , 'speed': 100.0
                }
            })
    motor_right.publish(move_clockwise_message)

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
