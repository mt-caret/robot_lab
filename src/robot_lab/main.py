import sys
import json

import rospy
from std_msgs.msg import String

def main():
    motor_right = rospy.Publisher('/motor/right', String, queue_size=1)
    motor_left = rospy.Publisher('/motor/left', String, queue_size=1)

    rospy.init_node('main_control')
     rate = rospy.Rate(10)

    move_clockwise_message = json.dumps(
            { 'command': 'run'
            , 'parameters':
                { 'direction': 'clockwise'
                , 'speed': 100.0
                }
            })
    move_counterclockwise_message = json.dumps(
            { 'command': 'run'
            , 'parameters':
                { 'direction': 'counter-clockwise'
                , 'speed': 100.0
                }
            })

    while not rospy.is_shutdown():
        motor_left.publish(move_clockwise_message)
        motor_right.publish(move_counterclockwise_message)
        rate.sleep()

if __name__ == '__main__':
    sys.exit("main.py should not be called directly! use roslaunch instead.")
