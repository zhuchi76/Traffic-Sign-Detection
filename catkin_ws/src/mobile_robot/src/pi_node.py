#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32MultiArray

# left_speed, right_speed, count_max
def move_forward():
    return [217, 200], 300
    # return [138, 125], 300

def move_backward():
    return [-50, -50], 100

def stop():
    return [0, 0], 200


def turn_left_90_degree():
    return [-50, 50], 200

def turn_right_90_degree():
    return [50, -50], 200

def main():
    # Publisher to send data to Arduino
    pub = rospy.Publisher('to_arduino', Int32MultiArray, queue_size=10)
    rospy.init_node('pi_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    rospy.sleep(5)

    counter = 0

    state = 0

    while not rospy.is_shutdown():
        right_speed_to_send = int(input("Enter speed for right motor: "))
        left_speed_to_send = int(input("Enter speed for left motor: "))
        to_send = Int32MultiArray()
        to_send.data = [right_speed_to_send, left_speed_to_send]
        pub.publish(to_send)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

