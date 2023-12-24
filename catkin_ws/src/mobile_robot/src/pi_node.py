#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Int32MultiArray

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

distance_cm = 0

def callback(data):
   global distance_cm
   distance_cm = data.data


def main():
    # Publisher to send data to Arduino
    pub = rospy.Publisher('to_arduino', Int32MultiArray, queue_size=10)
    rospy.init_node('pi_node', anonymous=True)
    # Subscriber to receive data from Arduino
    rospy.Subscriber("from_arduino", Int32, callback)
    rate = rospy.Rate(10)  # 10hz
    rospy.sleep(5)

    to_send = Int32MultiArray()
    to_send.data = [0, 0]

    counter = 0

    state = 1
    

    while not rospy.is_shutdown():
        # right_speed_to_send = int(input("Enter speed for right motor: "))
        # left_speed_to_send = int(input("Enter speed for left motor: "))
        # to_send = Int32MultiArray()
        # to_send.data = [right_speed_to_send, left_speed_to_send]
        # pub.publish(to_send)
        
        if state == 1: # Exploration forward
            # Move forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print 'move_forward'
            pub.publish(to_send)

            # Next state
            if distance_cm < 50:
                counter = 0
                state = 4
            elif counter > count_max:
                counter = 0
            else:
                state = 1

                
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

