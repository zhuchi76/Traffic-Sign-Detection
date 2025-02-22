#!/usr/bin/env python3

import rospy
from std_msgs.msg import Int32, Int32MultiArray, Bool, String

# left_speed, right_speed, count_max
def move_forward():
    # return [217, 200], 300
    return [138, 125], 300

def move_backward():
    return [-50, -50], 100

def stop():
    return [0, 0], 300

def turn_left_90_degree():
    return [-50, 50], 100

def turn_right_90_degree():
    return [50, -50], 100

def turn_left_180_degree():
    return [-50, 50], 200


distance_cm = 0
traffic_sign = 'WAITING FOR DETECT'

def callback_arduino(data):
   global distance_cm
   distance_cm = data.data

def callback_detection(data):
   global traffic_sign
   traffic_sign = data.data


def main():
    global distance_cm
    global traffic_sign
    rospy.init_node('pi_node', anonymous=True)
    
    # Arduino
    pub_to_arduino = rospy.Publisher('to_arduino', Int32MultiArray, queue_size=10)
    rospy.Subscriber("from_arduino", Int32, callback_arduino)

    # detection.py
    pub_to_detection = rospy.Publisher('start_detection', Bool, queue_size=10)
    rospy.Subscriber('traffic_sign', String, callback_detection)
    
    
    rate = rospy.Rate(100)  # 10hz
    rospy.sleep(5)

    to_send = Int32MultiArray()
    to_send.data = [0, 0]

    start_detection_msg = Bool()
    start_detection_msg.data = False

    counter = 0
    state = 1

    traffic_sign_temp = 'WAITING FOR DETECT'
    

    while not rospy.is_shutdown():

        if state != 2:
            start_detection_msg.data = False
            pub_to_detection.publish(start_detection_msg)
        
        if state == 0: # Stop for training
            # Stop
            to_send.data, count_max = stop()
            if counter % 100 == 0:
                print('stop')
            pub_to_arduino.publish(to_send)

            # Next state
            if counter > 1000:
                counter = 0
                state = 1
        
        elif state == 1: # Move forward to the wall
            # Move forward
            to_send.data, count_max = move_forward()
            if counter % 100 == 0:
                print('move_forward') 
            pub_to_arduino.publish(to_send)

            # Next state
            if distance_cm < 100:
                counter = 0
                state = 2
                


        elif state == 2: # Detect traffic sign on the wall
            start_detection_msg.data = True
            pub_to_detection.publish(start_detection_msg)
            # Stop
            to_send.data, count_max = stop()
            if counter % 100 == 0:
                print('stop') 
            pub_to_arduino.publish(to_send)
            
            # Next state
            if traffic_sign != 'WAITING FOR DETECT':
                counter = 0
                if traffic_sign == 'TURN LEFT':
                    state = 3
                elif traffic_sign == 'TURN RIGHT':
                    state = 4
                elif traffic_sign == 'ONE WAY':
                    state = 5
                elif traffic_sign == 'STOP':
                    state = 6
                else:
                    state = 2
                
        
        elif state == 3: # Turn left
            # Turn left
            to_send.data, count_max = turn_left_90_degree()
            if counter % 100 == 0:
                print('turn_left_90_degree') 
            pub_to_arduino.publish(to_send)
            
            # Next state
            if counter > count_max:
                counter = 0
                state = 1

        elif state == 4: # Turn right
            # Turn right
            to_send.data, count_max = turn_right_90_degree()
            if counter % 100 == 0:
                print('turn_right_90_degree') 
            pub_to_arduino.publish(to_send)
            
            # Next state
            if counter > count_max:
                counter = 0
                state = 1

        elif state == 5: # move_backward
            # move_backward
            #to_send.data, count_max = turn_left_180_degree()
            to_send.data, count_max = move_backward()
            if counter % 100 == 0:
                print('move_backward') 
            pub_to_arduino.publish(to_send)
            
            # Next state
            if counter > count_max:
                counter = 0
                state = 1

        elif state == 6: # Stop
            # Stop
            to_send.data, count_max = stop()
            if counter % 100 == 0:
                print('stop') 
            pub_to_arduino.publish(to_send)

            # Next state
            if counter > count_max:
                counter = 0
                state = 1
            # break

        counter += 1
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

