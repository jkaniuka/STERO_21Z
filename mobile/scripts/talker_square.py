#!/usr/bin/env python

import rospy
from math import pi
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Duration

import time

length = 1

# linear_velocity = 0.04
# angular_velocity = 0.11

linear_velocity = 0.04 * 2.4
angular_velocity = 0.11 * 2.4

# linear_velocity = 0.04 * 2.4 * 2.4
# angular_velocity = 0.11 * 2.4 * 2.4

direction = 0 # 0 - left, 1- right

linear_time = rospy.Duration(length/linear_velocity)
angular_time = rospy.Duration(pi/2/angular_velocity)


def talker():
    pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    move_cmd = Twist()
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    pub.publish(move_cmd)
    step=1
    start = rospy.Time.now()
    

    while not rospy.is_shutdown():
        print("pi", angular_time)
        # print("time", time.time())

        if(step==1):
            print("step1")
            move_cmd.linear.x = linear_velocity
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            if(start + linear_time <= rospy.Time.now()):
                print("end_step_1")
                step=2
                start = rospy.Time.now()
        if(step==2):
            print("step2")
            move_cmd.linear.x = 0.0
            if(direction == 0):
                move_cmd.angular.z = angular_velocity 
            if(direction == 1):
                move_cmd.angular.z = -angular_velocity 
            pub.publish(move_cmd)
            if(start + angular_time <= rospy.Time.now()):
                print("end_step_2")
                step=1
                start = rospy.Time.now()

        rate.sleep()


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
