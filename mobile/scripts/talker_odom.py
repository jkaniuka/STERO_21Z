#!/usr/bin/env python
import rospy
from math import pi,sqrt
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf

length = 1



# linear_velocity = 0.04
# angular_velocity = 0.11

linear_velocity = 0.04 * 2.4
angular_velocity = 0.11 * 2.4

# linear_velocity = 0.04 * 2.4 * 2.4
# angular_velocity = 0.11 * 2.4 * 2.4

direction = 0 # 0 - left, 1- right

linear_time = length/linear_velocity
angular_time = pi/2/angular_velocity


global theta
global x 
global y
global start_theta
global start_x
global start_y 
global spin_once
spin_once = True

def convert_angle(angle):
    angle = angle * 180 / pi
    if(angle < 0):
        angle = angle + 360
    return angle

def callback(data):
    global spin_once
    global start_x
    global start_y
    global x
    global y
    global theta
    global start_theta
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = convert_angle(euler[2])
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    if (spin_once):
        start_x = data.pose.pose.position.x
        start_y = data.pose.pose.position.y
        start_theta = theta
        spin_once = False

def calculate_distance(start_x, start_y, x, y):
    return sqrt((start_x - x)**2 + (start_y - y)**2)

def talker():
    global spin_once
    global start_x
    global start_y
    global start_theta
    global x
    global y
    sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, callback, queue_size=1)
    pub = rospy.Publisher('/key_vel', Twist, queue_size=1)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(40) # 10hz
    move_cmd = Twist()
    move_cmd.linear.x = 0.0
    move_cmd.angular.z = 0.0
    step=1

    pub.publish(move_cmd)

    if(direction == 0):
        start_theta = start_theta - 90
    if(direction == 1):
        start_theta = start_theta + 90

    while not rospy.is_shutdown():

        if(step==1):
            move_cmd.linear.x = linear_velocity
            move_cmd.angular.z = 0.0

            if(calculate_distance(start_x, start_y, x, y) >= length):
                move_cmd.linear.x = 0.0
                if(direction == 0):
                    start_theta = start_theta + 90
                    move_cmd.angular.z = angular_velocity 
                if(direction == 1):
                    start_theta = start_theta - 90
                    move_cmd.angular.z = -angular_velocity 
                step=2
                if(start_theta > 360):
                    start_theta = start_theta - 360
                if(start_theta < 0):
                    start_theta = start_theta + 360
            pub.publish(move_cmd)
            rate.sleep()
        if(step==2):
            move_cmd.linear.x = 0.0
            if(direction == 0):
                actual_difference = theta - start_theta
                move_cmd.angular.z = angular_velocity 
            if(direction == 1):
                move_cmd.angular.z = -angular_velocity 
                actual_difference = start_theta - theta
            if(actual_difference < 0):
                actual_difference = actual_difference + 360
            if(actual_difference >= 90 and actual_difference < 300):
                print(actual_difference)
                step=1
                move_cmd.linear.x = linear_velocity
                move_cmd.angular.z = 0.0
                start_x = x
                start_y = y
            pub.publish(move_cmd)
            rate.sleep()

        if(step==3):

            if(direction == 0):
                start_theta = start_theta + 90
                move_cmd.angular.z = angular_velocity 
            if(direction == 1):
                start_theta = start_theta - 90
                move_cmd.angular.z = -angular_velocity 
            step=2
            if(start_theta > 360):
                start_theta = start_theta - 360
            if(start_theta < 0):
                start_theta = start_theta + 360
        print(start_theta, theta, step, move_cmd.angular.z)
        rate.sleep()
               
if __name__ == '__main__':
    global theta
    global x 
    global y
    global start_x
    global start_y 
    global spin_once
    global start_theta
    spin_once = True

    try:
        talker()
    except rospy.ROSInterruptException:
        pass
