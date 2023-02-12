#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy
from math import pi,sqrt
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from std_msgs.msg import Duration
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
import tf

import time



length = 2

# linear_velocity = 0.25
# angular_velocity = 0.11

# linear_velocity = 0.04
# angular_velocity = 0.11

# linear_velocity = 0.04 * 2.4
# angular_velocity = 0.11 * 2.4

linear_velocity = 0.04 * 2.4 * 2.4
angular_velocity = 0.11 * 2.4 * 2.4

direction = 0 # 0 - left, 1- right

linear_time = length/linear_velocity
angular_time = pi/2/angular_velocity

global theta
global x 
global y
linear_precision = 0.1
angle_precision = 0.01 * pi

global start_x
global start_y 
global spin_once
spin_once = True



def callback(data):

    if (spin_once):
        start_x = data.pose.pose.position.x
        start_y = data.pose.pose.position.y
        spin_once = False
    #rospy.loginfo(rospy.get_caller_id() + 'I heard %s', data.pose.pose)
    quaternion = (
    data.pose.pose.orientation.x,
    data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,
    data.pose.pose.orientation.w)
    euler = tf.transformations.euler_from_quaternion(quaternion)
    theta = euler[2]
    x = data.pose.pose.position.x
    y = data.pose.pose.position.y
    print("x:", x, "y:", y ,"Theta: ", theta)

def calculate_distance(start_x, start_y, x, y):
    return sqrt((start_x - x)**2 + (start_y - y)**2)




def talker():
    sub = rospy.Subscriber('/mobile_base_controller/odom', Odometry, callback)
    pub = rospy.Publisher('/key_vel', Twist, queue_size=10)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(10) # 10hz
    move_cmd = Twist()
    #move_cmd.linear.x = 1.0
    move_cmd.angular.z = 1.0
    step=1
    start = time.time()


    

    while not rospy.is_shutdown():
        #print("pi", angular_time)
        # print("time", time.time())


        if(step==1):
            move_cmd.linear.x = linear_velocity
            move_cmd.angular.z = 0.0
            pub.publish(move_cmd)
            if(calculate_distance(start_x, start_y, x, y) >= length):
                step=2
                start_theta = theta

        if(step==2):
            move_cmd.linear.x = 0.0
            if(direction == 0):
                move_cmd.angular.z = angular_velocity 
            if(direction == 1):
                move_cmd.angular.z = -angular_velocity 
            pub.publish(move_cmd)
            if(start + angular_time <= time.time()):
                step=1






        
        rate.sleep()

               
        
        

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
