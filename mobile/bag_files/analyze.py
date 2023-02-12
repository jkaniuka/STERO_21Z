# coding=utf-8
import matplotlib.pyplot as plt
import csv
from math import cos, sin, atan2, asin, pi
import numpy as np


# File names:
odom_low = 'bag_files/odom_low.csv'
odom_med = 'bag_files/odom_med.csv'
odom_high = 'bag_files/odom_high.csv'
global_low = 'bag_files/global_low.csv'
global_med = 'bag_files/global_med.csv'
global_high = 'bag_files/global_high.csv'

#Arrays of postion values:

# GLOBAL:
x_pose_global = []
y_pose_global = []
theta_pose_global =[]

# ODOMETRY:
x_pose_odom = []
y_pose_odom = []
theta_pose_odom =[]

title = ''

def convert_angle(angle):
    angle = angle * 180 / pi
    if(angle < 0):
        angle = angle + 360
    if(angle > 360):
        angle = angle - 360
    return angle


def normalize_angle(angle):
  if angle > pi:
    angle = angle -  2 *pi
  if angle < -pi:
    angle = angle +  2 *pi
  return angle


def euler_from_quaternion(x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z # in radians


def show_tiago_path(file, x_array, y_array, theta_array, type):
    if "low" in file:
        title = 'low velocity'
    elif "med" in file:
        title = 'medium velocity'
    elif "high" in file:
        title = 'high velocity'

    if file == odom_low or file == global_low:
        x_offset = -0.434
        y_offset = 0.499
        theta_offset = 2.218
    if file == odom_med or file == global_med:
        x_offset = -0.431
        y_offset = 0.601
        theta_offset = 2.137
    if file == odom_high or file == global_high:
        x_offset = -0.425
        y_offset = 0.700
        theta_offset = 2.061
    with open(file,'r') as csvfile:
        plots = csv.reader(csvfile, delimiter=',')
        line = 0
        for row in plots:
            if line != 0:
                if "/odom" in file:
                    x = float(row[1])
                    y = float(row[2])
                    z = float(row[6])
                    w = float(row[7])
                    x_array.append(x*cos(theta_offset)-y*sin(theta_offset) + x_offset)
                    y_array.append(x*sin(theta_offset)+y*cos(theta_offset) + y_offset)
 
                    theta_array.append(normalize_angle(euler_from_quaternion(0,0,z,w)[2] + theta_offset))
                else:
                    x_array.append(float(row[1]))
                    y_array.append(float(row[2]))
                    theta_array.append(euler_from_quaternion(0,0,float(row[6]),float(row[7]))[2])
            line+=1


show_tiago_path(odom_high, x_pose_odom, y_pose_odom, theta_pose_odom, title)
show_tiago_path(global_high, x_pose_global, y_pose_global, theta_pose_global, title)

# show_tiago_path(odom_med, x_pose_odom, y_pose_odom, theta_pose_odom, title)
# show_tiago_path(global_med, x_pose_global, y_pose_global, theta_pose_global, title)

# show_tiago_path(odom_low, x_pose_odom, y_pose_odom, theta_pose_odom, title)
# show_tiago_path(global_low, x_pose_global, y_pose_global, theta_pose_global, title)

error_x = 0
error_y = 0
error_theta = 0

for i in range(min(len(x_pose_odom),len(x_pose_global))):

    error_x += abs(x_pose_odom[i] - x_pose_global[i])
    error_y += abs(y_pose_odom[i] - y_pose_global[i])

    actual_difference = convert_angle(theta_pose_odom[i]) - convert_angle(theta_pose_global[i])
    actual_difference = (actual_difference + 180) % 360 - 180
    error_theta += actual_difference

    i+=1
print(error_x, error_y, error_theta)
print(error_x/min(len(x_pose_odom),len(x_pose_global)), error_y/min(len(x_pose_odom),len(x_pose_global)), error_theta/min(len(x_pose_odom),len(x_pose_global)))
plt.plot(theta_pose_odom, 'r',linestyle = 'dotted', label='odometria')
plt.plot(theta_pose_global, 'b', linestyle = 'dashed', label='globalna lok.')
plt.title('Analiza orientacji dla odometrii i lokalizacji globalnej')
plt.legend()
plt.show()

plt.plot(x_pose_odom, y_pose_odom, 'r',linestyle = 'dotted', label='odometria')
plt.plot(x_pose_global, y_pose_global, 'b', linestyle = 'dashed', label='globalna lok.')
plt.title('Analiza odometrii i lokalizacji globalnej')
plt.legend()
plt.show()