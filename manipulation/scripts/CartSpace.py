#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
from velma_common import *
import rospy
import PyKDL
from rcprg_planner import *
from rcprg_ros_utils import exitError
import numpy as np
import math


def calc_pose_wrt_frame(ref_frame, x, y, z):
    (_, _, yaw) = ref_frame.M.GetRPY()
    # calculated based on rotation matrix (https://en.wikipedia.org/wiki/Rotation_matrix)
    pose_X = ref_frame.p.x() + math.cos(yaw)*x - math.sin(yaw)*y
    pose_Y = ref_frame.p.y() + math.sin(yaw)*x + math.cos(yaw)*y
    pose_Z = ref_frame.p.z() + z 
    angle = yaw - math.pi
    if angle < -math.pi:
        angle = 2*math.pi + angle
    return [pose_X, pose_Y, pose_Z, angle]

def calc_cabinet_radius(frame_before, frame_after):
    # based on the law of cosines
    x0 = frame_before.p.x()
    y0 = frame_before.p.y()
    x1 = frame_after.p.x()
    y1 = frame_after.p.y()
    distance = math.sqrt((x1-x0)**2 + (y1-y0)**2)
    yaw0 = frame_before.M.GetRPY()[2]
    yaw1 = frame_after.M.GetRPY()[2]
    gamma = (yaw0 - abs(yaw1))
    radius = abs(distance)/math.sqrt(2*(1-math.cos(gamma)))
    return radius

def calculate_goals(num_of_subgoals, radius, z_offset = 0.15):
    sub_goals = []
    for i in range(num_of_subgoals):
        y= radius/num_of_subgoals * (1+i)
        x = math.sqrt(y*(2*radius-y))
        z = z_offset
        angle = (math.pi/2)/num_of_subgoals * (1+i)
        sub_goal = [x, y, z, angle]
        sub_goals.append(sub_goal)
    return sub_goals




class CartSpace:

    def __init__(self, velma):
        self.velma = velma

    def init_move(self):
        print "Switch to cart_imp mode (no trajectory)..."
        if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
            exitError(10)
        if self.velma.waitForEffectorRight() != 0:
            exitError(11)
    
        rospy.sleep(0.5)
    
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateCartImp():
            print "The core_cs should be in cart_imp state, but it is not"
            exitError(12)

        rospy.sleep(0.5) 
        diag = self.velma.getCoreCsDiag()
        if not diag.inStateCartImp():
            print "The core_cs should be in cart_imp state, but it is not"
            exitError(3)


    def setImpedance(self, lin_x, lin_y, lin_z, rot_x, rot_y, rot_z):
        if not self.velma.moveCartImpRight(None, None, None, None, [PyKDL.Wrench(PyKDL.Vector(lin_x, lin_y, lin_z), PyKDL.Vector(rot_x, rot_y, rot_z))], [2], PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(16)
        if self.velma.waitForEffectorRight() != 0:
            exitError(17)
        rospy.sleep(1)

    def move_wrt_frame(self, frame, x, y, z, angle, tol=1):
        [x_f, y_f, z_f, angle_f] = calc_pose_wrt_frame(frame, x, y, z)
        goal = PyKDL.Frame(PyKDL.Rotation.RPY(0, 0, angle_f+angle), PyKDL.Vector(x_f, y_f, z_f))
        if not self.velma.moveCartImpRight([goal], [5.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.1, path_tol=PyKDL.Twist(PyKDL.Vector(tol, tol, tol), PyKDL.Vector(tol, tol, tol))):
            exitError(8)
        if self.velma.waitForEffectorRight() != 0:
            exitError(9)
        rospy.sleep(0.5)






