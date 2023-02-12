#!/usr/bin/env python
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
from velma_common import *
import rospy
import PyKDL
from rcprg_planner import *
from rcprg_ros_utils import exitError
import numpy as np
import math
from math import radians

def deg2rad(deg):
    return float(deg)/180.0*math.pi 




class Hand:

    def __init__(self, velma, p):
        self.velma = velma
        self.p = p
        if not self.velma.moveCartImpRightCurrentPos(start_time=0.2):
            exitError(8)
        if self.velma.waitForEffectorRight() != 0:
            exitError(9)


    def close_hand(self):
        dest_q = [deg2rad(100), deg2rad(100), deg2rad(100), deg2rad(180)]
        self.velma.moveHand('right', dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
        dest_q = [deg2rad(180), deg2rad(180), deg2rad(180), deg2rad(0)]
        self.velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
        if self.velma.waitForHand('right') != 0:
            exitError(2)
        if self.velma.waitForHandLeft() != 0:
            exitError(2)
        rospy.sleep(1)   

    def go_to_handle(self, T_B):
        T_B_handle = T_B * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.01, +0.12, +0.06))
        T_B_handle = T_B_handle * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

        if not self.velma.moveCartImpRight([T_B_handle], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(8)
        if self.velma.waitForEffectorRight() != 0:
            print "go_to_handle"
            # exitError(9)

    def get_handle(self, T_B):
        T_B_handle = T_B * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, -0.04, +0.06))
        T_B_handle = T_B_handle * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))

        if not self.velma.moveCartImpRight([T_B_handle], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(0.4,0.4,0.4), PyKDL.Vector(0.4,0.4,0.4)), start_time=0.5):
                exitError(8)
        if self.velma.waitForEffectorRight() != 0:
            print "get_handle"
        #         exitError(9)
        rospy.sleep(0.5)


    # def tool_to_grip(self):
    #     T_B_Gr = self.velma.getTf("B", "Gr")
    #     T_Wr_Gr = self.velma.getTf("Wr", "Gr")
    #     # T_B_Gr = [T_B_Gr*T_Wr_Gr]
    #     if not self.velma.moveCartImpRight([T_B_Gr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
    #         exitError(18)
    #     if self.velma.waitForEffectorRight() != 0:
    #         print "tool_to_grip"
    #         exitError(19)
    #     rospy.sleep(0.5)

    def tool_to_grip(self):
        T_B_Wr = self.velma.getTf("B", "Wr")
        T_Wr_Gr = self.velma.getTf("Wr", "Gr")
        T_B_Gr = [T_B_Wr*T_Wr_Gr]
        if not self.velma.moveCartImpRight([T_B_Wr*T_Wr_Gr], [0.1], [T_Wr_Gr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        if self.velma.waitForEffectorRight() != 0:
            exitError(19)
        rospy.sleep(0.5)



    def tool_to_wrist(self):
        T_B_Wr = self.velma.getTf("B", "Wr")
        T_Gr_Wr = self.velma.getTf("Gr", "Gr")
        # T_B_Gr = [T_B_Wr*T_Gr_Wr]
        if not self.velma.moveCartImpRight([T_B_Wr*T_Gr_Wr], [0.1], [T_Gr_Wr], [0.1], None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
            exitError(18)
        if self.velma.waitForEffectorRight() != 0:
            print "tool_to_wrist"
            exitError(19)
        rospy.sleep(0.5)


    def away_handle(self, T_B):
        T_B_handle = T_B * PyKDL.Frame(PyKDL.Rotation.RotY(radians(90)), PyKDL.Vector(+0.007, +0.12, +0.06))
        T_B_handle = T_B_handle * PyKDL.Frame(PyKDL.Rotation.RotX(radians(180)), PyKDL.Vector(0, 0, 0))
        
        if not self.velma.moveCartImpRight([T_B_handle], [10.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
                exitError(8)
        if self.velma.waitForEffectorRight() != 0:
                print "away_handle"
                exitError(9)








