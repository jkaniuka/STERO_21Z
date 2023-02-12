#!/usr/bin/env python
# -*- coding: utf-8 -*-
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
from velma_common import *
import rospy
import PyKDL
from control_msgs.msg import FollowJointTrajectoryResult
from rcprg_planner import *
from rcprg_ros_utils import exitError
from velma_common.velma_interface import VelmaInterface, isConfigurationClose,\
    symmetricalConfiguration
from velma_kinematics.velma_ik_geom import KinematicsSolverLWR4, KinematicsSolverVelma
import numpy as np
import math
from math import radians

from Initialization import Initialization
from JointSpace import JointSpace
from Hand import Hand
from CartSpace import CartSpace
from CartSpace import calc_cabinet_radius, calculate_goals


def main():

    # ****************************** INITIALIZATION (ROBOT, PLANNER, OCTOMAP, OBJECTS) ************************************************************
    rospy.init_node('my_node')
    rospy.sleep(0.5)

    interface = Initialization()
  
    right_hand = Hand(interface.velma, interface.p)

    right_hand.close_hand()


    interface.go_to_default()


    # # JIMP section
    # jimp = JointSpace(interface.velma, interface.p)
    # T_B_handle = interface.velma.getTf("B", "right_handle")
    # q_map_goals = jimp.get_goals(T_B_handle)
    # jimp.init_move()
    # if (jimp.move_JIMP_towards_jar(q_map_goals)): 
    #     print('GOAL REACHED')


    # # CIMP section
    # right_hand.tool_to_grip()
    # right_hand.go_to_handle(interface.velma.getTf("B", "right_handle"))


    # right_hand.get_handle(interface.velma.getTf("B", "right_handle"))
    
    # # right_hand.away_handle(interface.velma.getTf("B", "right_handle"))
    # right_hand.tool_to_wrist()

    # interface.go_to_default()




    # pobiera początkową pozycję uchwytu + w tym układzie będą wyznaczane kolejne punkty docelowe
    frame_0 = interface.velma.getTf("B", "right_handle")

    cart = CartSpace(interface.velma)
    cart.init_move()
    # robocza sekwencja zbliżenia się do szafki i do uchwytu ( bez JIMP )
    cart.move_wrt_frame(interface.velma.getTf("B", "right_handle"), 0.35, 0.2, 0.15, 0)
    cart.move_wrt_frame(interface.velma.getTf("B", "right_handle"), 0.25, 0.2, 0.15, 0)
    cart.move_wrt_frame(interface.velma.getTf("B", "right_handle"), 0.25, -0.01, 0.15, 0)


    cart.setImpedance(100, 100, 200, 100, 100, 100)

    # malutkie przesunięcie, aby móc wyliczyć promień
    cart.move_wrt_frame(interface.velma.getTf("B", "right_handle"), 0.25+0.05, -0.05+ 0.05, 0.15, math.pi/7)

    frame_1 = interface.velma.getTf("B", "right_handle")

    radius = calc_cabinet_radius(frame_0, frame_1)
    print radius

    # wyznacza 5 pozycji pośrednich przy ruchu po okręgu ( wszystko w początkowym układzie uchwytu)
    sub_goals = calculate_goals(5,radius)
    for element in sub_goals:
        print element
        print " "

    # wykonanie ruchu do kolejnych punktów
    for i in range(len(sub_goals)):
            cart.move_wrt_frame(frame_0, 0.25+sub_goals[i][0], -0.05+sub_goals[i][1], sub_goals[i][2], sub_goals[i][3])
            print i

    # Potrzebny jest ruch korekcyjny, bo jest pewien offset między nadgarstkiem, a klamką
    print "correction start"
    cart.move_wrt_frame(frame_0, 0.25+sub_goals[4][0], -0.05+sub_goals[4][1]+0.4, sub_goals[4][2], sub_goals[4][3])
    print "correction end"

    # 2 ruchy na odsunięcie się od klamki 
    print "moving back"
    cart.move_wrt_frame(interface.velma.getTf("B", "right_handle"), 0.28, 0.1, 0.15, 0)
    cart.move_wrt_frame(interface.velma.getTf("B", "right_handle"), 0.28+0.1, 0.1, 0.15, 0)

    # powrót do pozycji początkowej
    interface.go_to_default()


    print "End of task"

    rospy.sleep(1.0)
    return 0

if __name__ == "__main__":
    main()