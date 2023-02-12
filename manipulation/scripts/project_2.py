#!/usr/bin/env python
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
from CartSpace import calc_cabinet_radius


def main():

    # ****************************** INITIALIZATION (ROBOT, PLANNER, OCTOMAP, OBJECTS) ************************************************************
    rospy.init_node('my_node')
    rospy.sleep(0.5)

    interface = Initialization()
  
    right_hand = Hand(interface.velma, interface.p)

    right_hand.close_hand()


    interface.go_to_default()


    # JIMP section
    jimp = JointSpace(interface.velma, interface.p)
    T_B_handle = interface.velma.getTf("B", "right_handle")
    q_map_goals = jimp.get_goals(T_B_handle)
    jimp.init_move()
    if (jimp.move_JIMP_towards_jar(q_map_goals)): 
        print('GOAL REACHED')


    # CIMP section
    right_hand.tool_to_grip()
    right_hand.go_to_handle(interface.velma.getTf("B", "right_handle"))


    # right_hand.get_handle(interface.velma.getTf("B", "right_handle"))
    
    # right_hand.away_handle(interface.velma.getTf("B", "right_handle"))
    right_hand.tool_to_wrist()

    #interface.go_to_default()

    print "End of task"

    rospy.sleep(1.0)
    return 0

if __name__ == "__main__":
    main()