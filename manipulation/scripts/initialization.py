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


from joint_space import *

from cartesian_space import *
from initialization import *



class Initialization:

    def __init__(self):

        self.velma = self.init_velma()
        self.p = self.init_planner()
        self.q_start = symmetricalConfiguration( {'torso_0_joint':0,
    'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
    'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0,} )


    def init_velma(self):

        print("Starting Interface...")
        velma = VelmaInterface()
        init_robot(velma)
        return velma

    def init_planner(self):

        print("Waiting for Planner initialization...")
        p = Planner(self.velma.maxJointTrajLen())
        init_planner(p)
        return p

    def init_octomat(self):
        print("Waiting for Octomap initialization...")
        oml = OctomapListener("/octomap_binary")
        init_octomap(oml,self.p)

    def go_to_default(self):
        print("JIMP MODE")
        self.velma.moveJointImpToCurrentPos(start_time=0.5)
        error = self.velma.waitForJoint()
        if error != 0:
            print("The action should have ended without error, but the error code is", error)
            exitError(4)

        print("Moving to basic pose")
        plan_and_move(self.velma, self.p, self.q_start)





if __name__ == "__main__":
    pass