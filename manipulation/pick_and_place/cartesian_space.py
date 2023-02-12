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

# Approach jar with CIMP
def move_moveCartImp(velma, frame):
    print "Moving right wrist to pose defined in world frame..."
    if not velma.moveCartImpRight([frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)



# Retreat to get out of a collision with the environment
def move_moveCartImp_vector(velma, vector):
    print "Moving right wrist to pose defined in world frame..."
    T_B_Trd = PyKDL.Frame(PyKDL.Rotation.Quaternion( 0.0 , 0.0 , 0.0 , 1.0 ), vector)
    if not velma.moveCartImpRight([T_B_Trd], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)



def move_CIMP_towards_jar(velma):
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)


    T_B_Jar = velma.getTf("B", "object1")
    base_wrist=velma.getTf("B", "Wr")


    T_B_Jar_angle=math.atan2(T_B_Jar.p.y(), T_B_Jar.p.x())
    
    rotation_angle=T_B_Jar_angle
    rotation=PyKDL.Rotation.RotZ(rotation_angle)

    
    pose=T_B_Jar.p
    if(T_B_Jar.p.x()<0):
        pose=pose+PyKDL.Vector(0.17,0,0)
    else:
        pose=pose+PyKDL.Vector(-0.17,0,0)

    if(T_B_Jar.p.y()<0):
        pose=pose+PyKDL.Vector(0,0.17,0)
    else:
        pose=pose+PyKDL.Vector(0,-0.17,0)
    
    pose=pose+PyKDL.Vector(0,0,0.09)
    
    frame=PyKDL.Frame(rotation,pose)
    move_moveCartImp(velma, frame)
   
    
    gripper_close(velma)
    
    T_B_Wr=velma.getTf("B", "Wr")
    move_moveCartImp_vector(velma,T_B_Wr.p+PyKDL.Vector(0,0,0.1))




def move_CIMP_towards_table(velma):
    if not velma.moveCartImpRightCurrentPos(start_time=0.2):
        exitError(8)
    if velma.waitForEffectorRight() != 0:
        exitError(9)


    T_B_Jar = velma.getTf("B", "table_surface")
    base_wrist=velma.getTf("B", "Wr")


    T_B_Jar_angle=math.atan2(T_B_Jar.p.y(), T_B_Jar.p.x())
    
    rotation_angle=T_B_Jar_angle
    rotation=PyKDL.Rotation.RotZ(rotation_angle)

    
    pose=T_B_Jar.p
 
    
    pose=pose+PyKDL.Vector(0,0,0.3)
    
    frame=PyKDL.Frame(rotation,pose)
    move_moveCartImp(velma, frame)
   
    
    gripper_open(velma)
    
    T_B_Wr=velma.getTf("B", "Wr")
    move_moveCartImp_vector(velma,T_B_Wr.p+PyKDL.Vector(0,0,0.1))


def gripper_close(velma):
    print("close gripper")
    dest_q = [deg2rad(180), deg2rad(180), deg2rad(180), deg2rad(0)]
    velma.moveHandRight(dest_q, [1.25, 1.25, 1.25, 1.25], [4000,4000,4000,4000], 1000, hold=False)
    rospy.sleep(5)

def gripper_open(velma):
    rospy.sleep(1)
    print "opening gripper"
    velma.moveHandRight([0, 0, 0, 0], [1, 1, 1, 1], [4000,4000,4000,4000], 1000, hold=True)
    rospy.sleep(5)


def deg2rad(deg):
    return float(deg)/180.0*math.pi

# Close both grippers at the beginning of Pick & Place task
def close_grippers(velma):
    dest_q = [deg2rad(180), deg2rad(180), deg2rad(180), deg2rad(0)]
    velma.moveHandLeft(dest_q, [1, 1, 1, 1], [8000, 8000, 8000, 8000], 100000000, hold=False)
    velma.moveHandRight(dest_q, [1.25, 1.25, 1.25, 1.25], [4000,4000,4000,4000], 1000, hold=False)
    if velma.waitForHandLeft() != 0:
        exitError(2)
    if velma.waitForHandRight() != 0:
        exitError(4)