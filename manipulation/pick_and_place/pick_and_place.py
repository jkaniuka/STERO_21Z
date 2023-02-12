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



from cartesian_space import *
from initialization import *
from joint_space import *


def main():

    # Start (and also end) position in joints space
    q_start = symmetricalConfiguration( {'torso_0_joint':0,
    'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
    'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0,} )

    
    # ****************************** TASK SEQUENCE ************************************************************
    rospy.init_node('my_pick_and_place')
    rospy.sleep(0.5)

    print "Starting Interface..."

    velma = VelmaInterface()
    
    # Robot, planner and octomap initialization
    init_robot(velma)

    p = Planner(velma.maxJointTrajLen())
    init_planner(p)

    oml = OctomapListener("/octomap_binary")
    init_octomap(oml,p)

    # Closing grippers for better planner performance
    close_grippers(velma)

    # Downloading objects TFs from Gazebo
    T_B_Jar = velma.getTf("B", "object1")
    print(T_B_Jar)
    T_B_Table = velma.getTf("B", "table_surface")


    # Approaching jar in JIMP mode
    T_B_Wr= T_B_Wr_aroud_goal( T_B_Jar.p, velma)
    IK, torso_angle = InvKinCalc(T_B_Wr)
    q_map_goals = choose_possible_goals(IK, torso_angle, velma)
    init_move(velma, p) 
    if (move_JIMP_towards_jar(velma, p,q_map_goals)): 
        print('GOAL REACHED')


    gripper_open(velma)

    # Cartesian impedance move when robot is close to the jar
    move_CIMP_towards_jar(velma)

    # Approaching table in JIMP mode
    table_frames= T_B_Wr_aroud_goal3( T_B_Table.p, velma)
    IK_table, torso_angle_table = InvKinCalc(table_frames)
    q_map_goals_table = choose_possible_goals(IK_table, torso_angle_table, velma)
    init_move(velma, p) 
    if (move_JIMP_towards_jar(velma, p,q_map_goals_table)):
        print('GOAL REACHED')

    # Cartesian impedance move when robot is close to the table
    move_CIMP_towards_table(velma)

    # Close grippers for better planner performance, and go to initial position
    gripper_close(velma)

    back_to_default_pose(velma,p,q_start)

    print "End of task"

    rospy.sleep(1.0)
    return 0

if __name__ == "__main__":
    main()