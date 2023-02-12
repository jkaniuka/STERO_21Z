
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


# Plan trajectory using planner
def plan_and_move(velma, p, q_dest):
    print "Planning motion to the goal position using set of all joints..."
    goal_constraint = qMapToConstraints(q_dest, 0.01, group=velma.getJointGroup("impedance_joints"))
    for i in range(5):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print "Planning (try", i, ")..."
        traj = p.plan(js[1], [goal_constraint], "impedance_joints", max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        if traj == None:
            continue
        print "Executing trajectory..."
        if not velma.moveJointTraj(traj, start_time=0.5):
            exitError(5)
        if velma.waitForJoint() == 0:
            break
        else:
            print "The trajectory could not be completed, retrying..."
            continue
    rospy.sleep(0.5)
    js = velma.getLastJointState()
    if not isConfigurationClose(q_dest, js[1]):
        exitError(6)


# Move to default pose using planner
def back_to_default_pose(velma,p,q_start):
    print "JIMP MODE"
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    
    print "Moving to basic pose"
    plan_and_move(velma, p, q_start)

# Calculate InvKin
def InvKinCalc(T_B_Wrs):
    my_Iks= []
    torso_angles_ik= []
    solver = KinematicsSolverVelma()

    for torso_angle in np.linspace(1.0, -1.0, 8):
        for elbow_circle_angle in np.linspace(math.pi, -math.pi, 8):
            for T_B_Wr in T_B_Wrs:
                ik = solver.calculateIkRightArm(T_B_Wr, torso_angle, elbow_circle_angle, False, False, False)
                if None not in ik:
                    my_Iks.append(ik)
                    torso_angles_ik.append(torso_angle)
    return my_Iks, torso_angles_ik



# Generate possible destinations (for jar object)
def T_B_Wr_aroud_goal(point, velma):
    R=0.15 # R = 0.3
    angle_step=math.pi/8 # angle_step 
    steps = int(round(2*math.pi/angle_step))
    T_B_Wr = []
    for step in range(steps):
        angle = angle_step * step
        x = point.x() + R * math.cos(angle)
        y = point.y() + R * math.sin(angle)
        z = point.z() 
        roll = pitch = 0.0
        yaw = correct_angle(math.pi + angle)
        rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
        vector = PyKDL.Vector(x, y, z)
        frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))
        T_B_Wr.append(frame*velma.getTf('Gr', 'Wr'))
    return T_B_Wr

# Generate possible destinations (for table object) and set offsets for better planning
def T_B_Wr_aroud_goal3(point, velma):
    R=0.15 # R = 0.3
    angle_step=math.pi/8 # angle_step
    steps = int(round(2*math.pi/angle_step))
    T_B_Wr = []
    for step in range(steps):
        angle = angle_step * step
        x = point.x()
        y = point.y()
        z = point.z() + 0.45
        roll = pitch = 0.0
        yaw = correct_angle(math.pi + angle)
        rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
        vector = PyKDL.Vector(x, y, z)
        frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))
        T_B_Wr.append(frame*velma.getTf('Gr', 'Wr'))
    return T_B_Wr



def correct_angle(angle):
    angle =  angle % (2*math.pi)
    angle = (angle + 2*math.pi) % (2*math.pi)
    if angle > math.pi:  
        angle =angle- 2*math.pi
    return angle



# Init move in joits space
def init_move(velma, p):
    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print("Motors must be homed and ready to use for this test.")
        exitError(1)
    if velma.enableMotors() != 0:
        exitError(3)

    print("Switch to jnt_imp mode (no trajectory)...")
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print("The action should have ended without error, but the error code is", error)
        exitError(4)


# Execute move in JIMP mode
def move_JIMP_towards_jar(velma, p, q_map_goals):
    object1=None
    print("Moving to valid position, using planned trajectory.")
    goal_constraint_1=[]
    for goal in q_map_goals:
        goal_constraint_1.append(qMapToConstraints(goal, 0.01, group=velma.getJointGroup("impedance_joints")))
    for i in range(15):
        rospy.sleep(0.5)
        js = velma.getLastJointState()
        print("Planning (try", i, ")...")
        if object1==None:
            traj = p.plan(js[1], goal_constraint_1, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
        else:
            traj = p.plan(js[1], goal_constraint_1, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect", attached_collision_objects=[object1])
        
        if traj == None:
            continue
        print("Executing trajectory...")
        if not velma.moveJointTraj(traj, start_time=0.5, position_tol=10.0/180.0 * math.pi, velocity_tol=10.0/180.0*math.pi):
            exitError(5)
        if velma.waitForJoint() == 0:
            return True
        else:
            print("The trajectory could not be completed, retrying...")
            continue

    return False


# Completing goals based on constraints and IK solutions
def choose_possible_goals(IK, torso_angle, velma):
    if IK != None or len(IK)!=0:
        q_map_goals= []
        dict_joint_limits= velma.getBodyJointLimits()
        for i in range(0, len(IK)):
            q_map_goal = {'torso_0_joint':torso_angle[i], 'right_arm_0_joint':IK[i][0], 'right_arm_1_joint':IK[i][1],
                'right_arm_2_joint':IK[i][2], 'right_arm_3_joint':IK[i][3], 'right_arm_4_joint':IK[i][4], 'right_arm_5_joint':IK[i][5],
                'right_arm_6_joint':IK[i][6],  'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
            'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0 }
            
            if (not isJointLimitClose(dict_joint_limits, q_map_goal)):
                q_map_goals.append(q_map_goal)
        
        return q_map_goals


# Checking how close is Velma to joints limits
def isJointLimitClose(dict_joint_limits, q_map_goal):
    for key, value in q_map_goal.items():
        if dict_joint_limits[key][0] + 0.1 > value:
            return True
        if dict_joint_limits[key][1] - 0.1 < value:
            return True
    return False

    