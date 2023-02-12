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


from project_1 import close_grippers

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

def deg2rad(deg):
    return float(deg)/180.0*math.pi

def init_robot(velma):
    rospy.sleep(0.5)
    print "Waiting for VelmaInterface initialization..."
    if not velma.waitForInit(timeout_s=10.0):
        exitError(1, msg="Could not initialize VelmaInterface")
    print "Initialization ok!\n"

    if velma.enableMotors() != 0:
        exitError(14)

    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)

    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)

    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        exitError(1, msg="Motors must be homed and ready to use for this test.")

def init_planner(p):
    if not p.waitForInit(timeout_s=10.0):
        exitError(2, msg="Could not initialize Planner")
    print "Planner initialization ok!"

def init_octomap(oml,p):
    rospy.sleep(1.0)
    octomap = oml.getOctomap(timeout_s=5.0)
    p.processWorld(octomap)


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


def back_to_default_pose(velma,p,q_start):
    print "JIMP MODE"
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)
    
    print "Moving to basic pose"
    plan_and_move(velma, p, q_start)


def InvKinCalc(T_B_Wrs):
    my_Iks= []
    torso_angles_ik= []
    solver = KinematicsSolverVelma()
<<<<<<< HEAD
    
    for torso_angle in np.linspace(1.3, -1.3, 10):
        for elbow_circle_angle in np.linspace(1.5, -1.5, 10):
=======

    for torso_angle in np.linspace(1.0, -1.0, 8):
        for elbow_circle_angle in np.linspace(1.55, -1.55, 8):
>>>>>>> 256ef4e52ec8ffd29cfbefbc05a834f66aa80030
            for T_B_Wr in T_B_Wrs:
                ik = solver.calculateIkRightArm(T_B_Wr, torso_angle, elbow_circle_angle, False, False, False)
                if None not in ik:
                    my_Iks.append(ik)
                    torso_angles_ik.append(torso_angle)
    return my_Iks, torso_angles_ik


# picking jar 
def T_B_Wr_aroud_goal(point, velma):
    R=0.15 # R = 0.3
    angle_step=math.pi/8 # angle_step = math.pi/16 -> zbyt mały krok metody (nie jest to potrzebne)
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
<<<<<<< HEAD
        frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))# * PyKDL.Frame(PyKDL.Rotation.RPY(0,0,math.pi))
=======
        frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))
>>>>>>> 256ef4e52ec8ffd29cfbefbc05a834f66aa80030
        T_B_Wr.append(frame*velma.getTf('Gr', 'Wr'))
    return T_B_Wr


# disposing jar
def T_B_Wr_aroud_goal2(point, velma):
    R=0.15
    angle_step=math.pi/16
    steps = int(round(2*math.pi/angle_step))
    T_B_Wr = []
    for step in range(steps):
        angle = angle_step * step
        x = point.x() - 0.2 + R * math.cos(angle)
        y = point.y()- 0.2 + R * math.sin(angle)
        z = point.z() + 0.2
        roll = pitch = 0.0
        yaw = correct_angle(math.pi + angle)
        rotation = PyKDL.Rotation.RPY(roll, pitch, yaw)
        vector = PyKDL.Vector(x, y, z)
        frame =  PyKDL.Frame(rotation, vector) * PyKDL.Frame(PyKDL.Rotation.RPY(0,math.pi/2,0))# * PyKDL.Frame(PyKDL.Rotation.RPY(0,0,math.pi))
        T_B_Wr.append(frame*velma.getTf('Gr', 'Wr'))
    return T_B_Wr



def correct_angle(angle):
    angle =  angle % (2*math.pi)
    angle = (angle + 2*math.pi) % (2*math.pi)
    if angle > math.pi:  
        angle =angle- 2*math.pi
    return angle

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



def executeJIMPTraj(velma, p,q_map_goals):  
    constraint=[]
    for goal in q_map_goals:
        constraint.append(qMapToConstraints(goal, 0.01, group=velma.getJointGroup("impedance_joints")))
    for i in range(10):
        rospy.sleep(1)
        js = velma.getLastJointState()
        print("Planning (try", i, ")...")
        traj = p.plan(js[1], constraint, "impedance_joints", num_planning_attempts=10, max_velocity_scaling_factor=0.15, planner_id="RRTConnect")
    
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



def move_JIMP_towards_jar(velma,p,q_before):
    # print "JIMP MODE"
    # velma.moveJointImpToCurrentPos(start_time=0.5)
    # error = velma.waitForJoint()
    # if error != 0:
    #     print "The action should have ended without error, but the error code is", error
    #     exitError(4)

    # print "Checking if the starting configuration is as expected..."
    # rospy.sleep(0.5)
    # js = velma.getLastJointState()

    # T_B_Jar = velma.getTf("B", "object1")

    # T_B_Wr= T_B_Wr_aroud_goal(T_B_Jar.p, velma)
    # IK, torso_angle= InvKinCalc(T_B_Wr)

    # if IK != None:
    #     goal = {'torso_0_joint':torso_angle, 'right_arm_0_joint':IK[0], 'right_arm_1_joint':IK[1],
    #         'right_arm_2_joint':IK[2], 'right_arm_3_joint':IK[3], 'right_arm_4_joint':IK[4], 'right_arm_5_joint':IK[5],
    #         'right_arm_6_joint':IK[6]}
    #     init_move(velma, p)
    #     executeJIMPTraj(velma, p, goal)


    
        

def carry_jar(velma,p):
    print "JIMP MODE"
    velma.moveJointImpToCurrentPos(start_time=0.5)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(4)

    print "Checking if the starting configuration is as expected..."
    rospy.sleep(0.5)
    js = velma.getLastJointState()

    T_B_Table = velma.getTf("B", "table_surface")

    T_B_Wr= T_B_Wr_aroud_goal2(T_B_Table.p, velma)
    IK, torso_angle= InvKinCalc(T_B_Wr)

    if IK != None:
        table_goal = {'torso_0_joint':torso_angle, 'right_arm_0_joint':IK[0], 'right_arm_1_joint':IK[1],
            'right_arm_2_joint':IK[2], 'right_arm_3_joint':IK[3], 'right_arm_4_joint':IK[4], 'right_arm_5_joint':IK[5],
            'right_arm_6_joint':IK[6]}

        init_move(velma, p)
        executeJIMPTraj(velma, p, table_goal)


def move_moveCartImp(velma, frame):
    print "Moving right wrist to pose defined in world frame..."
    if not velma.moveCartImpRight([frame], [3.0], None, None, None, None, PyKDL.Wrench(PyKDL.Vector(5,5,5), PyKDL.Vector(5,5,5)), start_time=0.5):
        exitError(13)
    if velma.waitForEffectorRight() != 0:
        exitError(14)
    rospy.sleep(0.5)

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
        pose=pose+PyKDL.Vector(0.14,0,0)
    else:
        pose=pose+PyKDL.Vector(-0.14,0,0)

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



def main():

    # Posisions are const - still working on IK, it goes better and better ;-)
    q_start = symmetricalConfiguration( {'torso_0_joint':0,
    'right_arm_0_joint':-0.3, 'right_arm_1_joint':-1.8, 'right_arm_2_joint':1.25,
    'right_arm_3_joint':0.85, 'right_arm_4_joint':0, 'right_arm_5_joint':-0.5,
    'right_arm_6_joint':0,} )

    # Pose near jar (for testing purposes only!)
    q_before = {'torso_0_joint':-0.2024, 'right_arm_0_joint':-0.0405, 'right_arm_1_joint':-1.9396,
         'right_arm_2_joint':2.2042, 'right_arm_3_joint':1.3031, 'right_arm_4_joint':0.1361, 'right_arm_5_joint':-1.7205,
         'right_arm_6_joint':0.8851, 'left_arm_0_joint':0.3, 'left_arm_1_joint':1.8, 'left_arm_2_joint':-1.25,
         'left_arm_3_joint':-0.85, 'left_arm_4_joint':0, 'left_arm_5_joint':0.5, 'left_arm_6_joint':0, }
    



    # ****************************** INITIALIZATION (ROBOT, PLANNER, OCTOMAP, OBJECTS) ************************************************************
    rospy.init_node('my_node')
    rospy.sleep(0.5)

    print "Starting Interface..."
    moveHand= MoveHand() 
    inverseKinematics= InverseKinematics()
    velma = VelmaInterface()
    init_robot(velma)

    print "Waiting for Planner initialization..."
    p = Planner(velma.maxJointTrajLen())
    init_planner(p)

    oml = OctomapListener("/octomap_binary")
    init_octomap(oml,p)

    close_grippers(velma)

    T_B_Jar = velma.getTf("B", "object1")
    print(T_B_Jar)
    T_B_Table = velma.getTf("B", "table_surface")

    if T_B_Jar== NONE:
        print('NONW')

<<<<<<< HEAD
  
=======

    #move_JIMP_towards_jar(velma,p,q_before)
    T_B_Wr= T_B_Wr_aroud_goal( T_B_Jar.p, velma)
    IK, torso_angle = InvKinCalc(T_B_Wr)

>>>>>>> 256ef4e52ec8ffd29cfbefbc05a834f66aa80030

    # Cartesian impedance implementation
    gripper_open(velma)


    move_CIMP_towards_jar(velma)



    carry_jar(velma,p)

    # Cartesian impedance implementation

    gripper_open(velma)

    back_to_default_pose(velma,p,q_start)



    print "End of task"

    rospy.sleep(1.0)
    return 0

if __name__ == "__main__":
    main()