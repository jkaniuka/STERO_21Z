#!/usr/bin/env python
 
 
 
 # Copyright (c) 2017, Robot Control and Pattern Recognition Group,
 # Institute of Control and Computation Engineering
 # Warsaw University of Technology
 #
 # All rights reserved.
 # 
 # Redistribution and use in source and binary forms, with or without
 # modification, are permitted provided that the following conditions are met:
 #     * Redistributions of source code must retain the above copyright
 #       notice, this list of conditions and the following disclaimer.
 #     * Redistributions in binary form must reproduce the above copyright
 #       notice, this list of conditions and the following disclaimer in the
 #       documentation and/or other materials provided with the distribution.
 #     * Neither the name of the Warsaw University of Technology nor the
 #       names of its contributors may be used to endorse or promote products
 #       derived from this software without specific prior written permission.
 # 
 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 # ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 # WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 # DISCLAIMED. IN NO EVENT SHALL <COPYright HOLDER> BE LIABLE FOR ANY
 # DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 # (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 # ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 # (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 # SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 #
 # Author: Dawid Seredynski
 #
import roslib; roslib.load_manifest('velma_task_cs_ros_interface')
 
import rospy
import copy
 
from velma_common import *
from rcprg_ros_utils import exitError

if __name__ == "__main__":

    rospy.init_node('head_test', anonymous=False)

 
    print "This test/tutorial executes initialization"\
        " procedures required for robot operation.\n"


    velma = VelmaInterface()

    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)


    if velma.enableMotors() != 0:
        exitError(14)

    velma.startHomingHP()
    if velma.waitForHP() != 0:
        exitError(14)

    velma.startHomingHT()
    if velma.waitForHT() != 0:
        exitError(15)
    print "Init successful."

    rospy.sleep(0.5)


    velma = VelmaInterface()

    if not velma.waitForInit(timeout_s=10.0):
        print "Could not initialize VelmaInterface\n"
        exitError(1)


    diag = velma.getCoreCsDiag()
    if not diag.motorsReady():
        print "Motors must be homed and ready to use for this test."
        exitError(1)

    if velma.enableMotors() != 0:
        exitError(2)

    js_start = velma.getLastJointState()
    velma.moveJoint(js_start[1], 0.5, start_time=0.5, position_tol=15.0/180.0*math.pi)
    error = velma.waitForJoint()
    if error != 0:
        print "The action should have ended without error, but the error code is", error
        exitError(3)

    destination_torso  =[{'torso_0_joint':0}, {'torso_0_joint':1.56}, {'torso_0_joint':-1.56} ]
    destination_head = [(0,0), (1.56, 0), (1.56, 0.7), (-1.56, 0.7), (-1.56, -0.7), (1.56, -0.7), (-1.56, 0)]

    for q_dest_torso in destination_torso:

        print "moving torso to position: ", q_dest_torso

        velma.moveJoint(q_dest_torso, 4.0, start_time=0.5, position_tol=15.0/180.0*math.pi)
        error = velma.waitForJoint()
        if error != 0:
            exitError(9, msg="The action should have ended without error,"\
                            " but the error code is {}".format(error))

        for q_dest in destination_head:

            print "moving head to position: ", q_dest
            # q_dest = (1.56, 0)
            velma.moveHead(q_dest, 5, start_time=0.5)
            if velma.waitForHead() != 0:
                exitError(6)
            rospy.sleep(0.5)
            if not isHeadConfigurationClose( velma.getHeadCurrentConfiguration(), q_dest, 0.1 ):
                exitError(7)


    exitError(0)