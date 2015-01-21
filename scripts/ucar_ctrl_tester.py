#!/usr/bin/env python
# Software License Agreement (BSD License)
#
# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of Willow Garage, Inc. nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
# Revision $Id$

## Simple talker demo that published std_msgs/Strings messages
## to the 'chatter' topic

import rospy, sys, random
from std_msgs.msg import String 

from cob_msgs.msg import EmergencyStopState
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticStatus
from control_msgs.msg import JointTrajectoryControllerState

def test_Twist():
    '''
    PUBLISHER METHODE: Twist
    '''
    pub_twist = rospy.Publisher('command', Twist, queue_size=10)
    Twist_msg = Twist()
    Twist_msg.linear.x = random.uniform(0.0, 1.0) # 0.5
    Twist_msg.linear.y = random.uniform(0.0, 1.0) # 0.23
    Twist_msg.angular.z = random.uniform(0.0, 1.5) # 0.1

    print "Twist: " + Twist_msg.linear.x.__str__() + " , " + Twist_msg.linear.y.__str__() + " , " + Twist_msg.angular.z.__str__()
    pub_twist.publish(Twist_msg)

def test_EmergencyStopState():
    '''
    PUBLISHER METHODE: EmergencyStopState
    '''
    pub_EMStop = rospy.Publisher('/emergency_stop_state', EmergencyStopState, queue_size=10)

    EM_msg = EmergencyStopState()
    #EM_msg.emergency_state = EM_msg.EMFREE
    EM_msg.emergency_state = EM_msg.EMSTOP

    print "EMState: " + EM_msg.emergency_state.__str__()
    pub_EMStop.publish(EM_msg)

def test_DiagnosticStatus():
    '''
    PUBLISHER METHODE: DiagnosticStatus
    '''
    pub_DiagnosticStatus = rospy.Publisher('diagnostic', DiagnosticStatus, queue_size=10)

    message = DiagnosticStatus()
    message.name = "ROB"
    message.message = "RUNNING"

    print "DiagnosticStatus: " + message.name + " , " + message.message
    pub_DiagnosticStatus.publish(message)

def test_JointTrajectoryControllerState():
    '''
    PUBLISHER METHODE: JointControllerStates
    '''
    pub_JointControllerStates = rospy.Publisher('state', JointTrajectoryControllerState, queue_size=10)

    message = JointTrajectoryControllerState()
    message.header.frame_id = "testID: 101"

    message.joint_names = ["fl_caster_r_wheel_joint", "bl_caster_r_wheel_joint", "br_caster_r_wheel_joint", "fr_caster_r_wheel_joint", "fl_caster_rotation_joint", "bl_caster_rotation_joint", "br_caster_rotation_joint", "fr_caster_rotation_joint"]
    message.actual.velocities = [4,4,4,4,4,4,4,4]
    message.actual.positions = [4,4,4,4,4,4,4,4]

    print "JointControllerStates: " + message.header.frame_id
    pub_JointControllerStates.publish(message)

def usage():
    return "python_ucar_ctrl_Tester: ..."

def auto_publish_messages():
    '''
    MAIN METHODE WITCH CONTROLS ALL PUBLSHERS
    '''

    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
 
    while not rospy.is_shutdown():
        # call specific testing methods
#        test_Twist()
        #test_EmergencyStopState()
        test_JointTrajectoryControllerState()
        #test_DiagnosticStatus()

        r.sleep()
        
if __name__ == '__main__':
    try:
        auto_publish_messages()
    except rospy.ROSInterruptException:
        usage()
        pass
