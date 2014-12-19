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

import rospy
from std_msgs.msg import String 
from cob_msgs.msg import EmergencyStopState
from geometry_msgs.msg import Twist
from diagnostic_msgs.msg import DiagnosticStatus
from control_msgs.msg import JointTrajectoryControllerState

def test_JointTrajectoryControllerState():

    publisher_4 = rospy.Publisher('state', JointTrajectoryControllerState, queue_size=10)

    message = JointTrajectoryControllerState()
    message.header.frame_id = "test_frame_id"

    print "JointTrajectoryControllerState: " + message.header.frame_id
    publisher_4.publish(message)

def talker():
    pub = rospy.Publisher('/emergency_stop_state', EmergencyStopState, queue_size=10)
    pub2 = rospy.Publisher('command', Twist, queue_size=10)



    rospy.init_node('talker', anonymous=True)
    r = rospy.Rate(10) # 10hz
 
    while not rospy.is_shutdown():
	EM_msg = EmergencyStopState()

	Twist_msg = Twist()
	Twist_msg.linear.x = 10000000000000

        EM_msg.emergency_state = EM_msg.EMFREE
        #EM_msg.emergency_state = EM_msg.EMSTOP

	print "EMState: " + EM_msg.emergency_state.__str__()
        pub.publish(EM_msg)

	print "Twist: " + Twist_msg.linear.x.__str__()
        pub2.publish(Twist_msg)

        # testing JointTrajectoryControllerState
        test_JointTrajectoryControllerState()

        r.sleep()
        
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
