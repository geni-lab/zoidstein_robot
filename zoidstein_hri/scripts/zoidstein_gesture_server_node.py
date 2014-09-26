#!/usr/bin/env python
# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'Alex van der Peet, James Diprose'

from hri_framework import IGestureActionServer
from zoidstein_hri.zoidstein import Gesture, AnimationType
from zoidstein_hri import RSMSerialNode
from threading import Timer
import rospy
import actionlib
from ros_pololu_servo.msg import pololu_trajectoryAction, pololu_trajectoryGoal
from trajectory_msgs.msg import JointTrajectory
from ros_blender_bridge.srv import GetJointTrajectory


class ZoidsteinGestureActionServer(IGestureActionServer):

    def __init__(self):
        super(ZoidsteinGestureActionServer, self).__init__(Gesture)
        self.rsm_serial_node = RSMSerialNode()
        self.traj_action_client = actionlib.SimpleActionClient('pololu_trajectory_action_server', pololu_trajectoryAction)
        self.blender_traj_srv = rospy.ServiceProxy('get_joint_trajectory', GetJointTrajectory)
        self.keyframe_gh = None

    def start(self):
        self.rsm_serial_node.open()
        super(ZoidsteinGestureActionServer, self).start()

    def start_gesture(self, goal_handle):
        goal = goal_handle.get_goal()

        if self.is_valid_gesture(goal.gesture):
            gesture = Gesture[goal.gesture]
            duration = goal.duration

            if duration == -1:
                duration = Gesture.default_duration

            if gesture.animation_type is AnimationType.BodyCon:
                self.rsm_serial_node.executeScript(gesture.bcon_script)
                timer = Timer(duration, self.set_succeeded, [goal_handle])
                timer.start()

            elif gesture.animation_type is AnimationType.Keyframe:
                rospy.loginfo('Fetching trajectory msg from Blender')
                resp = self.blender_traj_srv(gesture.name)

                rospy.loginfo('Sending trajectory message to pololu joint trajectory server')
                goal = pololu_trajectoryGoal()
                goal.joint_trajectory = resp.joint_trajectory
                self.keyframe_gh = goal_handle
                self.traj_action_client.send_goal(goal, done_cb=self.keyframe_animation_finished)

        else:
            self.set_aborted(goal_handle)

    def keyframe_animation_finished(self):
        self.set_succeeded(self.keyframe_gh)
        self.keyframe_gh = None

    def cancel_gesture(self, goal_handle):
        #TODO: stop RSM from performing action
        #TODO: stop timer for this gesture
        rospy.logwarn('Not implemented, need to find a way to stop RSM from performing action')

    def __exit__(self, type, value, traceback):
        self.rsm_serial_node.close()


if __name__ == "__main__":
    rospy.init_node('gesture_action_server')
    gesture_server = ZoidsteinGestureActionServer()
    gesture_server.start()
    rospy.spin()