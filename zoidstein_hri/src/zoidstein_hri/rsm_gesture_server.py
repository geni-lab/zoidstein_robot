__author__ = 'alex'

from hri_framework import GestureActionServer
from zoidstein_hri.zoidstein import ZoidGesture
from rsm_serial_node import RSMSerialNode
from threading import Timer
import rospy



#!/usr/bin/env python

class GestureHandle():
    def __init__(self, goal_handle, motion_id, timer):
        self.goal_id = goal_handle.get_goal_id().id
        self.goal_handle = goal_handle
        self.motion_id = motion_id
        self.timer = timer

class RSMGestureActionServer(GestureActionServer):
    NODE_NAME = "RSMGestureServer"
    serialPort = None

    def __init__(self):
        GestureActionServer.__init__(self, ZoidGesture)
        self.gesture_handle_lookup = {}
        self.motion_proxy = None


    def gesture_finished(self, goal_handle):
        super(RSMGestureActionServer, self).gesture_finished(goal_handle)
        self.remove_gesture_handle(goal_handle)

    def start_gesture(self, goal_handle):
        self.rsm_serial_node = RSMSerialNode()
        goal = goal_handle.get_goal()
        goal_duration = self.get_goal_duration(goal_handle)

        if self.has_gesture(goal.gesture):
            gesture = ZoidGesture[goal.gesture]

            (names, times, keys) = ZoidGesture.get_ntk(gesture, goal_duration)
            motion_id = self.motion_proxy.post.angleInterpolationBezier(names, times, keys)
            timer = Timer(goal_duration, self.gesture_finished, goal_handle)
            gesture_handle = GestureHandle(goal_handle, motion_id, timer)
            self.add_gesture_handle(gesture_handle)
            timer.start()

        else:
            self.action_server.set_aborted()

    def get_goal_duration(self, goal_handle):
        return {
            'a': 1,
            'b': 2,
            }.get(goal_handle, 9)

    def get_gesture_handle(self, goal_handle):
        return self.gesture_handle_lookup[goal_handle.get_goal_id().id]

    def add_gesture_handle(self, gesture_handle):
        self.gesture_handle_lookup[gesture_handle.goal_id] = gesture_handle

    def remove_gesture_handle(self, goal_handle):
        self.gesture_handle_lookup.pop(goal_handle.get_goal_id().id)

if __name__ == '__main__':
    rospy.init_node('gesture_action_server')
    gesture_server = RSMGestureActionServer()
    gesture_server.start()
    rospy.spin()
    gesture_server.start_gesture(ZoidGesture.BConDance)