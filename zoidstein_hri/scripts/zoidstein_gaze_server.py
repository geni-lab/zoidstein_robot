#!/usr/bin/env python

import rospy
from hri_msgs.msg import GazeAction, GazeActionFeedback
from hri_framework import GazeActionServer
from geometry_msgs.msg import PointStamped, Point
from std_srvs.srv import Empty
import tf
from math import sqrt
from ros_bge_bridge.srv import SetSpeed, SetAcceleration
from hri_framework.entity_proxy import EntityProxy


class ZoidsteinGazeServer(GazeActionServer):
    GAZE_ACTION_TOPIC = 'gaze'
    BGE_CONTROLLER_PREFIX = 'bge_neck_controller'
    GAZE_REACHED = 0.1

    def __init__(self):
        GazeActionServer.__init__(self, ZoidsteinGazeServer.GAZE_ACTION_TOPIC)
        self.tl = tf.TransformListener()
        self.gaze_target_pub = rospy.Publisher('zoidstein/gaze_target', PointStamped, queue_size=1) # TODO: confusing that this has a different name than the controller
        self.enable_srv = rospy.ServiceProxy(ZoidsteinGazeServer.BGE_CONTROLLER_PREFIX + '/enable', Empty)
        self.disable_srv = rospy.ServiceProxy(ZoidsteinGazeServer.BGE_CONTROLLER_PREFIX + '/disable', Empty)
        self.speed_srv = rospy.ServiceProxy(ZoidsteinGazeServer.BGE_CONTROLLER_PREFIX + '/set_speed', SetSpeed)
        self.accel_srv = rospy.ServiceProxy(ZoidsteinGazeServer.BGE_CONTROLLER_PREFIX + '/set_acceleration', SetAcceleration)

    def execute(self, gaze_goal):
        self.enable_srv()
        self.speed_srv(gaze_goal.speed)
        self.speed_srv(gaze_goal.acceleration)

        while not rospy.is_shutdown() and not self.action_server.is_preempt_requested() and self.action_server.is_active():
            entity = EntityProxy(gaze_goal.target)
            entity_tf_frame = entity.tf_frame()

            try:
                (target_trans, target_rot) = self.tl.lookupTransform(self.origin_frame, entity_tf_frame, rospy.Time(0))
                point_stamped = PointStamped()
                point_stamped.header.frame_id = self.origin_frame
                point_stamped.header.stamp = rospy.Time().now()
                point_stamped.point = Point(x=target_trans[0], y=target_trans[1], z=target_trans[2])
                self.gaze_target_pub.publish(point_stamped)

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            try:
                (curr_trans, curr_rot) = self.tl.lookupTransform(self.gaze_frame, entity_tf_frame, rospy.Time(0))
                y = curr_trans[1]
                z = curr_trans[2]
                distance_to_target = sqrt(y*y + z*z)
                rospy.loginfo('gaze_frame: {0}, entity_tf_frame: {1}, y: {2}, z: {3}, distance: {4}'.format(self.gaze_frame, entity_tf_frame, y, z, distance_to_target))
                self.send_feedback(distance_to_target) #TODO: make urdf so can receive feedback

                if distance_to_target < ZoidsteinGazeServer.GAZE_REACHED: #TODO: check min max motor positions because person could be out of bounds of gaze
                    self.action_server.set_succeeded()

            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue

            self.rate.sleep()

        self.disable_srv()

if __name__ == '__main__':
    rospy.init_node('zoidstein_gaze_node')
    server = ZoidsteinGazeServer()
    server.start()
    rospy.spin()