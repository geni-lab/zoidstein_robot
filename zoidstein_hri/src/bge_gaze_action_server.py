#!/usr/bin/env python
import roslib
roslib.load_manifest('hri_framework')
import rospy
from hri_msgs.msg import GazeAction, GazeActionFeedback
from hri_framework import GazeActionServer
from geometry_msgs.msg import PointStamped, Point
import tf
from math import sqrt


class BgeGazeActionServer(GazeActionServer):

    def __init__(self):
        rospy.init_node('gaze_action_server', anonymous=True)
        self.tl = tf.TransformListener()
        self.robot_name = rospy.get_param('~robot_name', 'robot')
        self.gaze_target_pub = rospy.Publisher(self.robot_name + '/gaze_target', PointStamped)
        self.origin_frame = rospy.get_param('~origin_frame', 'base_link')
        self.gaze_frame = rospy.get_param('~gaze_frame', 'gaze')
        self.rate = rospy.Rate(rospy.get_param('~hz'), 10)

    def run(self):
        while not rospy.is_shutdown():
            if self.gaze_goal is not None:
                entity = EntityProxy(self.gaze_goal.target)
                entity_tf_frame = entity.get_tf_frame()

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
                    self.send_feedback(distance_to_target)

                except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                    continue

                self.rate.sleep()

    def send_feedback(self, distance_to_target):
        """ Call this method to send feedback about the distance to the target """
        feedback = GazeActionFeedback()
        feedback.distance_to_target = distance_to_target
        self.action_server.publish_feedback(self.feedback)
        rospy.loginfo("Gaze feedback. distance_to_target: $s", distance_to_target)