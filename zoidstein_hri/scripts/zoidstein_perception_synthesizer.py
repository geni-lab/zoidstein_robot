#!/usr/bin/env python
import rospy
from hri_framework import PerceptionSynthesizer, SingleEntitySource
from geometry_msgs.msg import PointStamped

if __name__ == '__main__':
    rospy.init_node('zoidstein_perception_synthesizer')
    ps = PerceptionSynthesizer()
    ps.add_entity_source(SingleEntitySource('salient_point_3d', PointStamped, 'saliency'))
    ps.start()
    rospy.spin()
