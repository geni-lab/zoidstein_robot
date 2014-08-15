#!/usr/bin/env python
import ros_pololu_servo

__author__ = 'Alex van der Peet'

import rospy
from sensor_msgs.msg import JointState
from threading import Thread
from ros_pololu_servo.msg import pololu_info, motor

class ZoidsteinJointStatePublisher(Thread):
    NODE_NAME = 'zoidstein_robot_joint_state_publisher'

    def __init__(self):
        Thread.__init__(self)
        # Init node
        rospy.init_node(ZoidsteinJointStatePublisher.NODE_NAME, log_level=rospy.INFO)
        # Configure publishing rate
        self.rate = rospy.Rate(rospy.get_param('~sensor_rate', 15.0))
        self.base_frame_id = rospy.get_param('~base_frame_id', "base_link")

        # Initialize publisher
        self.joint_state_pub = rospy.Publisher("joint_states", JointState, queue_size=None)
        # Initialize JointState message
        self.joint_state = JointState()
        self.joint_state.header.frame_id = self.base_frame_id

        # Subscribe to joint state messages for each joint
        self.joint_names = []
        self.joint_positions = []

        # Subscribe to pololu_info topic that publishes servo names and positions.
        rospy.Subscriber("pololu_info", ros_pololu_servo.msg.pololu_info, self.update_joint_states)

    def update_joint_states(self, msg):
        # Reset names and positions
        self.joint_names = []
        self.joint_positions = []

        # Redefine names and positions
        for motor in msg.info:
            self.joint_names.append(motor.name + "_joint")
            self.joint_positions.append(motor.position)

    def run(self):
        while not rospy.is_shutdown():
            self.joint_state.header.stamp = rospy.Time.now()
            # rospy.loginfo("JOINT NAMES: " + str(self.joint_names))
            # rospy.loginfo("JOINT VELOCITIES: " + str(self.joint_velocities))

            # Complete JointState message
            self.joint_state.name = list(self.joint_names)
            self.joint_state.position = list(self.joint_positions)
            #self.joint_state.velocity = list(self.joint_velocities)

            # Publish JointState messags
            self.joint_state_pub.publish(self.joint_state)
            self.rate.sleep()

if __name__ == '__main__':
    rospy.loginfo("Starting {0}...".format(ZoidsteinJointStatePublisher.NODE_NAME))
    joint_pub = ZoidsteinJointStatePublisher()

    joint_pub.start()
    rospy.loginfo("{0} started".format(ZoidsteinJointStatePublisher.NODE_NAME))

    rospy.spin()

    rospy.loginfo("Stopping {0}...".format(ZoidsteinJointStatePublisher.NODE_NAME))
    joint_pub.join()
    rospy.loginfo("{0} stopped.".format(ZoidsteinJointStatePublisher.NODE_NAME))
