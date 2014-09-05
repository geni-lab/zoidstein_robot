#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32
from ros_pololu_servo.srv import MotorRange
from ros_pololu_servo.msg import MotorCommand


def interpolate(value, old_min, old_max, new_min, new_max):
    # Width of each range
    old_range = old_max - old_min
    new_range = new_max - new_min

    # Scale old value into range between 0 and 1
    scaled_value = (value - old_min) / old_range

    # Convert the scaled value into the new range
    new_val = new_min + (scaled_value * new_range)

    return new_val


class LipSync():

    def __init__(self):
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=1)
        self.motor_range_srv = rospy.ServiceProxy('pololu/motor_range', MotorRange)

        # Setup motor command
        self.cmd = MotorCommand()
        self.cmd.joint_name = 'jaw_joint'
        self.cmd.speed = 1.0
        self.cmd.acceleration = 1.0

        # Get motor range
        rospy.loginfo('Waiting for pololu/motor_range service...')
        self.motor_range_srv.wait_for_service()
        rospy.loginfo('Found')

        response = self.motor_range_srv(self.cmd.joint_name)
        self.max_range = max([response.min, response.max])
        self.sub = rospy.Subscriber('speech_strength', Float32, self.speech_strength_callback, queue_size=1)

    def speech_strength_callback(self, msg):
        strength = msg.data
        self.cmd.position = interpolate(strength, 0.0, 1.0, 0.0, self.max_range)
        self.motor_pub.publish(self.cmd)


if __name__ == '__main__':
    rospy.init_node('zoidstein_lip_sync')
    lip_sync = LipSync()
    rospy.spin()
