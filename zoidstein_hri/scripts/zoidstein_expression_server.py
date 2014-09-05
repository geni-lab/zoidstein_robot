#!/usr/bin/env python
import rospy
from hri_msgs.msg import ExpressionAction, ExpressionResult
from hri_framework.multi_goal_action_server import MultiGoalActionServer
from zoidstein_hri import ZoidExpression
import threading
from ros_pololu_servo.msg import MotorCommand
from ros_pololu_servo.srv import MotorRange
from hri_api.entities import Speed, Intensity
import time


def clamp(self, value, min_value, max_value):
    return max(min(value, max_value), min_value)

def interpolate(value, old_min, old_max, new_min, new_max):
    # Width of each range
    old_range = old_max - old_min
    new_range = new_max - new_min

    # Scale old value into range between 0 and 1
    scaled_value = (value - old_min) / old_range

    # Convert the scaled value into the new range
    new_val = new_min + (scaled_value * new_range)

    return new_val


class ZoidsteinExpressionServer(object):

    def __init__(self):
        self.action_server = MultiGoalActionServer('expression', ExpressionAction, auto_start=False)
        self.action_server.register_goal_callback(self.goal_callback)
        self.motor_pub = rospy.Publisher('pololu/command', MotorCommand, queue_size=10)
        self.motor_range_srv = rospy.ServiceProxy('pololu/motor_range', MotorRange)
        self.rate = rospy.Rate(10)

    def start(self):
        self.action_server.start()

    def goal_callback(self, goal_handle):
        #self.action_server.set_accepted(goal_handle)
        new_goal = goal_handle.get_goal()
        rospy.loginfo('Accepted new goal: {0}'.format(new_goal))

        found = False
        for name, member in ZoidExpression.__members__.items():
            if name == new_goal.expression:
                found = True

        if not found:
            rospy.logerr('{0} is not a valid expression. Valid expressions are: {1}'.format(new_goal.expression, ZoidExpression))
            self.action_server.set_aborted(goal_handle)
            return

        expression_thread = threading.Thread(target=self.start_expression, args=[goal_handle])
        expression_thread.run()

    def start_expression(self, goal_handle):
        goal = goal_handle.get_goal()

        expression = ZoidExpression[goal.expression]
        speed = goal.speed
        intensity = goal.intensity
        duration = goal.duration

        if expression in [ZoidExpression.smile, ZoidExpression.frown_mouth]:
            joint_name = 'smile_joint'
        elif expression in [ZoidExpression.open_mouth]:
            joint_name = 'jaw_joint'
        elif expression in [ZoidExpression.frown]:
            joint_name = 'brow_joint'

        negative = True
        if expression in [ZoidExpression.smile, ZoidExpression.open_mouth]:
            negative = False

        response = self.motor_range_srv(joint_name)

        msg = MotorCommand()
        msg.joint_name = joint_name

        min_range = min([response.min, response.max])
        max_range = max([response.min, response.max])

        if negative:
            msg.position = interpolate(intensity, 0.0, 1.0, 0.0, min_range)
        else:
            msg.position = interpolate(intensity, 0.0, 1.0, 0.0, max_range)

        msg.speed = interpolate(speed, 0.0, 1.0, 0.0, 0.5)
        msg.acceleration = interpolate(speed, 0.0, 1.0, 0.0, 0.4)

        start = time.time()
        while not rospy.is_shutdown() and not self.action_server.is_preempt_requested(goal_handle) and (time.time() - start) < duration:
            self.motor_pub.publish(msg)
            self.rate.sleep()

        if not self.action_server.is_preempt_requested(goal_handle):
            self.action_server.set_succeeded(goal_handle)


if __name__ == '__main__':
    rospy.init_node('zoidstein_expression_node')
    server = ZoidsteinExpressionServer()
    server.start()
    rospy.spin()
