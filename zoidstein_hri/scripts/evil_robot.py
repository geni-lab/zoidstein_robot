#!/usr/bin/env python
import rospy
import random
from hri_api.entities import Person, World, Saliency
from zoidstein_hri.zoidstein import Zoidstein, Expression, ZoidGestureData
from hri_api.query import Query
import time

# Initialize objects
world = World()
robot = Zoidstein()
people = [Person(1), Person(2), Person(3)]

bob = Person(4)


time.sleep(5)

#robot.say("Hi, I'm Zoidstein! Hahahahahahaha")

i = 0

robot.expression(Expression.smile, 1.0)

time.sleep(1)

# robot.show_expression(ZoidExpression.smile, 1.0)
#
# time.sleep(1)
#
# robot.show_expression(ZoidExpression.frown, 0.0)
#
# time.sleep(1)
#
# robot.show_expression(ZoidExpression.smile, 1.0)
#
# time.sleep(1)
#
# robot.show_expression(ZoidExpression.frown, 0.0)
#
# time.sleep(1)

robot.gesture(ZoidGestureData.HeadUpDown)

robot.expression( Expression.smile, 1.0)
time.sleep(2)
while i < 5000:
    person = random.choice(people)
    robot.gaze_and_wait(person.head, speed=0.5)

    wait_time = random.randrange(1, 5)

    if (wait_time == 4) or (i % 10 == 0):
        wait_time=4
        utterance = random.randrange(1, 5)

        if (utterance == 1):
            robot.expression(Expression.smile,1.0)
            robot.say("I sound like a woman. Give me a man's voice.") # Haha! Good going :D
        elif (utterance == 2):
            robot.expression(Expression.frown)
            robot.say("No more I love you")
        elif (utterance == 3):
            robot.expression(Expression.smile)
            robot.say("I'll only love you till the money comes")
        else:
            robot.expression(Expression.frown)
            robot.say("I don't think I like you better")



    time.sleep(wait_time)

    # robot.show_expression(ZoidExpression.frown, 1.0)
    #
    # time.sleep(1)
    #
    # # robot.show_expression(ZoidExpression.frown_mouth, 1.0)
    # #
    # # time.sleep(1)
    #
    # robot.show_expression(ZoidExpression.smile, 1.0)
    #
    # time.sleep(1)
    #
    # robot.show_expression(ZoidExpression.open_mouth, 1.0)
    #
    # time.sleep(1)


    i += 1


rospy.loginfo('hello')
robot.show_expression_and_wait(Expression.frown, 0.0) #TODO: show expression without waiting doesn't work
robot.show_expression_and_wait(Expression.smile, 0.0)


