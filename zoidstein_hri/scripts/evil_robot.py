import rospy
import random
from hri_api.entities import Person, World, Saliency
from zoidstein_hri.zoidstein import Zoidstein, ZoidExpression
from hri_api.query import Query
import time

# Initialize objects
world = World()
robot = Zoidstein()
people = [Person(1), Person(2), Person(3)]

time.sleep(5)

robot.say("Hi, I'm Zoidstein! Hahahahahahaha")

i = 0

robot.show_expression(ZoidExpression.smile, 0.5)
robot.show_expression(ZoidExpression.frown, 1.0)

while i < 5:
    person = random.choice(people)
    robot.gaze_and_wait(person.head, speed=0.5)

    if i == 4:
        robot.say("Hahahahahahaha")
    i += 1


rospy.loginfo('hello')
robot.show_expression_and_wait(ZoidExpression.frown, 0.0) #TODO: show expression without waiting doesn't work
robot.show_expression_and_wait(ZoidExpression.smile, 0.0)


