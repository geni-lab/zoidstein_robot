import rospy
from hri_api.entities import Person, World, Speed, Intensity
from zoidstein_hri.zoidstein import Zoidstein
from zoidstein_hri.zoidstein_expressions import ZoidsteinExpression

world = World()
robot = Zoidstein(1)
people = [Person(1), Person(2), Person(3)]

# # Test gaze
# for person in people:
#     robot.gaze_and_wait(person.head, speed=Speed.moderately)


robot.show_expression_and_wait(ZoidsteinExpression.smile, Intensity.strongly, Speed.quickly, 2.0)
robot.show_expression_and_wait(ZoidsteinExpression.frown_mouth, Intensity.strongly, Speed.quickly, 2.0)
robot.show_expression_and_wait(ZoidsteinExpression.open_mouth, Intensity.strongly, Speed.quickly, 2.0)
robot.show_expression_and_wait(ZoidsteinExpression.open_mouth, Intensity.nothing, Speed.quickly, 2.0)
robot.show_expression_and_wait(ZoidsteinExpression.frown, Intensity.mildly, Speed.quickly, 2.0)
robot.show_expression_and_wait(ZoidsteinExpression.raise_eyebrows, Intensity.strongly, Speed.quickly, 2.0)


