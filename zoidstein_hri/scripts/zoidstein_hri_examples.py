from hri_api.entities import Person, World, Saliency
from zoidstein_hri.zoidstein import Zoidstein, ZoidExpression
from hri_api.query import Query

# Initialize objects
world = World()
robot = Zoidstein()
people = [Person(1), Person(2), Person(3)]

# Test gazing
for person in people:
    robot.gaze_and_wait(person.head, speed=0.5)

# Test expressions
robot.show_expression_and_wait(ZoidExpression.smile, 1.0)
robot.show_expression_and_wait(ZoidExpression.smile, 0.0)

robot.show_expression_and_wait(ZoidExpression.frown_mouth, 1.0)
robot.show_expression_and_wait(ZoidExpression.frown_mouth, 0.0)

robot.show_expression_and_wait(ZoidExpression.frown, 1.0)
robot.show_expression_and_wait(ZoidExpression.frown, 0.0)

robot.show_expression_and_wait(ZoidExpression.open_mouth, 1.0)
robot.show_expression_and_wait(ZoidExpression.open_mouth, 0.0)

# Test speaking
robot.say("Hi, I'm Zoidstein! Hahahahahahaha")


# robot.gaze_and_wait(saliency)
#world.add_entity_class(Saliency, 'saliency') #TODO: initialize from same place as entity source, e.g. config file, and make it a service
#saliency = Query(world).of_type(Saliency)