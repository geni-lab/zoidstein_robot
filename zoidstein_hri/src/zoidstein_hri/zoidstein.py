__author__ = 'Jamie Diprose'

from hri_api.entities import Robot
from hri_api.entities import Expression


class ZoidExpression(Expression):
    def __init__(*args):
        Expression.__init__(*args)

    smile = 1
    frown_mouth = 2
    frown = 3
    open_mouth = 4


class Zoidstein(Robot):

    def __init__(self):
        Robot.__init__(self, 1)



