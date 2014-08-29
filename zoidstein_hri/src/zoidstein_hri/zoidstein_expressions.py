from hri_api.entities import Robot
from hri_api.entities import Expression


class ZoidsteinExpression(Expression):
    def __init__(*args):
        Expression.__init__(*args)

    smile = 1
    frown_mouth = 2
    frown = 3
    raise_eyebrows = 4
    open_mouth = 5