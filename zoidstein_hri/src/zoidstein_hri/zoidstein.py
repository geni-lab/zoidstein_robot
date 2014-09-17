__author__ = 'Jamie Diprose'

from hri_api.entities import Robot
from hri_api.entities import Expression
from hri_api.entities import Gesture


class ZoidExpression(Expression):
    def __init__(*args):
        Expression.__init__(*args)

    smile = 1
    frown_mouth = 2
    frown = 3
    open_mouth = 4


class Zoidstein(Robot):
    def __init__(self):
        Robot.__init__(self, ZoidExpression, ZoidGesture)


class ZoidGesture(Gesture):
    def __init__(*args):
        Gesture.__init__(*args)

    HeadUpDown = 1001
    HeadLeftRight = 1002
    LeftArm = 1003
    RightArm = 1004
    LeftHand = 1005
    RightHand = 1006
    BodyLeanForwardBackward = 1007
    BodyWaistTwist = 1008
    BodyTiltLeftRight = 1009
    LeftLegForwardBack = 1010
    RightLegForwardBack = 1011

    WalkForward1 = 2001
    WalkForward2 = 2002
    WalkForward3 = 2003
    WalkReverse1 = 2004
    WalkReverse2 = 2005
    WalkRightTurnInPlace = 2006
    WalkLeftTurnInPlace = 2007
    WalkForward4 = 2008
    WalkForward5 = 2009

    TrackBlue = 3001
    TrackGreen = 3002
    TrackHuman = 3003
    TrackRed = 3004
    ClearScreen = 3005

    BConRightArmDrop = 4010
    BConLeftArmDrop = 4011
    BConRightArmGive = 4012
    BConLeftArmGive = 4013
    BConRightArmGrab = 4014
    BConLeftArmGrab = 4015
    BConRightArmRoll = 4016
    BConLeftArmRoll = 4017
    BConRightArmThrow = 4018
    BConLeftArmThrow = 4019
    BConRightArmChop = 40110
    BConLeftArmChop = 40111
    BConRightFootKick = 40112
    BConLeftFootKick = 40113
    BConRightArmPush = 40114
    BConLeftArmPush = 40115
    BConDance = 4020
    BConLieDown = 4021
    BConSitUp = 4022
    BConStandUp = 4023
    BConRS2Interact = 4024
    BConRoboraptorInteract = 4025
    BConRobopetInteract = 4026
    BConWaveHands = 4027
    BConFlinch = 4028
    BConIntroduction = 40210
    BConRightArmHighPickup = 40211
    BConleftArmHighPickup = 40212
    BConRightArmLowPickup = 40213
    BConLeftArmLowPickup = 40214
    BConDefaultPosition = 40215
    BConDefaultReset = 4030
    BConPuppetModeProgram = 4032
    BConPuppetModePlayback = 4033
    BConMainProgramMode = 4034
    BConVisionProgramMode = 4035
    BConSoundProgramMode = 4036
    BConGuardMode = 4037
    BConMainProgramPlayback = 4038
    BConRightArmWave = 4039
    BConRightHandShake = 40310
    BConLeftArmWave = 40311
    BConLeftHandShake = 40312
    BConRoboReptileInteract = 40313



