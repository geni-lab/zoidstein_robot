#from reportlab.graphics.charts.piecharts import PL

__author__ = 'Jamie Diprose'

from hri_api.entities import Robot
from hri_api.entities import Expression
from hri_api.entities import Gesture
from enum import Enum


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
        
class ZoidGestureData(Enum):
    HeadUpDown = (1001, "/usr/bin/robot/scripts/MotorRel.sh 01 03 08", 3)
    HeadLeftRight = (1002, "/usr/bin/robot/scripts/MotorRel.sh 02 03 20", 3)
    LeftArm = (1003, "/usr/bin/robot/scripts/MotorRel.sh 03 13 08", 3)
    RightArm = (1004, "/usr/bin/robot/scripts/MotorRel.sh 04 13 02", 3)
    LeftHand = (1005, " 	/usr/bin/robot/scripts/MotorRel.sh 05 03 08", 3)
    RightHand = (1006, "/usr/bin/robot/scripts/MotorRel.sh 06 03 0a", 3)
    BodyLeanForwardBackward = (1007, "/usr/bin/robot/scripts/MotorRel.sh 07 03 08", 3)
    BodyWaistTwist = (1008, "/usr/bin/robot/scripts/MotorRel.sh 08 13 08", 3)
    BodyTiltLeftRight = (1009, "/usr/bin/robot/scripts/MotorRel.sh 09 03 08", 3)
    LeftLegForwardBack = (1010, "/usr/bin/robot/scripts/MotorRel.sh 0a 03 08", 3)
    RightLegForwardBack = (1011, "/usr/bin/robot/scripts/MotorRel.sh 0b 03 08", 3)

    WalkForward1 = (2001, "/usr/bin/robot/scripts/BasicMove.sh 01 2", 3)
    WalkForward2 = (2002, "/usr/bin/robot/scripts/BasicMove.sh 02 5", 3)
    WalkForward3 = (2003, "/usr/bin/robot/scripts/BasicMove.sh 03 12", 3)
    WalkReverse1 = (2004, "/usr/bin/robot/scripts/BasicMove.sh 04 2", 3)
    WalkReverse2 = (2005, "/usr/bin/robot/scripts/BasicMove.sh 05 2", 3)
    WalkRightTurnInPlace = (2006, "/usr/bin/robot/scripts/BasicMove.sh 06 2", 3)
    WalkLeftTurnInPlace = (2007, "/usr/bin/robot/scripts/BasicMove.sh 07 2", 3)
    WalkForward4 = (2008, "/usr/bin/robot/scripts/BasicMove.sh 08 2", 3)
    WalkForward5 = (2009, "/usr/bin/robot/scripts/BasicMove.sh 09 2", 3)

    TrackBlue = (3001, "/usr/bin/robot/scripts/track_color.sh blue ?", 3)
    TrackGreen = (3002, "/usr/bin/robot/scripts/track_color.sh green ?", 3)
    TrackHuman = (3003, "/usr/bin/robot/scripts/track_color.sh human ?", 3)
    TrackRed = (3004, "/usr/bin/robot/scripts/track_color.sh red ?", 3)
    ClearScreen = (3005, "/usr/bin/robot/scripts/clear_screen.sh", 3)

    BConRightArmDrop = (4010, "/usr/bin/robot/scripts/DefaultBcon.sh 10", 3)
    BConLeftArmDrop = (4011, "/usr/bin/robot/scripts/DefaultBcon.sh 11", 3)
    BConRightArmGive = (4012, "/usr/bin/robot/scripts/DefaultBcon.sh 12", 3)
    BConLeftArmGive = (4013, "/usr/bin/robot/scripts/DefaultBcon.sh 13", 3)
    BConRightArmGrab = (4014, "/usr/bin/robot/scripts/DefaultBcon.sh 14", 3)
    BConLeftArmGrab = (4015, "/usr/bin/robot/scripts/DefaultBcon.sh 15", 3)
    BConRightArmRoll = (4016, "/usr/bin/robot/scripts/DefaultBcon.sh 16", 3)
    BConLeftArmRoll = (4017, "/usr/bin/robot/scripts/DefaultBcon.sh 17", 3)
    BConRightArmThrow = (4018, "/usr/bin/robot/scripts/DefaultBcon.sh 18", 3)
    BConLeftArmThrow = (4019, "/usr/bin/robot/scripts/DefaultBcon.sh 19", 3)
    BConRightArmChop = (40110, "/usr/bin/robot/scripts/DefaultBcon.sh 1a", 3)
    BConLeftArmChop = (40111, "/usr/bin/robot/scripts/DefaultBcon.sh 1b", 3)
    BConRightFootKick = (40112, "/usr/bin/robot/scripts/DefaultBcon.sh 1c", 3)
    BConLeftFootKick = (40113, "/usr/bin/robot/scripts/DefaultBcon.sh 1d", 3)
    BConRightArmPush = (40114, "/usr/bin/robot/scripts/DefaultBcon.sh 1e", 3)
    BConLeftArmPush = (40115, "/usr/bin/robot/scripts/DefaultBcon.sh 1f", 3)
    BConDance = (4020, " 	/usr/bin/robot/scripts/DefaultBcon.sh 20", 3)
    BConLieDown = (4021, "/usr/bin/robot/scripts/DefaultBcon.sh 21", 3)
    BConSitUp = (4022, "/usr/bin/robot/scripts/DefaultBcon.sh 22", 3)
    BConStandUp = (4023, "/usr/bin/robot/scripts/DefaultBcon.sh 23", 3)
    BConRS2Interact = (4024, "/usr/bin/robot/scripts/DefaultBcon.sh 24", 3)
    BConRoboraptorInteract = (4025, "/usr/bin/robot/scripts/DefaultBcon.sh 25", 3)
    BConRobopetInteract = (4026, "/usr/bin/robot/scripts/DefaultBcon.sh 26", 3)
    BConWaveHands = (4027, "/usr/bin/robot/scripts/DefaultBcon.sh 27    ", 3)
    BConFlinch = (4028, "/usr/bin/robot/scripts/DefaultBcon.sh 28", 3)
    BConIntroduction = (40210, "/usr/bin/robot/scripts/DefaultBcon.sh 2a", 3)
    BConRightArmHighPickup = (40211, "/usr/bin/robot/scripts/DefaultBcon.sh 2b", 3)
    BConleftArmHighPickup = (40212, "/usr/bin/robot/scripts/DefaultBcon.sh 2c", 3)
    BConRightArmLowPickup = (40213, "/usr/bin/robot/scripts/DefaultBcon.sh 2d", 3)
    BConLeftArmLowPickup = (40214, "/usr/bin/robot/scripts/DefaultBcon.sh 2e", 3)
    BConDefaultPosition = (40215, "/usr/bin/robot/scripts/DefaultBcon.sh 2f", 3)
    BConDefaultReset = (4030, "/usr/bin/robot/scripts/DefaultBcon.sh 30", 3)
    BConPuppetModeProgram = (4032, "/usr/bin/robot/scripts/DefaultBcon.sh 32", 3)
    BConPuppetModePlayback = (4033, "/usr/bin/robot/scripts/DefaultBcon.sh 33", 3)
    BConMainProgramMode = (4034, "/usr/bin/robot/scripts/DefaultBcon.sh 34", 3)
    BConVisionProgramMode = (4035, "/usr/bin/robot/scripts/DefaultBcon.sh 35", 3)
    BConSoundProgramMode = (4036, "/usr/bin/robot/scripts/DefaultBcon.sh 36", 3)
    BConGuardMode = (4037, "/usr/bin/robot/scripts/DefaultBcon.sh 37", 3)
    BConMainProgramPlayback = (4038, "/usr/bin/robot/scripts/DefaultBcon.sh 38", 3)
    BConRightArmWave = (4039, "/usr/bin/robot/scripts/DefaultBcon.sh 39", 3)
    BConRightHandShake = (40310, "/usr/bin/robot/scripts/DefaultBcon.sh 3a", 3)
    BConLeftArmWave = (40311, "/usr/bin/robot/scripts/DefaultBcon.sh 3b", 3)
    BConLeftHandShake = (40312, "/usr/bin/robot/scripts/DefaultBcon.sh 3c", 3)
    BConRoboReptileInteract = (40313, "/usr/bin/robot/scripts/DefaultBcon.sh 3d", 3)

    def __init__(self, code, script, duration):
        self.code = code
        self.script = script
        self.duration = duration

    def get_name(self):
        return self.name;

    def get_code(self):
        return self.code;

    def get_script(self):
        return self.script;

    def get_duration(self):
        return self.duration;

class ZoidGesture(Gesture):

    # def get_duration(self, gestureData):
    #     if (self.action_dictionary.has_key(gesture)):
    #         return self.action_dictionary.get(gesture)[1]
    #     else:
    #         return 3
    #
    # def get_shell_command(self, gestureData):
    #     if (self.action_dictionary.has_key(gesture)):
    #         return self.action_dictionary.get(gesture)[0]
    #     else:
    #         return 3

    def __init__(self, *args):
        Gesture.__init__(*args)
        # self.init_dictionary()

    


