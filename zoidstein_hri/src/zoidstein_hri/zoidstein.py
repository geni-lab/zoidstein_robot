# Copyright (c) 2014, OpenCog Foundation
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
#
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
#
# * Neither the name of the OpenCog Foundation nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

__author__ = 'Alex van der Peet, Jamie Diprose'

from hri_api.entities import Robot, IGesture


class Gesture(IGesture):

    LeftArm = 1
    RightArm = 2
    LeftHand = 3
    RightHand = 4
    BodyLeanForwardBackward = 5
    BodyWaistTwist = 6
    BodyTiltLeftRight = 7
    LeftLegForwardBack = 8
    RightLegForwardBack = 9
    WalkForward1 = 10
    WalkForward2 = 11
    WalkForward3 = 12
    WalkReverse1 = 13
    WalkReverse2 = 14
    WalkRightTurnInPlace = 15
    WalkLeftTurnInPlace = 16
    WalkForward4 = 17
    WalkForward5 = 18
    RightArmDrop = 19
    LeftArmDrop = 20
    RightArmGive = 21
    LeftArmGive = 22
    RightArmGrab = 23
    LeftArmGrab = 24
    RightArmRoll = 25
    LeftArmRoll = 26
    RightArmThrow = 27
    LeftArmThrow = 28
    RightArmChop = 29
    LeftArmChop = 30
    RightFootKick = 31
    LeftFootKick = 32
    RightArmPush = 33
    LeftArmPush = 34
    Dance = 35
    LieDown = 36
    SitUp = 37
    StandUp = 38
    WaveHands = 39
    Flinch = 40
    Introduction = 41
    RightArmHighPickup = 42
    leftArmHighPickup = 43
    RightArmLowPickup = 44
    LeftArmLowPickup = 45
    DefaultPosition = 46
    DefaultReset = 47
    RightArmWave = 48
    RightHandShake = 49
    LeftArmWave = 50
    LeftHandShake = 51

    def __init__(self, *args):
        IGesture.__init__(*args)

        self.data = [(Gesture.left_arm, "/usr/bin/robot/scripts/MotorRel.sh 03 13 08", 3),
        (Gesture.RightArm, "/usr/bin/robot/scripts/MotorRel.sh 04 13 02", 3),
        (Gesture.LeftHand, "usr/bin/robot/scripts/MotorRel.sh 05 03 08", 3),
        (Gesture.RightHand, "/usr/bin/robot/scripts/MotorRel.sh 06 03 0a", 3),
        (Gesture.BodyLeanForwardBackward, "/usr/bin/robot/scripts/MotorRel.sh 07 03 08", 3),
        (Gesture.BodyWaistTwist, "/usr/bin/robot/scripts/MotorRel.sh 08 13 08", 3),
        (Gesture.BodyTiltLeftRight, "/usr/bin/robot/scripts/MotorRel.sh 09 03 08", 3),
        (Gesture.LeftLegForwardBack,  "/usr/bin/robot/scripts/MotorRel.sh 0a 03 08", 3),
        (Gesture.RightLegForwardBack, "/usr/bin/robot/scripts/MotorRel.sh 0b 03 08", 3),
        (Gesture.WalkForward1, "/usr/bin/robot/scripts/BasicMove.sh 01 2", 3),
        (Gesture.WalkForward2, "/usr/bin/robot/scripts/BasicMove.sh 02 5", 3),
        (Gesture.WalkForward3, "/usr/bin/robot/scripts/BasicMove.sh 03 12", 3),
        (Gesture.WalkReverse1, "/usr/bin/robot/scripts/BasicMove.sh 04 2", 3),
        (Gesture.WalkReverse2, "/usr/bin/robot/scripts/BasicMove.sh 05 2", 3),
        (Gesture.WalkRightTurnInPlace, "/usr/bin/robot/scripts/BasicMove.sh 06 2", 3),
        (Gesture.WalkLeftTurnInPlace, "/usr/bin/robot/scripts/BasicMove.sh 07 2", 3),
        (Gesture.WalkForward4, "/usr/bin/robot/scripts/BasicMove.sh 08 2", 3),
        (Gesture.WalkForward5, "/usr/bin/robot/scripts/BasicMove.sh 09 2", 3),
        (Gesture.RightArmDrop, "/usr/bin/robot/scripts/DefaultBcon.sh 10", 3),
        (Gesture.LeftArmDrop, "/usr/bin/robot/scripts/DefaultBcon.sh 11", 3),
        (Gesture.RightArmGive, "/usr/bin/robot/scripts/DefaultBcon.sh 12", 3),
        (Gesture.LeftArmGive, "/usr/bin/robot/scripts/DefaultBcon.sh 13", 3),
        (Gesture.RightArmGrab, "/usr/bin/robot/scripts/DefaultBcon.sh 14", 3),
        (Gesture.LeftArmGrab, "/usr/bin/robot/scripts/DefaultBcon.sh 15", 3),
        (Gesture.RightArmRoll, "/usr/bin/robot/scripts/DefaultBcon.sh 16", 3),
        (Gesture.LeftArmRoll, "/usr/bin/robot/scripts/DefaultBcon.sh 17", 3),
        (Gesture.RightArmThrow, "/usr/bin/robot/scripts/DefaultBcon.sh 18", 3),
        (Gesture.LeftArmThrow, "/usr/bin/robot/scripts/DefaultBcon.sh 19", 3),
        (Gesture.RightArmChop, "/usr/bin/robot/scripts/DefaultBcon.sh 1a", 3),
        (Gesture.LeftArmChop, "/usr/bin/robot/scripts/DefaultBcon.sh 1b", 3),
        (Gesture.RightFootKick, "/usr/bin/robot/scripts/DefaultBcon.sh 1c", 3),
        (Gesture.LeftFootKick, "/usr/bin/robot/scripts/DefaultBcon.sh 1d", 3),
        (Gesture.RightArmPush, "/usr/bin/robot/scripts/DefaultBcon.sh 1e", 3),
        (Gesture.LeftArmPush, "/usr/bin/robot/scripts/DefaultBcon.sh 1f", 3),
        (Gesture.Dance, "/usr/bin/robot/scripts/DefaultBcon.sh 20", 3),
        (Gesture.LieDown, "/usr/bin/robot/scripts/DefaultBcon.sh 21", 3),
        (Gesture.SitUp, "/usr/bin/robot/scripts/DefaultBcon.sh 22", 3),
        (Gesture.StandUp, "/usr/bin/robot/scripts/DefaultBcon.sh 23", 3),
        (Gesture.WaveHands, "/usr/bin/robot/scripts/DefaultBcon.sh 27", 3),
        (Gesture.Flinch, "/usr/bin/robot/scripts/DefaultBcon.sh 28", 3),
        (Gesture.Introduction, "/usr/bin/robot/scripts/DefaultBcon.sh 2a", 3),
        (Gesture.RightArmHighPickup, "/usr/bin/robot/scripts/DefaultBcon.sh 2b", 3),
        (Gesture.leftArmHighPickup, "/usr/bin/robot/scripts/DefaultBcon.sh 2c", 3),
        (Gesture.RightArmLowPickup, "/usr/bin/robot/scripts/DefaultBcon.sh 2d", 3),
        (Gesture.LeftArmLowPickup, "/usr/bin/robot/scripts/DefaultBcon.sh 2e", 3),
        (Gesture.DefaultPosition, "/usr/bin/robot/scripts/DefaultBcon.sh 2f", 3),
        (Gesture.DefaultReset, "/usr/bin/robot/scripts/DefaultBcon.sh 30", 3),
        (Gesture.RightArmWave, "/usr/bin/robot/scripts/DefaultBcon.sh 39", 3),
        (Gesture.RightHandShake, "/usr/bin/robot/scripts/DefaultBcon.sh 3a", 3),
        (Gesture.LeftArmWave, "/usr/bin/robot/scripts/DefaultBcon.sh 3b", 3),
        (Gesture.LeftHandShake, "/usr/bin/robot/scripts/DefaultBcon.sh 3c", 3)]

    def bodycon_script(self):
        gesture_data = self.get_data()
        return gesture_data[1]

    def default_duration(self):
        gesture_data = self.get_data()
        return gesture_data[2]


class Zoidstein(Robot):
    def __init__(self):
        Robot.__init__(self, Expression, Gesture)