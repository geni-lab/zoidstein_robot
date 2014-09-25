#!/usr/bin/env python
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
from hri_common import Expression
from enum import Enum

class AnimationType(Enum):
    BodyCon = 1
    Keyframe = 2

class Gesture(IGesture):

    NodHead = (3, "", AnimationType.Keyframe)
    LeftArm = (3, "/usr/bin/robot/scripts/MotorRel.sh 03 13 08", AnimationType.BodyCon)
    RightArm = (3, "/usr/bin/robot/scripts/MotorRel.sh 04 13 02", AnimationType.BodyCon)
    LeftHand = (3, "usr/bin/robot/scripts/MotorRel.sh 05 03 08", AnimationType.BodyCon)
    RightHand = (3, "/usr/bin/robot/scripts/MotorRel.sh 06 03 0a", AnimationType.BodyCon)
    BodyLeanForwardBackward = (3, "/usr/bin/robot/scripts/MotorRel.sh 07 03 08", AnimationType.BodyCon)
    BodyWaistTwist = (3, "/usr/bin/robot/scripts/MotorRel.sh 08 13 08", AnimationType.BodyCon)
    BodyTiltLeftRight = (3, "/usr/bin/robot/scripts/MotorRel.sh 09 03 08", AnimationType.BodyCon)
    LeftLegForwardBack = (3,  "/usr/bin/robot/scripts/MotorRel.sh 0a 03 08", AnimationType.BodyCon)
    RightLegForwardBack = (3, "/usr/bin/robot/scripts/MotorRel.sh 0b 03 08", AnimationType.BodyCon)
    WalkForward1 = (3, "/usr/bin/robot/scripts/BasicMove.sh 01 2", AnimationType.BodyCon)
    WalkForward2 = (3, "/usr/bin/robot/scripts/BasicMove.sh 02 5", AnimationType.BodyCon)
    WalkForward3 = (3, "/usr/bin/robot/scripts/BasicMove.sh 03 12", AnimationType.BodyCon)
    WalkReverse1 = (3, "/usr/bin/robot/scripts/BasicMove.sh 04 2", AnimationType.BodyCon)
    WalkReverse2 = (3, "/usr/bin/robot/scripts/BasicMove.sh 05 2", AnimationType.BodyCon)
    WalkRightTurnInPlace = (3, "/usr/bin/robot/scripts/BasicMove.sh 06 2", AnimationType.BodyCon)
    WalkLeftTurnInPlace = (3, "/usr/bin/robot/scripts/BasicMove.sh 07 2", AnimationType.BodyCon)
    WalkForward4 = (3, "/usr/bin/robot/scripts/BasicMove.sh 08 2", AnimationType.BodyCon)
    WalkForward5 = (3, "/usr/bin/robot/scripts/BasicMove.sh 09 2", AnimationType.BodyCon)
    RightArmDrop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 10", AnimationType.BodyCon)
    LeftArmDrop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 11", AnimationType.BodyCon)
    RightArmGive = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 12", AnimationType.BodyCon)
    LeftArmGive = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 13", AnimationType.BodyCon)
    RightArmGrab = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 14", AnimationType.BodyCon)
    LeftArmGrab = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 15", AnimationType.BodyCon)
    RightArmRoll = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 16", AnimationType.BodyCon)
    LeftArmRoll = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 17", AnimationType.BodyCon)
    RightArmThrow = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 18", AnimationType.BodyCon)
    LeftArmThrow = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 19", AnimationType.BodyCon)
    RightArmChop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1a", AnimationType.BodyCon)
    LeftArmChop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1b", AnimationType.BodyCon)
    RightFootKick = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1c", AnimationType.BodyCon)
    LeftFootKick = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1d", AnimationType.BodyCon)
    RightArmPush = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1e", AnimationType.BodyCon)
    LeftArmPush = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1f", AnimationType.BodyCon)
    Dance = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 20", AnimationType.BodyCon)
    LieDown = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 21", AnimationType.BodyCon)
    SitUp = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 22", AnimationType.BodyCon)
    StandUp = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 23", AnimationType.BodyCon)
    WaveHands = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 27", AnimationType.BodyCon)
    Flinch = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 28", AnimationType.BodyCon)
    Introduction = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2a", AnimationType.BodyCon)
    RightArmHighPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2b", AnimationType.BodyCon)
    leftArmHighPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2c", AnimationType.BodyCon)
    RightArmLowPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2d", AnimationType.BodyCon)
    LeftArmLowPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2e", AnimationType.BodyCon)
    DefaultPosition = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2f", AnimationType.BodyCon)
    DefaultReset = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 30", AnimationType.BodyCon)
    RightArmWave = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 39", AnimationType.BodyCon)
    RightHandShake = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 3a", AnimationType.BodyCon)
    LeftArmWave = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 3b", AnimationType.BodyCon)
    LeftHandShake = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 3c", AnimationType.BodyCon)

    def __init__(self, default_duration, bcon_script, animation_type):
        IGesture.__init__(self, default_duration)
        self.bcon_script = bcon_script
        self.animation_type = animation_type


class Zoidstein(Robot):
    def __init__(self):
        Robot.__init__(self, Gesture, Expression)