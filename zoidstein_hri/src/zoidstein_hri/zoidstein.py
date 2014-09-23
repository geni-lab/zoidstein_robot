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


class Gesture(IGesture):

    LeftArm = (3, "/usr/bin/robot/scripts/MotorRel.sh 03 13 08")
    RightArm = (3, "/usr/bin/robot/scripts/MotorRel.sh 04 13 02")
    LeftHand = (3, "usr/bin/robot/scripts/MotorRel.sh 05 03 08")
    RightHand = (3, "/usr/bin/robot/scripts/MotorRel.sh 06 03 0a")
    BodyLeanForwardBackward = (3, "/usr/bin/robot/scripts/MotorRel.sh 07 03 08")
    BodyWaistTwist = (3, "/usr/bin/robot/scripts/MotorRel.sh 08 13 08")
    BodyTiltLeftRight = (3, "/usr/bin/robot/scripts/MotorRel.sh 09 03 08")
    LeftLegForwardBack = (3,  "/usr/bin/robot/scripts/MotorRel.sh 0a 03 08")
    RightLegForwardBack = (3, "/usr/bin/robot/scripts/MotorRel.sh 0b 03 08")
    WalkForward1 = (3, "/usr/bin/robot/scripts/BasicMove.sh 01 2")
    WalkForward2 = (3, "/usr/bin/robot/scripts/BasicMove.sh 02 5")
    WalkForward3 = (3, "/usr/bin/robot/scripts/BasicMove.sh 03 12")
    WalkReverse1 = (3, "/usr/bin/robot/scripts/BasicMove.sh 04 2")
    WalkReverse2 = (3, "/usr/bin/robot/scripts/BasicMove.sh 05 2")
    WalkRightTurnInPlace = (3, "/usr/bin/robot/scripts/BasicMove.sh 06 2")
    WalkLeftTurnInPlace = (3, "/usr/bin/robot/scripts/BasicMove.sh 07 2")
    WalkForward4 = (3, "/usr/bin/robot/scripts/BasicMove.sh 08 2")
    WalkForward5 = (3, "/usr/bin/robot/scripts/BasicMove.sh 09 2")
    RightArmDrop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 10")
    LeftArmDrop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 11")
    RightArmGive = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 12")
    LeftArmGive = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 13")
    RightArmGrab = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 14")
    LeftArmGrab = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 15")
    RightArmRoll = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 16")
    LeftArmRoll = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 17")
    RightArmThrow = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 18")
    LeftArmThrow = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 19")
    RightArmChop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1a")
    LeftArmChop = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1b")
    RightFootKick = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1c")
    LeftFootKick = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1d")
    RightArmPush = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1e")
    LeftArmPush = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 1f")
    Dance = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 20")
    LieDown = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 21")
    SitUp = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 22")
    StandUp = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 23")
    WaveHands = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 27")
    Flinch = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 28")
    Introduction = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2a")
    RightArmHighPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2b")
    leftArmHighPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2c")
    RightArmLowPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2d")
    LeftArmLowPickup = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2e")
    DefaultPosition = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 2f")
    DefaultReset = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 30")
    RightArmWave = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 39")
    RightHandShake = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 3a")
    LeftArmWave = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 3b")
    LeftHandShake = (3, "/usr/bin/robot/scripts/DefaultBcon.sh 3c")

    def __init__(self, default_duration, bcon_script):
        IGesture.__init__(self, default_duration)
        self.bcon_script = bcon_script


class Zoidstein(Robot):
    def __init__(self):
        Robot.__init__(self, 3, Expression)