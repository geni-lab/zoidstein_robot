from scipy.interpolate.dfitpack import regrid_smth

__author__ = 'jamie'

import serial
import rospy

class RSMSerialNode:

    def __init__(self):
        self.serialPort = serial.Serial('/dev/ttyUSB0', 115200)

    def testSerial(self):
        self.serialPort.write('usr/bin/robot/scripts/DefaultBcon.sh 10\n')
        # x = self.serialPort.read()
        # s = self.serialPort.read(10)
        # line = self.serialPort.readline()
        #self.serialPort.close()

    def executeScript(self, script, duration):
        #self.serialPort = serial.Serial('/dev/ttyUSB0', 115200)
        self.serialPort.write(script + "\n")
        #self.serialPort.close()

    def close(self):
        self.serialPort.close()

if __name__ == '__main__':
    rsm_serial_node = RSMSerialNode()
    rsm_serial_node.testSerial()

