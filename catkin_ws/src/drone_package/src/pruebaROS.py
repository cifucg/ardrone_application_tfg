#!/usr/bin/env python

from droneController import DroneController
import rospy
import sys

class pruebaROS:

    def __init__(self):
        self.droneController = DroneController()

    def sendTest(self):
        self.droneController.testTakeoffLand()

def main(args):
    rospy.init_node('pruebaROS', anonymous=True)
    prueba = pruebaROS()
    prueba.sendTest()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print ('sdsdsad')

if __name__ == '__main__':
    main(sys.argv)