#!/usr/bin/env python

# This file contains the implementation that we use to known the information that the drone have in every moment
# and to send the different commands into the drone to move around

# Import the ros files necessary
import rospy
import sys

# Import the different messages that we need to sending and receiving information to the drone
from ardrone_autonomy.msg import Navdata # Its necessary to receive all navdata of drone
from std_msgs.msg import Empty # Its necessary to send message to land, takeoff or emergency
from geometry_msgs.msg import Twist # Its necessary to sending commands to move the drone
from nav_msgs.msg import Odometry #Its necessary to send odometry

from droneStatus import DroneStatus

MAX_ALTITUDE = 1500 #mm
MIN_ALTITUDE = 500 #mm

class DroneController(object):
    def __init__(self, args):
        # Init the drone parameters
        self.status = -1
        self.navdata = None
        self.battery = 0
        self.altitude = 0
        self.odometry = None

        # Subscribe to the /ardrone/navdata topic with the type of message and the callback method
        self.subNavData = rospy.Subscriber('/ardrone/navdata', Navdata, self.NavData)

        if (args['environment'] == 'real'):
            # Subscribe to the /ardrone/odometry topic with the type of message at the real dron
            self.subOdometry = rospy.Subscriber('/ardrone/odometry', Odometry, self.OdomData)
        elif (args['environment'] == 'simulated'):
            # Subscribe to the /ground_truth/state topic with the type of message at  the simulate dron in gazebo
            self.subOdometry = rospy.Subscriber('/ground_truth/state', Odometry, self.OdomData)

        # Create publishers to send commands into this topics
        self.pubTakeoff = rospy.Publisher('/ardrone/takeoff', Empty, queue_size=10)
        self.pubLand = rospy.Publisher('/ardrone/land', Empty, queue_size=10)
        self.pubReset = rospy.Publisher('/ardrone/reset', Empty, queue_size=10)

        self.pubCommand = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.command = Twist()

        # If we are shutting down the drone, send land command
        rospy.on_shutdown(self.Land)

    def NavData(self, navdata):
        self.status = navdata.state
        self.navdata = navdata

    def OdomData(self, odomdata):
        self.odometry = odomdata

    def TakeOff(self):
        if (self.status == DroneStatus.Landed):
            self.pubTakeoff.publish(Empty())

    def Land(self):
        self.pubLand.publish(Empty())

    def Emergency(self):
        self.pubReset.publish(Empty())

    def SetCommand(self, roll, pitch, yaw_velocity, z_velocity):
        if self.navdata.batteryPercent > 10:
            self.command.linear.x = roll
            self.command.linear.y = pitch
            if (self.navdata.altd > MAX_ALTITUDE):
                print ('Down for safety!')
                self.command.linear.z = -0.20
            elif (self.navdata.altd < MIN_ALTITUDE):
                print('Up for safety!')
                self.command.linear.z = 0.20
            else:
                self.command.linear.z = z_velocity
            self.command.angular.z = yaw_velocity
            self.SendCommand()
        else:
            print('The dron will be landed for battery security')
            self.command.linear.x = 0
            self.command.linear.y = 0
            self.command.linear.z = 0
            self.SendCommand()
            rospy.sleep(2.)
            self.Land()
            rospy.signal_shutdown('Quit')


    def SendCommand(self):
        if self.status == DroneStatus.Flying or self.status == DroneStatus.GoToHover or self.status == DroneStatus.Hovering:
            self.pubCommand.publish(self.command)

    def testTakeoffLand(self):
        print('Takeoff')
        while not rospy.is_shutdown():
            self.TakeOff()
            if (self.status == DroneStatus.Flying or self.status == DroneStatus.Hovering):
                print ('Land')
                self.Land()
                break

    def Status(self):
        return self.status

    def Odometry(self):
        return self.odometry


# Definition of main and init node when use in another file
def main(arg):
    rospy.init_node('droneController', anonymous=True)
    rospy.loginfo('Init droneController node')
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print('shutting down droneController node')


if __name__ == '__main__':
    main(sys.argv)