#!/usr/bin/env python

import sys
import cv2
import math
import rospy
import time
import argparse
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tf.transformations import euler_from_quaternion

from droneController import DroneController
from droneStatus import DroneStatus
from centroidtracker import CentroidTracker

from hog import Hog # Method to detect people
from caffe import Caffe # Method to detect people
from yolo import Yolo # Method to detect people

THRESHOLD = 0.5
MOVEMENTS = {'pitch': 0, 'roll': 0, 'z_velocity': 0, 'yaw_velocity': 0}
DETECTION_STATUS = {'SearchingPerson': 0, 'CenteringPerson': 1, 'SearchingLastPerson': 2}

class computerVision:

    def __init__(self, args):

        if (args['method'] == 'HOG'):
            self.method = Hog(args['confidence'])

        elif (args['method'] == 'CAFFE'):
            self.method = Caffe(args['confidence'])

        elif (args['method'] == 'YOLO'):
            self.method = Yolo(args['confidence'])

        else:
            print('Fail, unknown algorithm. Try with -m HOG, -m CAFFE, -m YOLO')
            rospy.signal_shutdown('Quit')

        # Initialize the bridge to transform the image detect by topic's ROS
        self.bridge = CvBridge()

        # initialize our centroid tracker and frame dimensions
        self.ct = CentroidTracker()

        # Initialize the subscriber and publisher
        self.image_sub = rospy.Subscriber("/ardrone/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("image_processed", Image, queue_size=10)
        self.controller = DroneController(args)
        self.droneStatus = DroneStatus()

        self.previous_position = None
        self.actual_position = None
        self.detection_status = DETECTION_STATUS['SearchingPerson']
        self.last_action = {'roll': 0, 'pitch': 0, 'yaw_velocity': 0, 'z_velocity': 0}
        self.shutdown = 500


    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")

            if self:
                if self.controller:
                    self.actual_position = self.controller.Odometry()

                    if self.actual_position:
                        explicit_quat = [self.actual_position.pose.pose.orientation.x, self.actual_position.pose.pose.orientation.y,
                                         self.actual_position.pose.pose.orientation.z, self.actual_position.pose.pose.orientation.w]
                        roll, pitch, actual_yaw = euler_from_quaternion(explicit_quat)
                        actual_yaw = (actual_yaw * 180) / math.pi

                    self.controller.TakeOff()

                # If we are shutting down the drone, send land command
                rospy.on_shutdown(self.controller.Land)
        except CvBridgeError as e:
            print(e)

        try:
            if (self.controller.Status() == self.droneStatus.Hovering or self.controller.Status() == self.droneStatus.Flying):
                # Obtain the detection and a few params
                pick, img_width, img_height, cv_image = self.method.detection(cv_image)
                area_image = img_height * img_width
                # update our centroid tracker using the computed set of bounding box rectangles
                objects = self.ct.update(pick)

                # Tracking
                # loop over the tracked objects
                minObjectID = None
                if len(objects.items()) > 0:
                    minObjectID = objects.items()[0]

                pickToFollow = []
                minDistance = None
                continue_move = True

                # Calculate the pick to follow with the min distance for the centroid with min ID and draw the squares
                for (x, y, w, h) in pick:
                    (objectID, centroid) = minObjectID
                    if not pickToFollow:
                        pickToFollow = [(x, y, w, h)]
                        minDistance = abs(centroid[0] - ((float(x) + float(w)) / 2) + centroid[1] - ((float(y) + float(h)) / 2))
                    else:
                        if (abs(centroid[0] - ((float(x) + float(w)) / 2) + centroid[1] - ((float(y) + float(h)) / 2)) < minDistance):
                            pickToFollow = [(x, y, w, h)]
                            minDistance = abs(centroid[0] - ((float(x) + float(w)) / 2) + centroid[1] - ((float(y) + float(h)) / 2))

                    cv2.rectangle(cv_image, (x, y), (w, h), (0, 255, 0), 2)

                    # Avoid people nearly of drone detected
                    area_rectangle_person = (w - x) * (h - y)
                    if ((area_rectangle_person >= area_image/3) and len(pick)>1):
                        continue_move = False
                        print('There is a person too near')

                for (index, (id, centroid)) in enumerate(objects.items()):
                    # draw both the ID of the object and the centroid of the
                    # object on the output frame
                    if (index == 0):
                        text = "Target"
                    else:
                        text = "Possible target"
                    cv2.putText(cv_image, text, (centroid[0] - 10, centroid[1] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                    cv2.circle(cv_image, (centroid[0], centroid[1]), 4, (0, 255, 0), -1)

                MOVEMENTS['pitch'] = 0  # (+)left and (-)right
                MOVEMENTS['roll'] = 0  # (+)forward and (-)backward
                MOVEMENTS['z_velocity'] = 0  # (+)up and (-)down
                MOVEMENTS['yaw_velocity'] = 0  # The rotational velocity

                # We left the drone stabilized, without moving
                self.controller.SetCommand(MOVEMENTS['roll'], MOVEMENTS['pitch'], MOVEMENTS['yaw_velocity'], MOVEMENTS['z_velocity'])
                self.shutdown = self.shutdown - 1

                if continue_move:
                    for (x, y, w, h) in pickToFollow:
                        area_rectangle = (w - x) * (h - y)

                        self.detection_status = DETECTION_STATUS['CenteringPerson']

                        print('----------------------------------------------------')
                        if ((float(x) / float(img_width)) >= 0.40):
                            print('Turn right')
                            MOVEMENTS['yaw_velocity'] = -0.20
                        elif ((float(x) / float(img_width)) <= 0.30):
                            print('Turn left')
                            MOVEMENTS['yaw_velocity'] = 0.05
                        else:
                            print("Don't turn")
                            MOVEMENTS['yaw_velocity'] = 0
                        if ((float(y) / float(img_height)) >= 0.15):
                            print('Down')
                            MOVEMENTS['z_velocity'] = -0.10
                        elif ((float(y) / float(img_height)) <= 0.08):
                            print('Up')
                            MOVEMENTS['z_velocity'] = 0.10
                        else:
                            print("Don't up don't down")
                            MOVEMENTS['z_velocity'] = 0
                        if (area_rectangle <= (float(area_image) / 5)):
                            print('Zoom in')
                            MOVEMENTS['roll'] = 0.10
                        elif (area_rectangle > (float(area_image) / 4)):
                            print('Zoom out')
                            MOVEMENTS['roll'] = -0.05
                        else:
                            MOVEMENTS['roll'] = 0
                            print("Don't zoom in don't zoom out")

                        self.last_action = {'roll': MOVEMENTS['roll'], 'pitch': MOVEMENTS['pitch'], 'yaw_velocity': MOVEMENTS['yaw_velocity'], 'z_velocity': MOVEMENTS['z_velocity']}

                        # Send the movement corresponding to the drone
                        self.controller.SetCommand(MOVEMENTS['roll'], MOVEMENTS['pitch'], MOVEMENTS['yaw_velocity'], MOVEMENTS['z_velocity'])
                        self.previous_position = None
                        self.shutdown = 500

                    # If the drone lost the person detected, try moving in a manner contrary to the last movement made
                    if (len(pick) == 0 and (self.detection_status == DETECTION_STATUS['CenteringPerson'] or
                                            self.detection_status == DETECTION_STATUS['SearchingLastPerson']) and
                                            self.shutdown < 450):
                        self.detection_status = DETECTION_STATUS['SearchingLastPerson']
                        print('Search the last person detected')
                        # Do the last movement but inverted
                        self.controller.SetCommand((self.last_action['roll'] * -1), (self.last_action['pitch'] * -1),
                                                   (self.last_action['yaw_velocity'] * -1), (self.last_action['z_velocity'] * -1))

                    # If dont detect people, the drone start to rotate on the left with the objective of finding a person
                    if self.shutdown <= 250:
                        if self.previous_position is None:
                            self.previous_position = self.controller.Odometry()

                        number = float(self.shutdown) / float(100)
                        int_part = int(number)
                        decimal_part = number - int_part
                        if (decimal_part==0):
                            rospy.loginfo("Warning: The drone is searching people")
                        self.controller.SetCommand(0.0, 0.0, 0.15, 0.0)
                        self.detection_status = DETECTION_STATUS['SearchingPerson']

                        # Obtain the actual position of drone to calculate the rotation
                        if self.previous_position:
                            explicit_quat2 = [self.previous_position.pose.pose.orientation.x, self.previous_position.pose.pose.orientation.y,
                                             self.previous_position.pose.pose.orientation.z, self.previous_position.pose.pose.orientation.w]
                            roll, pitch, previous_yaw = euler_from_quaternion(explicit_quat2)
                            previous_yaw = (previous_yaw*180)/math.pi

                            # If the actual yaw is very nearly to the beginning yaw when the drone started to rotate
                            # land the drone and finish the detection and the system
                            if abs(actual_yaw - previous_yaw) <= THRESHOLD and self.shutdown < 200:
                                rospy.loginfo("Warning: The drone turn on and will be landed")
                                self.controller.SetCommand(0.0, 0.0, 0.0, 0.0)
                                rospy.sleep(2.)
                                self.controller.Land()
                                rospy.signal_shutdown('Quit')

                else:
                    print('Warning: The drone stop because there is a person detected nearby')
            else:
                print('Warning: The drone is in process to flying, hovering or landing, and during this time not detect')
                print('State: '+str(self.controller.Status()))

            # Convert the image with opencv format to format message ROS topic
            try:
                self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            except CvBridgeError as e:
                print (e)
        except KeyboardInterrupt:
            print("Shutting down node with KeyboardInterrupt")


def main(args):
    ap = argparse.ArgumentParser()
    ap.add_argument("-m", "--method", type=str, required=True,
                    help="method to process the image and detect")
    ap.add_argument("-c", "--confidence", type=float, default=.1,
                    help="confidence threshold")
    ap.add_argument("-e", "--environment", type=str, required=True,
                help="environment configuration, can be simulated or real")
    args = vars(ap.parse_args())
    # Init the node in ROS
    rospy.init_node("computerVision", anonymous=True)
    rospy.loginfo("Init computerVision")
    image = computerVision(args)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down node")

if __name__ == '__main__':
    main(sys.argv)