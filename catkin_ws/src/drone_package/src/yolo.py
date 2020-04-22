#!/usr/bin/env python

import os
import cv2
import numpy as np

# This file contains the implementation that we use to detect people with YOLO and YOLO-v2
class Yolo():
    def __init__(self, confidence):
        # Initialize the YOLO descriptor/person detector
        # load the COCO class labels our YOLO model was trained on / Change the routes where we have our model
        labelsPath = os.path.sep.join(["/home/cristian/repos/trabajofingrado/catkin_ws/src/drone_package/src/YOLO", "coco.names"])
        self.LABELS = open(labelsPath).read().strip().split("\n")

        weightsPath = os.path.sep.join(
            ["/home/cristian/repos/trabajofingrado/catkin_ws/src/drone_package/src/YOLO", "yolov2-tiny.weights"])
        configPath = os.path.sep.join(
            ["/home/cristian/repos/trabajofingrado/catkin_ws/src/drone_package/src/YOLO", "yolov2-tiny.cfg"])

        self.min_confidence = float(confidence)
        self.net = cv2.dnn.readNetFromDarknet(configPath, weightsPath)

    def detection(self, cv_image):
        # determine only the *output* layer names that we need from YOLO
        ln = self.net.getLayerNames()
        ln = [ln[i[0] - 1] for i in self.net.getUnconnectedOutLayers()]

        (img_height, img_width) = cv_image.shape[:2]

        # pass of the YOLO object detector, giving us our bounding boxes and associated probabilities
        blob = cv2.dnn.blobFromImage(cv_image, 1 / 255.0, (186, 186), swapRB=True, crop=False)
        self.net.setInput(blob)
        layerOutputs = self.net.forward(ln)

        # initialize our lists of detected bounding boxes, confidences, and class IDs, respectively
        boxes = []
        confidences = []
        classIDs = []

        for output in layerOutputs:
            # loop over each of the detections
            for detection in output:
                scores = detection[5:]
                classID = np.argmax(scores)
                confidence = scores[classID]

                if (self.LABELS[classID] == 'person'):
                    # filter out weak predictions by ensuring the detected
                    # probability is greater than the minimum probability
                    if confidence >= self.min_confidence:
                        box = detection[0:4] * np.array([img_width, img_height, img_width, img_height])
                        (centerX, centerY, width, height) = box.astype("int")

                        # use the center (x, y)-coordinates to derive the top
                        # and and left corner of the bounding box
                        x = int(centerX - (width / 2))
                        y = int(centerY - (height / 2))

                        # update our list of bounding box coordinates, confidences, and class IDs
                        boxes.append([x, y, int(width), int(height)])
                        confidences.append(float(confidence))
                        classIDs.append(classID)

        # apply non-maxima suppression to suppress weak, overlapping bounding boxes
        idxs = cv2.dnn.NMSBoxes(boxes, confidences, 0.5, 0.3)
        pick = []
        if len(idxs) > 0:
            for i in idxs.flatten():
                # extract the bounding box coordinates
                pick.append(np.array([boxes[i][0], boxes[i][1], boxes[i][0] + boxes[i][2], boxes[i][1] + boxes[i][3]]))

        return pick, img_width, img_height, cv_image
