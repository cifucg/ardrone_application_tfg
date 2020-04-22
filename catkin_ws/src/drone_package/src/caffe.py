#!/usr/bin/env python

import cv2
import imutils
import numpy as np

# This file contains the implementation that we use to detect people with CAFFE and MOBILENET_SSD
class Caffe():

    def __init__(self, confidence):
        # initialize the list of class labels MobileNet SSD was trained to
        # detect, then generate a set of bounding box colors for each class
        self.CLASSES = ["background", "aeroplane", "bicycle", "bird", "boat",
                        "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
                        "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
                        "sofa", "train", "tvmonitor"]

        self.COLORS = np.random.uniform(0, 255, size=(len(self.CLASSES), 3))

        # load our serialized model from disk / Change the route where we have our model
        prototxt = '/home/cristian/repos/trabajofingrado/catkin_ws/src/drone_package/src/CAFFE/MobileNetSSD_deploy.prototxt.txt'
        model = '/home/cristian/repos/trabajofingrado/catkin_ws/src/drone_package/src/CAFFE//MobileNetSSD_deploy.caffemodel'
        self.min_confidence = float(confidence)
        self.net = cv2.dnn.readNetFromCaffe(prototxt, model)

    def detection(self, cv_image):
        # Resize the image to process in real time
        cv_image = imutils.resize(cv_image, width=400, height=460)
        # grab the frame dimensions and convert it to a blob
        (img_height, img_width) = cv_image.shape[:2]
        blob = cv2.dnn.blobFromImage(cv2.resize(cv_image, (128, 128)), 0.007843, (128, 128), 127.5)
        if self.net:
            self.net.setInput(blob)
            detections = self.net.forward()

        pick = []
        if detections is not None:
            for i in np.arange(0, detections.shape[2]):
                confidence = detections[0, 0, i, 2]
                if confidence >= self.min_confidence:
                    # extract the index of the class label from the detections list
                    idx = int(detections[0, 0, i, 1])
                    # if the class label is not a person, ignore it
                    if self.CLASSES[idx] == "person":
                        box = detections[0, 0, i, 3:7] * np.array([img_width, img_height, img_width, img_height])
                        pick.append(box.astype("int"))

        return pick, img_width, img_height, cv_image