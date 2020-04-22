#!/usr/bin/env python

import cv2
import imutils
import numpy as np
from imutils.object_detection import non_max_suppression

# This file contains the implementation that we use to detect people with HOG
class Hog():

    def __init__(self, confidence):
        # Initialize the HOG descriptor/person detector and the min confidence parameter
        self.confidence = confidence
        self.hog = cv2.HOGDescriptor()
        self.hog.setSVMDetector(cv2.HOGDescriptor_getDefaultPeopleDetector())

    def detection(self, cv_image):
        # With this dimensions 215, 126, the real dron classify correctly, except when the camera don't capture all the body
        # This is a big problem with HOG, that the algorithm don't detect people when don't capture all the body
        # We need to resize the image to process in real time with my computer
        cv_image = imutils.resize(cv_image, width=215, height=175)
        (img_height, img_width) = cv_image.shape[:2]

        # Detect people in the image
        (rects, weights) = self.hog.detectMultiScale(cv_image, winStride=(2, 2), padding=(8, 8), scale=1.05)

        rects_aux = []
        for i in range(len(weights)):
            # If the probability of the each rect is more than confidence, include to process
            if weights[i] >= self.confidence:
                rects_aux.append(rects[i])

        # Apply non-maxima suppression to the bounding boxes to try to maintain overlapping boxes
        rects = np.array([[x, y, x + w, y + h] for (x, y, w, h) in rects_aux])
        pick = non_max_suppression(rects, probs=None, overlapThresh=0.65)  # [[x y w h]]

        return pick, img_width, img_height, cv_image