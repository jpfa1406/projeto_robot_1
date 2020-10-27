#! /usr/bin/env python
# -*- coding:utf-8 -*-

import rospy
import numpy as np
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError

import cv2.aruco as aruco
import sys

def desenha_contornos(mask):
    contornos, arvore = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    mask_rgb = cv2.cvtColor(mask, cv2.COLOR_GRAY2RGB) 
    contornos_img = mask_rgb.copy()
    cv2.drawContours(contornos_img, contornos, -1, [255, 0, 0], 5)
    cv2.imshow("image", contornos_img)