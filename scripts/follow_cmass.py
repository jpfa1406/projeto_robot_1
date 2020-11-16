#!/usr/bin/python
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import tf
import math
import cv2
import time
from geometry_msgs.msg import Twist, Vector3, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError

bridge = CvBridge()
cv_image = None
atraso = 1.5E9 
check_delay = False 

centro = None
point = None
heart = None
dist = None
dist_creeper = None

persuit = False
close_enough = False
choose_color = False
w = None
state = 0

ranges = None
minv = 0
maxv = 10

low = np.array([22, 50, 50],dtype=np.uint8)
high = np.array([36, 255, 255],dtype=np.uint8)

def filter_color(bgr, low, high):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, low, high)
    return mask     

def center_of_mass(mask):
    M = cv2.moments(mask)
    if M["m00"] == 0:
        M["m00"] = 1
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]

def crosshair(img, point, size, color):
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,5)
    cv2.line(img,(x,y - size),(x, y + size),color,5)

def center_of_mass_region(mask, x1, y1, x2, y2):
    mask_bgr = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    clipped = mask[y1:y2, x1:x2]
    c = center_of_mass(clipped)
    c[0]+=x1
    c[1]+=y1
    crosshair(mask_bgr, c, 10, (0,0,255))
    cv2.rectangle(mask_bgr, (x1, y1), (x2, y2), (255,0,0),2,cv2.LINE_AA)
    point = c[0]
    return mask_bgr, point

def roda_todo_frame(imagem):
    print("frame")
    global cv_image
    global centro
    global point
    global heart
    global dist
    global dist_creeper
    global w
    global state
    global persuit
    global close_enough
    global choose_color

    now = rospy.get_rostime()
    imgtime = imagem.header.stamp
    lag = now-imgtime 
    delay = lag.nsecs
    print("delay ", "{:.3f}".format(delay/1.0E9))
    if delay > atraso and check_delay==True:
        print("Descartando por causa do delay do frame:", delay)
        return 
    try:
        if choose_color == False:
            color = input('Escolha uma cor de creeper(p/o/b): ')
            if color == 'b':
                low_creeper = np.array([90, 50, 50],dtype=np.uint8)
                high_creeper = np.array([120, 255, 255],dtype=np.uint8) 
            elif color == 'p':
                low_creeper = np.array([90, 50, 50],dtype=np.uint8)
                high_creeper = np.array([120, 255, 255],dtype=np.uint8) 
            elif color == '0':
                low_creeper = np.array([90, 50, 50],dtype=np.uint8)
                high_creeper = np.array([120, 255, 255],dtype=np.uint8) 
            choose_color = True

        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
        cv_image = cv2.resize(cv_image,(cv_image.shape[1]*2,cv_image.shape[0]*2))

        mask = filter_color(cv_image, low, high)
        mask = cv2.morphologyEx(mask,cv2.MORPH_OPEN,np.ones((5, 5)))
        mask_bgr, point = center_of_mass_region(mask, 20, 300, cv_image.shape[1] - 20, cv_image.shape[0] - 20) 
        centro = (cv_image.shape[1]//2)

        if persuit == False:
            dist = 0.01*(centro - point)
            if dist > 0.2:
                w = 0.2
            elif dist < -0.2:
                w = -0.2
            else:
                w = dist
        elif persuit == True:
            if heart != None:
                dist_creeper = 0.01*(centro - heart[0])
                if dist_creeper > 0.2:
                    w = 0.2
                elif dist_creeper < -0.2:
                    w = -0.2
                else:
                    w = dist_creeper
        print(low_creeper)
            
        cv_HSV = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        segmentado_cor = cv2.inRange(cv_HSV, low_creeper, high_creeper)
        segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
        contornos, arvore = cv2.findContours(segmentado_cor.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 
        maior_contorno_area = 0

        def center_of_contour(contorno):
            M = cv2.moments(contorno)
            if M["m00"] != 0:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                return (int(cX), int(cY))
            
        for cnt in contornos:
            area = cv2.contourArea(cnt)
            heart = center_of_contour(cnt)
            if area > maior_contorno_area:
                maior_contorno_area = area
                print(area)

        if maior_contorno_area > 800 and close_enough == False:
            persuit = True
            
        print(maior_contorno_area)
        cv2.imshow('mask creeper', segmentado_cor)
        cv2.waitKey(1)
        cv2.imshow('mask', mask_bgr)
        cv2.waitKey(1)
        cv2.imshow("Camera", cv_image)
    except CvBridgeError as e:
        print('ex', e)

def scaneou(dado):
    global ranges
    global minv
    global maxv
    print("Faixa valida: ", dado.range_min , " - ", dado.range_max )
    ranges = np.array(dado.ranges).round(decimals=2)
    print("Leituras:", ranges)
    minv = dado.range_min 
    maxv = dado.range_max

def controle(leituras):
    global persuit
    global close_enough
    if leituras is None:
        return
    for i in range(len(leituras)):
        dist = leituras[i]
        if minv < dist < maxv:
            if dist < 0.35:
                persuit = False
                close_enough = True
            elif dist > 3:
                close_enough = False

if __name__=="__main__":
    rospy.init_node("creeper")
    topico_imagem = "/camera/image/compressed"
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)
    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    print("Usando ", topico_imagem)
    pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    v = 0
    ROTATE = 0
    FOLLOW = 1
    PAIRING = 2
    PERSUIT = 3

    try:
        while not rospy.is_shutdown():

            controle(ranges)
            if persuit == False:
                if dist > -0.1 and dist < 0.1:
                    state = FOLLOW
                elif dist > 2.95 or dist < -2.95:
                    state = ROTATE
            else:
                if dist_creeper > -0.1 and dist_creeper < 0.1:
                    state = PERSUIT
                elif dist_creeper > 2.95 or dist_creeper < -2.95:
                    state = PAIRING

            if state == FOLLOW:
                v = 0.3
            elif state == ROTATE:
                v = 0
            elif state == PAIRING:
                v = 0
            elif state == PERSUIT:
                v = 0.3
            vel = Twist(Vector3(v,0,0), Vector3(0,0,w))
            pub.publish(vel)
            rospy.sleep(0.1)
    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")