#! /usr/bin/env python
# -*- coding:utf-8 -*-

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
import cormodule


bridge = CvBridge()
ranges = None
minv = 0
maxv = 10
state = 0

cv_image = None
media = []
centro = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos

area = 0.0 # Variavel com a area do maior contorno

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

# A função a seguir é chamada sempre que chega um novo frame
def roda_todo_frame(imagem):
	print("frame")
	global cv_image
	global media
	global centro

	now = rospy.get_rostime()
	imgtime = imagem.header.stamp
	lag = now-imgtime # calcula o lag
	delay = lag.nsecs
	print("delay ", "{:.3f}".format(delay/1.0E9))
	if delay > atraso and check_delay==True:
		print("Descartando por causa do delay do frame:", delay)
		return 
	try:
		antes = time.clock()
		cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")
		media, centro, maior_area =  cormodule.identifica_cor(cv_image)
		depois = time.clock()
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
	global state
	if leituras is None:
		return
	for i in range(len(leituras)):
		dist = leituras[i]
		if minv < dist < maxv:
			if dist < 0.35:
				state = MANUAL

	
if __name__=="__main__":
	rospy.init_node("creeper")
	topico_imagem = "/camera/rgb/image_raw/compressed"
	recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

	recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
	print("Usando ", topico_imagem)

	pub = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
	v = 0
	AUTO = 0
	MANUAL = 1

	try:

		while not rospy.is_shutdown():

			controle(ranges)

			if state == AUTO:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				if len(media) != 0 and len(centro) != 0:
					x_margin = abs(media[0]-centro[0])
					if (x_margin < 5):
						v = 0.2
					if (media[0] < centro[0]):
						vel = Twist(Vector3(v,0,0), Vector3(0,0,0.1))
					elif (media[0] > centro[0]):
						vel = Twist(Vector3(v,0,0), Vector3(0,0,-0.1))
					pub.publish(vel)
					rospy.sleep(0.1)

			elif state == MANUAL:
				vel = Twist(Vector3(0,0,0), Vector3(0,0,0))
				pub.publish(vel)
				rospy.sleep(0.1)


	except rospy.ROSInterruptException:
	    print("Ocorreu uma exceção com o rospy")
