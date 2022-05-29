#!/usr/bin/env python

import rospy
import os
import subprocess
import numpy as np
import struct
import cv2
import numpy
import math
from std_msgs.msg import Float64
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
bridge = CvBridge()
cur_steer = 0;
cv_image = [];
c_l = 60
cam_matrix =  numpy.array([[322.59768648,0.,334.77373971],[ 0.,354.93133273,231.45054544],[0.,0.,1.]],numpy.float32)
dist_coefss = numpy.array([-0.23980236,0.05238634,0.01179236,-0.00902857,0.00645719],numpy.float32)


vv = np.float32([[118,435],[247,268],[405,273],[535,462]])
out = np.float32([[6*c_l,12*c_l-20],[6*c_l,7*c_l-20],[10*c_l,7*c_l-20],[10*c_l,12*c_l-20]])
pixel_ratio = 0.01
steering_ratio = 27.5/875.0

L = 6.85
r1 = 0
r3 = 0
r4 = 0
W = 2.85
back2cam = 1.785 + 0.5#mat dist
xmin = back2cam+0.2;
xmax = 7.5;


def image_callback(msg):
	global cv_image
	cv_image = bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
	#print(cv_image)
	
def steering_callback(msg):
	global r1,r3,r4,L
	wa = msg.data*steering_ratio;

	if(wa>0):
		r1 = L / math.tan(np.deg2rad(wa))
		r4 = math.sqrt(r1*r1 - L*L)
		r3 = r4 + 2.26
	elif(wa<0):
		r1 = L / math.tan(np.deg2rad(-1*wa))
		r3 = math.sqrt(r1*r1 - L*L)
		r4 = r3 + 2.26
	else:
		r3=0
		r4=0
		
	
def click_callback(event,x,y,flags,param):
	print(x,y)
	
rospy.init_node('backup_liner')
rospy.loginfo("[backup_liner] Backup Liner Started!")
im_sub = rospy.Subscriber("/cv_camera/image_raw",Image,image_callback)
str_sub = rospy.Subscriber("/steering_angle",Float64,steering_callback)
cv2.namedWindow('Rect Image')
cv2.setMouseCallback('Rect Image',click_callback)
last_time = rospy.get_time()
w = 640
h = 480
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(cam_matrix, dist_coefss, (w,h), 1, (w,h))
mapx, mapy = cv2.initUndistortRectifyMap(cam_matrix, dist_coefss, None, newcameramtx, (w,h), 5)

M = cv2.getPerspectiveTransform(vv,out)
invM = np.linalg.inv(M)

while not rospy.is_shutdown():
	if(len(cv_image)):
		recti = cv2.remap(cv_image,mapx,mapy, cv2.INTER_LINEAR);
		out = cv2.warpPerspective(recti,M,(16*c_l,12*c_l),flags=cv2.INTER_LINEAR)
		xs = []
		ys1 = []
		ys2 = []
		if(r3<r4):
			for x in np.arange(xmin,xmax,0.5):
				xs.append(12*c_l+math.ceil(back2cam*100) - math.ceil(x*100))
				ys1.append(8*c_l+113+math.ceil((r3 - math.sqrt(r3*r3-x*x))*100))
				ys2.append(8*c_l-113+math.ceil((r4 - math.sqrt(r4*r4-x*x))*100))
				
	
		elif(r4<r3):
			for x in np.arange(xmin,xmax,0.5):
				xs.append(12*c_l+math.ceil(back2cam*100) - math.ceil(x*100))
				ys1.append(8*c_l+113 -math.ceil((r3 - math.sqrt(r3*r3-x*x))*100))
				ys2.append(8*c_l-113 -math.ceil((r4 - math.sqrt(r4*r4-x*x))*100))
			
		else:
			for x in np.arange(xmin,xmax,0.5):
				xs.append(12*c_l+math.ceil(back2cam*100) - math.ceil(x*100))
				ys1.append(math.ceil(8*c_l+113))
				ys2.append(math.ceil(8*c_l-113))
		for i in range(0,len(xs)-1):
			cv2.line(out,(ys1[i],xs[i]),(ys1[i+1],xs[i+1]),(255,0,0),3,cv2.LINE_AA)
			cv2.line(out,(ys2[i],xs[i]),(ys2[i+1],xs[i+1]),(0,0,255),3,cv2.LINE_AA)
		
		invwarp = cv2.warpPerspective(out, invM, (recti.shape[1], recti.shape[0]))
		recti = cv2.addWeighted(recti, 1, invwarp, 0.9, 0)
			#cv2.line(recti,p3,p3,(0,0,255),2,cv2.LINE_AA)
		cv2.imshow("Rect Image",recti)
		cv2.imshow("Birds Eye",out)
			
			
	cv2.waitKey(1)

cv2.destroyAllWindows() 
