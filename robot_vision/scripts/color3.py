#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import os
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from robot_vision.msg import ballposition
from robot_vision.msg import color_choice

chasing_color = 0



class image_converter:
    def __init__(self):
        # 创建cv_bridge，声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("cv_bridge_image",
                                         Image,
                                         queue_size=1)
        self.colorchoice = rospy.Subscriber("color", color_choice,
                                            self.color_flag)
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber("/kinect/rgb/image_raw", Image,
                                          self.callback)
        self.pub = rospy.Publisher("ball_position",
                                   ballposition,
                                   queue_size=10)

    def color_flag(self, color):
        global chasing_color
        chasing_color = color.c
        #print(chasing_color)
    def callback(self, data):
        # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print e

        # 在opencv的显示窗口中绘制一个圆，作为标记
        #(rows,cols,channels) = cv_image.shape
        #if cols > 60 and rows > 60 :
        #cv2.circle(cv_image, (60, 60), 30, (0,0,255), -1)

        # 显示Opencv格式的图像
        #cv2.imshow("Image window1", cv_image)
        #cv2.waitKey(3)

        #设置颜色范围
        redlower = np.array([0, 43, 46])
        redupper = np.array([3, 255, 255])
        bluelower = np.array([95, 43, 46])
        blueupper = np.array([190, 255, 255])
        greenlower = np.array([55, 43, 46])
        greenupper = np.array([60, 255, 255])
        blacklower = np.array([0, 0, 0])
        blackupper = np.array([180, 255, 46])
        whitelower = np.array([0, 0,90])
        whiteupper = np.array([0, 0, 135])
        goldlower = np.array([27, 210, 100])
        goldupper = np.array([32, 255, 225])
        Ballposition = ballposition()  #给策略的消息 by WYZ
        Ballposition.flag = 0
        Ballposition.kict = 0
        flag = 0
        cv2.imwrite("save.jpg",cv_image)
        
        #将图像转化为HSV格式
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        #除去指定颜色范围内的其他颜色
        mask1 = cv2.inRange(hsv, bluelower, blueupper)
        mask2 = cv2.inRange(hsv, redlower, redupper)
        mask3 = cv2.inRange(hsv, greenlower, greenupper)
        mask4 = cv2.inRange(hsv, goldlower, goldupper)
        mask5 = cv2.inRange(hsv, whitelower, whiteupper)
        mask6 = cv2.inRange(hsv, blacklower, blackupper)
        

        #二职化操作
        ret, binary1 = cv2.threshold(mask1, 30, 255, cv2.THRESH_BINARY)
        ret, binary2 = cv2.threshold(mask2, 30, 255, cv2.THRESH_BINARY)
        ret, binary3 = cv2.threshold(mask3, 30, 255, cv2.THRESH_BINARY)
        ret, binary4 = cv2.threshold(mask4, 30, 255, cv2.THRESH_BINARY)
        ret, binary5 = cv2.threshold(mask5, 30, 255, cv2.THRESH_BINARY)
        ret, binary6 = cv2.threshold(mask6, 30, 255, cv2.THRESH_BINARY)

        #膨胀操作
        kernel1 = np.ones((5, 5), np.uint8)
        kernel2 = np.ones((5, 5), np.uint8)
        kernel3 = np.ones((5, 5), np.uint8)
        kernel4 = np.ones((5, 5), np.uint8)
        kernel5 = np.ones((5, 5), np.uint8)
        kernel6 = np.ones((5, 5), np.uint8)
        dilation1 = cv2.dilate(binary1, kernel1, iterations=1)
        dilation2 = cv2.dilate(binary2, kernel2, iterations=1)
        dilation3 = cv2.dilate(binary3, kernel3, iterations=1)
        dilation4 = cv2.dilate(binary4, kernel4, iterations=1)
        dilation5 = cv2.dilate(binary5, kernel5, iterations=1)
        dilation6 = cv2.dilate(binary6, kernel6, iterations=1)
        cv2.imshow("Image window1", dilation4)

        #获取图像轮廓坐标
        _, contours1, hierarchy = cv2.findContours(dilation1,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
        _, contours2, hierarchy = cv2.findContours(dilation2,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
        _, contours3, hierarchy = cv2.findContours(dilation3,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
        _, contours4, hierarchy = cv2.findContours(dilation4,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
        _, contours5, hierarchy = cv2.findContours(dilation5,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)
        _, contours6, hierarchy = cv2.findContours(dilation6,
                                                   cv2.RETR_EXTERNAL,
                                                   cv2.CHAIN_APPROX_SIMPLE)                                                                                      
        global chasing_color
        #print(chasing_color)
        if (chasing_color==2):
            if len(contours1) > 0:
                #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1) 
                
                boxes1 = [cv2.boundingRect(c) for c in contours1]
                area=0
                for i in range(len(contours1)):
                	area += cv2.contourArea(contours1[i])
                for box1 in boxes1:
                    x, y, w, h = box1
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                                  (153, 153, 0), 2)
                    if( (w*w)/area>4/3.14-0.1 and (w*w)/area<4/3.14+0.1):
			Ballposition.x=x
                        Ballposition.w=w
			Ballposition.flag=1
                    	cv2.circle(cv_image,(x+w/2,y+h/2),w/2,(36,255,12),3)
		    if(w>600 or h>280):
			Ballposition.kict=1;

        if (chasing_color==0):
            if len(contours2) > 0:
                #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)
                boxes2 = [cv2.boundingRect(c) for c in contours2]
                area=0
                for i in range(len(contours2)):
                	area += cv2.contourArea(contours2[i])
                for box2 in boxes2:
                    x, y, w, h = box2
                    # 在opencv的显示窗口中绘制一个圆，作为标记
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                                  (153, 153, 0), 2)
                    if( (w*w)/area>4/3.14-0.1 and (w*w)/area<4/3.14+0.1):
                        Ballposition.x=x
                        Ballposition.w=w
			Ballposition.flag=1
			cv2.circle(cv_image,(x+w/2,y+h/2),w/2,(36,255,12),3)
		    if(w>600 or h>280):
			Ballposition.kict=1;
                    	
        if (chasing_color==1):
            #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)
            if len(contours3) > 0:
                boxes3 = [cv2.boundingRect(c) for c in contours3]
                area=0
                for i in range(len(contours3)):
                	area += cv2.contourArea(contours3[i])
                for box3 in boxes3:
                    x, y, w, h = box3
                    
                    # 在opencv的显示窗口中绘制一个圆，作为标记
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                                  (153, 153, 0), 2)
                    if( (w*w)/area>4/3.14-0.1 and (w*w)/area<4/3.14+0.1):
                        Ballposition.x=x
                        Ballposition.w=w
			Ballposition.flag=1
                    	cv2.circle(cv_image,(x+w/2,y+h/2),w/2,(36,255,12),3)
		    if(w>600 or h>280):
			Ballposition.kict=1;
        if (chasing_color==3):
            if len(contours4) > 0:
                #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)
                boxes4 = [cv2.boundingRect(c) for c in contours4]
                area=0
                for i in range(len(contours4)):
                	area += cv2.contourArea(contours4[i])
                for box4 in boxes4:
                    x, y, w, h = box4
                    print(w*w/area)
                    # 在opencv的显示窗口中绘制一个圆，作为标记
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                                  (153, 153, 0), 2)
                    if( (w*w)/area>4/3.14-0.21 and (w*w)/area<4/3.14+0.21):
                        Ballposition.x=x
                        Ballposition.w=w
			Ballposition.flag=1
                    	cv2.circle(cv_image,(x+w/2,y+h/2),w/2,(36,255,12),3)
		    if(w>600 or h>280):
			Ballposition.kict=1;
        if (chasing_color==4) :
            if len(contours5) > 0:
                #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)
                boxes5 = [cv2.boundingRect(c) for c in contours5]
                area=0
                for i in range(len(contours5)):
                	area += cv2.contourArea(contours5[i])

                for box5 in boxes5:
                    x, y, w, h =box5
                    # 在opencv的显示窗口中绘制一个圆，作为标记
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                                  (153, 153, 0), 2)
                    if((w-h)<=2 or (w-h)>=-2):
			s=(4.0/3.14*h/w)
                    	if(s>1.27-0.1 and s<1.27+0.1 and w*h>900):
				Ballposition.x=x
                                Ballposition.w=w
				Ballposition.flag=1
                    		cv2.circle(cv_image,(x+w/2,y+h/2),w/2,(36,255,12),3)
		    if(w>600 or h>280):
			Ballposition.kict=1;
        if (chasing_color==5):
            #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)
            #flag=0
            if len(contours6) > 0:
                #cv2.circle(cv_image, (60, 60), 30, (0, 0, 255), -1)
                boxes6 = [cv2.boundingRect(c) for c in contours6]
                #print(area)
		area=0
		for i in range(len(contours6)):
                	area += cv2.contourArea(contours6[i])
                for box6 in boxes6:
                    x, y, w, h=box6
                    #在opencv的显示窗口中绘制一个圆，作为标记
                    cv2.rectangle(cv_image, (x, y), (x + w, y + h),
                                  (153, 153, 0), 2)
                    if(((w-h)<=2 or (w-h)>=-2) and w*h>900):
			s=(4.0/3.14*h/w)
                    	if(s>1.27-0.1 and s<1.27+0.1 ):
				Ballposition.x=x
                                Ballposition.w=w
				Ballposition.flag=1
                    		cv2.circle(cv_image,(x+w/2,y+h/2),w/2,(36,255,12),3)
			if(w>600 or h>280):
				Ballposition.kict=1;           
        cv2.imshow("Image window2", cv_image)
        cv2.waitKey(3)

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            self.pub.publish(Ballposition)  #给策略的消息 by WYZ
        except CvBridgeError as e:
            print e


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("color3")
        rospy.loginfo("Starting color3")
        image_converter()
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down color3"
        cv2.destroyAllWindows()


