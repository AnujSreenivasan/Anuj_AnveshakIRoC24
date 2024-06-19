#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
import numpy as np
import cv2
import imutils
from cv_bridge import CvBridge
class find_center():
    def __init__(self):
        self.up=np.array([140,255,255])
        self.down=np.array([80,100,30])
        rospy.Subscriber('/zed2i/zed_node/rgb/image_rect_color', Image, self.get_cframe)
        rospy.Subscriber('/zed2i/zed_node/depth/depth_registered', Image, self.get_dframe)
        self.rate = rospy.Rate(10)
        self.cframe = []
        self.dframe = []
        self.image_arrived = False
        self.depth_arrived= False
    def get_dframe(self,data):
        bridge=CvBridge()
        self.dframe=data
        self.dframe=bridge.imgmsg_to_cv2(self.dframe, "passthrough")
        self.depth_arrived =True
    def get_cframe(self,data):
        bridge=CvBridge()
        self.cframe=data
        self.cframe=bridge.imgmsg_to_cv2(self.cframe, "bgr8")
        self.image_arrived = True
    def contour(self):
        print('cframe shape', len(np.shape(self.cframe)))
        print('dframe shape', len(np.shape(self.dframe)))
        if self.image_arrived == True and self.depth_arrived == True and len(np.shape(self.cframe)) == 3 and len(np.shape(self.dframe)) ==2:
            blue_hsv=cv2.cvtColor(self.cframe,cv2.COLOR_BGR2HSV)
            mask=cv2.inRange(blue_hsv,self.down,self.up)
            ker=np.ones((7,7),np.uint8)
            mask=cv2.morphologyEx(mask,cv2.MORPH_CLOSE,ker)
            mask=cv2.morphologyEx(mask,cv2.MORPH_OPEN,ker)
            cont=cv2.findContours(mask.copy(),cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
            cont=imutils.grab_contours(cont)
            area=[]
            Y_array=[]
            print("hi")
            self.D_array=[]
            for i in cont:
                area.append(cv2.contourArea(i))
            if area==[]:
                pass
            else:
                print("hi3")
                z=max(area)
                i=area.index(z)
                app=cv2.approxPolyDP(cont[i],0.005*cv2.arcLength(cont[i],True),True)
                self.x,self.y,self.w,self.h=cv2.boundingRect(app)
                Y=self.y+1
                print("hi4")
                d1=self.get_depth(self.x+self.w/2,Y-1)
                print("d1", d1)
                while Y<=(self.y+self.h/2):
                    Y_array.append(Y)
                    d=self.get_depth(self.x+self.w/2,Y)
                    print("d", d)
                    if d == None or d1 == None:
                        pass
                    else:
                        self.D_array.append(d1-d)
                    d1=d
                    Y=Y+1
                self.D_array=self.D_array[:len(self.D_array)-1]
                print("self.D_array", self.D_array)
                
                Y_array=Y_array[:len(Y_array)-1]
                print("before declaration of self.check")
                self.check=[]
                for i in range(len(self.D_array)):
                    if i==len(self.D_array)-1:
                        break
                    else:
                        if (self.D_array[i]>=0 and self.D_array[i+1]<=0) or (self.D_array[i+1]>=0 and self.D_array[i]<=0):
                            self.check.append(i+1)
                print("before self.p_x and self.p_y")
                print("self.check", self.check)
                    
                if len(self.check) >=3:
                    self.p_x=int(self.x+self.w/2)
                    self.p_y=int((Y_array[self.check[0]]+Y_array[self.check[1]])/2)
                    print('type check inside self.check',type(self.cframe))
                    print("hi2")
                    self.d1=self.get_depth(self.x+self.w/2,Y_array[self.check[0]])
                    self.d2=self.get_depth(self.x+self.w/2,Y_array[self.check[1]])
                    self.get_coords()
    def get_depth(self,x,y):
        if x<360 and y<640:
            print("x and y in get_depth", x,y)
            depth=self.dframe[int(y),int(x)]
            return depth
    def get_coords(self):
        f_x=527.2972398956961
        f_y=527.2972398956961
        c_x=658.8206787109375
        c_y=372.25787353515625
        self.d=np.sqrt(0.5*(self.d1**2+self.d2**2-(0.15**2)/2))
        self.cord_x=self.d*(self.p_x-c_x)/f_x
        self.cord_y=self.d*(self.p_y-c_y)/f_y
        self.cord_z=self.d
    def spin(self):
        while not rospy.is_shutdown():
            self.contour()
            self.rate.sleep()
if __name__ == '__main__':
    try:
        rospy.init_node('iroc_centre', anonymous=True)
        rate = rospy.Rate(5)
        auto = find_center()
        auto.spin()
    except rospy.ROSInterruptException:
        pass
