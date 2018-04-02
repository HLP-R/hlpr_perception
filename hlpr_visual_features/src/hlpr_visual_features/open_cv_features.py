#!/usr/bin/env python

# Copyright (c) 2017, Elaine Short, SIM Lab
# All rights reserved.
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# 
# * Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# 
# * Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# 
# * Neither the name of the SIM Lab nor the names of its
#   contributors may be used to endorse or promote products derived from
#   this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.


import rospy
import cv2
import numpy
import math
import argparse
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from contingency.msg import ArrayFeature


class CVImageSubscriber:
    def __init__(self, display_on):
        self._sub=rospy.Subscriber("/camera/rgb/image_rect_color", Image, self._cb)
        self._bridge = CvBridge()
        self._fgbg = cv2.createBackgroundSubtractorMOG2(history=20)
        self._motion_pub = rospy.Publisher("/contingency/opencv/motion",ArrayFeature, queue_size=1)

        self._flow_ang_pub = rospy.Publisher("/contingency/opencv/flow/angle",ArrayFeature, queue_size=1)
        self._mag_pub = rospy.Publisher("/contingency/opencv/flow/magnitude",ArrayFeature, queue_size=1)
        self._acc_pub = rospy.Publisher("/contingency/opencv/flow/acceleration",ArrayFeature, queue_size=1)
        self._mag_total_pub = rospy.Publisher("/contingency/opencv/flow/total/magnitude",ArrayFeature, queue_size=1)
        self._ang_total_pub = rospy.Publisher("/contingency/opencv/flow/total/angle",ArrayFeature, queue_size=1)


        self._splits = 4
        self._disp = display_on
	self._prev = []
        self._prev_mag = [0 for i in range(self._splits)]

    def _cb(self,data):
        try:
            cv_image=self._bridge.imgmsg_to_cv2(data, "passthrough")
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        except CvBridgeError as e:
            rospy.logerr(e)
            return
        
        fgmask=self._fgbg.apply(cv_image)
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(3,3))
        fgmask = cv2.morphologyEx(fgmask,cv2.MORPH_OPEN, kernel)

        flow = []
        if len(self._prev)>0:
            hsv = numpy.zeros_like(cv2.cvtColor(cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR),cv2.COLOR_BGR2HSV))
            hsv[...,1] = 255
            flow = cv2.calcOpticalFlowFarneback(self._prev,cv_image, None, 
                                                0.5, 3, 15, 3, 5, 1.2, 0)
            mag, ang = cv2.cartToPolar(flow[...,0], flow[...,1])
            total = [numpy.sum(flow[...,0]),
                     numpy.sum(flow[...,1])]
            
            total_mag = math.sqrt(total[0]*total[0]+total[1]*total[1])
            total_ang = math.atan2(total[1],total[0])

            mag[numpy.isinf(mag)]=0

        if self._disp:
            cv2.imshow("frame",fgmask)#cv2.bitwise_and(cv_image,cv_image,mask=fgmask))
            cv2.imshow("frame0",cv_image)
            if len(flow)>0:
                hsv[...,0] =  ang*180/numpy.pi/2
                hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
                bgr = cv2.cvtColor(hsv,cv2.COLOR_HSV2BGR)
                cv2.imshow('frame2',bgr)
            cv2.waitKey(3)

        w = int(fgmask.shape[1]/self._splits)
        
        data_movement = []
        data_flow_mag = []
        data_flow_ang = []

        for i in range(self._splits):
            start = i*w
            end = min(i*w, fgmask.shape[1])
            data_movement.append(numpy.average(fgmask[:,w*i:w*(i+1)]))
            if len(flow)>0:
                data_flow_mag.append(numpy.average(mag[:,w*i:w*(i+1)]))
                try:
                    data_flow_ang.append(numpy.average(ang[:,w*i:w*(i+1)],weights = mag[:,w*i:w*(i+1)]*mag[:,w*i:w*(i+1)]>10))
                except ZeroDivisionError:
                    data_flow_ang.append(0)
        self._prev = cv_image    
        

        self._motion_pub.publish(ArrayFeature(data=data_movement))

        if len(flow)>0:
            self._flow_ang_pub.publish(ArrayFeature(data=data_flow_ang))
            self._mag_pub.publish(ArrayFeature(data=data_flow_mag))
            self._acc_pub.publish(ArrayFeature(data=[data_flow_mag[i]-self._prev_mag[i] for i in range(len(data_flow_mag))]))
            self._mag_total_pub.publish(ArrayFeature(data=[total_mag]))
            self._ang_total_pub.publish(ArrayFeature(data=[total_ang]))
            self._prev_mag = data_flow_mag

if __name__=="__main__":
    rospy.init_node("background_subtraction_features")
    parser=argparse.ArgumentParser(description="Use opencv to get motion features from video")
    parser.add_argument('-d', '--display-video', help="Show the masked video on screen", action='store_true')
    args = parser.parse_known_args()[0]
    
    c = CVImageSubscriber(args.display_video)
    rospy.spin()
