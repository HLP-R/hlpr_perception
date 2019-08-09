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
from std_msgs.msg import Float64MultiArray




class CVImageSubscriber:
    def __init__(self, display_on):
        self._sub=rospy.Subscriber("/camera/color/image_raw", Image, self._cb)
        self._bridge = CvBridge()
        self._fgbg = cv2.createBackgroundSubtractorMOG2(history=5)
        self._motion_pub = rospy.Publisher("/features/opencv/motion",Float64MultiArray, queue_size=1)

        self._flow_ang_pub = rospy.Publisher("/features/opencv/flow/angle",Float64MultiArray, queue_size=1)
        self._mag_pub = rospy.Publisher("/features/opencv/flow/magnitude",Float64MultiArray, queue_size=1)
        self._acc_pub = rospy.Publisher("/features/opencv/flow/acceleration",Float64MultiArray, queue_size=1)
        self._mag_total_pub = rospy.Publisher("/features/opencv/flow/total/magnitude",Float64MultiArray, queue_size=1)
        self._ang_total_pub = rospy.Publisher("/features/opencv/flow/total/angle",Float64MultiArray, queue_size=1)


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

        move = []
        mag = []
        ang = []
        if len(self._prev)>0:
            hsv = numpy.zeros_like(cv2.cvtColor(cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR),cv2.COLOR_BGR2HSV))
            hsv[...,1] = 255
            #flow = cv2.calcOpticalFlowFarneback(self._prev,cv_image, None, 
            #                                    0.5, 3, 15, 3, 5, 1.2, 0)

            feats = cv2.goodFeaturesToTrack(self._prev, maxCorners=500, qualityLevel=0.001,minDistance=5)
            new_points,status,err = cv2.calcOpticalFlowPyrLK(self._prev, cv_image, feats,numpy.zeros_like(feats))

            move = new_points-feats
            
            mag,ang = cv2.cartToPolar(move[...,0],move[...,1])
            total = [numpy.sum(move[...,0]),
                     numpy.sum(move[...,1])]

            total_mag = math.sqrt(total[0]*total[0]+total[1]*total[1])/move.shape[0]
            total_ang = math.atan2(total[1],total[0])

            mag[numpy.isinf(mag)]=0

        if self._disp:
            cv2.imshow("frame",fgmask)#cv2.bitwise_and(cv_image,cv_image,mask=fgmask))
            cv2.imshow("frame0",cv_image)
            
            if len(move)>0:
                
                flow_img = cv2.cvtColor(cv_image,cv2.COLOR_GRAY2BGR)
                for i in range(len(feats)):
                    if status[i]==0:
                        continue
                    x0 = feats[i][0,0]
                    y0 = feats[i][0,1]
                    x1 = new_points[i][0,0]
                    y1 = new_points[i][0,1]
                    cv2.line(flow_img, (x0,y0),(x1,y1),(0,255,0))

                    x0 = flow_img.shape[1]/2
                    y0 = flow_img.shape[1]/2
                    x1 = int(x0+total_mag*4*math.cos(total_ang))
                    y1 = int(y0+total_mag*4*math.sin(total_ang))
                    cv2.line(flow_img, (x0,y0),(x1,y1),(255,0,0))
                    
                    
                    #    hsv[...,0] =  ang*180/numpy.pi/2
                #    hsv[...,2] = cv2.normalize(mag,None,0,255,cv2.NORM_MINMAX)
                cv2.imshow('frame1',flow_img)
            cv2.waitKey(3)

        w = int(fgmask.shape[1]/self._splits)
        
        data_movement = []
        splits = []

        for i in range(self._splits):
            start = i*w
            data_movement.append(numpy.average(fgmask[:,w*i:w*(i+1)]))
            splits.append(start)
        
        
        data_flow_mags = [[0] for i in range(self._splits)]
        data_flow_angs = [[0] for i in range(self._splits)]
        if len(move)>0:
            for i in range(len(move)):
                prop = feats[i][0,0]/float(fgmask.shape[1])
                idx =  int(math.floor(prop*self._splits))#, move[i]

                try:
                    data_flow_mags[idx] = numpy.append(data_flow_mags[idx],mag[i])
                    data_flow_angs[idx] = numpy.append(data_flow_angs[idx],ang[i])
                except IndexError:
                    print prop, idx, new_points[i][0,0], flow_img.shape[1]
                    
        #print map(lambda a: numpy.average(a,0),data_flow_split)
        #print map(lambda a: numpy.average(a,0, weights=1*(a[...,1]>10)),data_flow_split)
        data_flow_ang = [numpy.average(a) for a in data_flow_mags]
        data_flow_mag = [numpy.average(a) for a in data_flow_angs]
            
        self._prev = cv_image    
        

        self._motion_pub.publish(Float64MultiArray(data=data_movement))

        if len(move)>0:
            self._flow_ang_pub.publish(Float64MultiArray(data=data_flow_ang))
            self._mag_pub.publish(Float64MultiArray(data=data_flow_mag))
            self._acc_pub.publish(Float64MultiArray(data=[data_flow_mag[i]-self._prev_mag[i] for i in range(len(data_flow_mag))]))
            self._mag_total_pub.publish(Float64MultiArray(data=[total_mag]))
            self._ang_total_pub.publish(Float64MultiArray(data=[total_ang]))
            self._prev_mag = data_flow_mag

if __name__=="__main__":
    rospy.init_node("background_subtraction_features")
    parser=argparse.ArgumentParser(description="Use opencv to get motion features from video")
    parser.add_argument('-d', '--display-video', help="Show the masked video on screen", action='store_true')
    args = parser.parse_known_args()[0]
    
    c = CVImageSubscriber(args.display_video)
    rospy.spin()
