#!/usr/bin/env python

import os
import sys, time, math, cmath
from std_msgs.msg import String
import numpy as np
import cv2
import roslib
import rospy
import pdb
import tf
from Tkinter import *
from feature_extraction.msg import PcFeatureArray
from hlpr_object_labeling.msg import LabeledObjects
from std_msgs.msg import String

pf = None
display = None
initX = None

hueW = 2 	# 0-180
satW = 1 	# 0-255
valW = 1	# 0-255
sizeW = 50000	# sq m
distW = 300	# m
threshold = 200
minSize = 0.01
filename = None 

def hsvDiff(c1,c2):
    hsv1 = c1[1:4]

    r2 = c2.rgba_color.r
    g2 = c2.rgba_color.g
    b2 = c2.rgba_color.b
    hsv2 = cv2.cvtColor(np.array([[(r2,g2,b2)]],dtype='float32'), cv2.COLOR_RGB2HSV)

    return abs(hsv2[0][0][0]-float(hsv1[0])), abs(hsv2[0][0][1]-float(hsv1[1])), abs(hsv2[0][0][2]-float(hsv1[2]))

def locDiff(c1,c2):
    return math.sqrt((c2.points_centroid.x-c1.points_centroid.x)**2 + (c2.points_centroid.y-c1.points_centroid.y)**2 + (c2.points_centroid.z-c1.points_centroid.z)**2)

def sizeDiff(c1,c2):
    size = c2.bb_dims.x * c2.bb_dims.y
    return abs(size - float(c1[4]))

def E(init, prev, cluster):
    if prev is None:
	dist = 0
    else:
	dist = distW * locDiff(prev,cluster)
    size = sizeW * sizeDiff(init,cluster)
    hueDiff, satDiff, valDiff = hsvDiff(init,cluster)
    hue = hueW * hueDiff
    sat = satW * satDiff
    val = valW * valDiff
    total = float(hue + sat + val + dist + size)
    return total

def getMatchingLabel(allLabels, labels, cluster):
    minError = -1
    match = None
    idx = 0
    for c in allLabels:
        e = E(c, None, cluster)
        if match is None or e < minError:
            match = c
	    label = labels[idx]
            minError = e
	idx = idx + 1
    return cluster, label, minError

class filter:
    def __init__(self):
#	rospy.init_node('labeling', anonymous=True)
        self.subscriber = rospy.Subscriber("/beliefs/features", PcFeatureArray, self.cbClusters, queue_size = 1)
	self.orderPub = rospy.Publisher("/beliefs/labels", LabeledObjects)
        self.labeled = None
        self.tracked = None
	self.errors = None
        self.ids = None
	self.labels = None
	self.initialized = False
	self.br = tf.TransformBroadcaster()

    def run_filter(self, initX, initLabels, clusters):
        ordered = []
        tracked = []
	errors = []
	ids = []
        count = len(clusters)
	isEmpty = True
        for i in range(0,count):
       	    match,label,error = getMatchingLabel(initX,initLabels,clusters[i])
	    if error <= threshold:
		isEmpty = False
	        ordered.append(match)
	        tracked.append(match)
		errors.append(error)
		string = String()
		string.data = label
		ids.append(string)
  	    else:
	        ordered.append(None)
		errors.append(0.0)
	if isEmpty is True:
	    return None, None, errors, None 
	else:
            return ordered,tracked,errors,ids

    def cbClusters(self, ros_data):
	clusterArr = ros_data
        clusters = ros_data.objects
        transforms = ros_data.transforms
        if self.initialized is False:
	    self.initX = []
	    self.labels = []
	    objFile = open(filename, 'r')
	    for line in objFile.readlines():
		self.initX.append(line.split(','))
		self.labels.append(line.split(',')[0])
	    print str(len(self.initX)) + ' objects loaded'
	if self.labeled is None:
	    #self.labeled = self.initX
	    self.initialized = True
	self.labeled, self.tracked, self.errors, self.ids = self.run_filter(self.initX, self.labels, clusters)
	if len(clusters) is 0 or self.tracked is None:
	    return
	outMsg = LabeledObjects()
	msgTime = rospy.Time.now()
	outMsg.header.stamp = msgTime
	outMsg.objects = self.tracked
	outMsg.labels = self.ids
	self.orderPub.publish(outMsg)
	idx = 0
        for l in self.ids:
          t = None
          for i in range(len(clusters)):
	    if clusters[i] is self.tracked[idx]:
		t = transforms[i]
	  tl = (t.translation.x, t.translation.y, t.translation.z)
	  r = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
	  #self.br.sendTransform(tl, r, rospy.Time.now(), 'kinect_ir_optical_frame', l.data)
	  self.br.sendTransform(tl, r, msgTime, l.data, 'kinect_ir_optical_frame')
	  idx += 1
	  
	#self.idPub.publish(self.ids)

class ui:
    def __init__(self):
	self.master = Tk()
	self.canvas = Canvas(self.master,width=800,height=500)
	self.canvas.pack()

    def startDrawing(self,labeling):
	self.drawClusters(labeling.tracked,labeling.ids)
	self.master.after(10, self.startDrawing, labeling)

    def drawClusters(self,clusters,ids):
	if clusters is None or ids is None:
	    return
	self.canvas.delete("all")
	for idx in range(0,len(clusters)):
	    c = clusters[idx]
	    if c is None:
		continue
	    pts = [(c.points_min.x,c.points_min.y),(c.points_min.x,c.points_max.y),(c.points_max.x,c.points_max.y),(c.points_max.x,c.points_min.y)]
	    offset = complex(c.points_centroid.x,c.points_centroid.y)
	    cangle = 0 # cmath.exp(c.angle*1j)
	    rot = []
	    for x,y in pts:
		r = cangle * (complex(x,y)-offset) + offset
		rot.append((-r.real + 0.5) * 500)
		rot.append((-r.imag + 0.5) * 500)
 	    rgb = '#%02x%02x%02x' % (c.rgba_color.r,c.rgba_color.g,c.rgba_color.b)
	    poly = self.canvas.create_polygon(rot,outline=rgb,fill='white',width=5)
	    label = self.canvas.create_text((-c.points_centroid.x+0.5)*500, (-c.points_centroid.y + 0.5)*500,text=str(ids[idx].data),font="Verdana 10 bold")
	    self.canvas.pack()

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

def main(args):
    global pf, display, filename
    filename = get_param("labeling_data_loc")
    print "reading from " + filename

    pf = filter()
    display = ui()
    display.master.after(10,display.startDrawing,pf)
    display.master.mainloop()

if __name__ == '__main__':
    #print "here"
    rospy.init_node("object_labeling", anonymous=False)
    rospy.loginfo("Initializing the object labeling node")

    main(sys.argv)
