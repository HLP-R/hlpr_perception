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

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class filter:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/beliefs/features", PcFeatureArray, self.cbClusters, queue_size = 1)
	self.orderPub = rospy.Publisher("/beliefs/labels", LabeledObjects)
        self.labeled = None
        self.tracked = None
	self.errors = None
        self.ids = None
	self.labels = None
	self.initialized = False
	self.br = tf.TransformBroadcaster()

	self.hueW = get_param("hsv_hue_weight", 2)
	self.satW = get_param("hsv_sat_weight",1)
	self.valW = get_param("hsv_val_weight",1)
	self.sizeW = get_param("size_weight",50000)
	self.errorThreshold = get_param("error_threshold",200)
    	self.filename = get_param("feature_file_location")

    def cbClusters(self, ros_data):
	#Initialize object feature values
        if self.initialized is False:
	  self.loadObjects()
	  self.initialized = True

	#Read cluster message
	clusterArr = ros_data
        clusters = ros_data.objects
        transforms = ros_data.transforms

	#Classify clusters
	self.labeled, self.tracked, self.errors, self.ids = self.run_filter(self.initX, self.labels, clusters)

	#Publish labels
	if len(clusters) is 0 or self.tracked is None:
	    return
	outMsg = LabeledObjects()
	msgTime = rospy.Time.now()
	outMsg.header.stamp = msgTime
	outMsg.objects = self.tracked
	outMsg.labels = self.ids
	self.orderPub.publish(outMsg)

	#Publish transforms
	idx = 0
        for l in self.ids:
          t = None
          for i in range(len(clusters)):
	    if clusters[i] is self.tracked[idx]:
		t = transforms[i]
	  tl = (t.translation.x, t.translation.y, t.translation.z)
	  r = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
	  self.br.sendTransform(tl, r, msgTime, l.data, 'kinect_ir_optical_frame')
	  idx += 1

    def run_filter(self, initX, initLabels, clusters):
        ordered = []
        tracked = []
	errors = []
	ids = []
        count = len(clusters)
	isEmpty = True
        for i in range(0,count):
       	    match,label,error = self.getMatchingLabel(initX,initLabels,clusters[i])
	    if error <= self.errorThreshold:
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

    def loadObjects(self):
      self.initX = []
      self.labels = []
      objFile = open(self.filename, 'r')
      for line in objFile.readlines():
	self.initX.append(line.split(','))
	self.labels.append(line.split(',')[0])
      print str(len(self.initX)) + ' objects loaded'
	  
    def hsvDiff(self, c1,c2):
      	hsv1 = c1[1:4]
      	r2 = c2.rgba_color.r
      	g2 = c2.rgba_color.g
      	b2 = c2.rgba_color.b
   	hsv2 = cv2.cvtColor(np.array([[(r2,g2,b2)]],dtype='float32'), cv2.COLOR_RGB2HSV)

    	return abs(hsv2[0][0][0]-float(hsv1[0])), abs(hsv2[0][0][1]-float(hsv1[1])), abs(hsv2[0][0][2]-float(hsv1[2]))

    def sizeDiff(self, c1,c2):
    	size = c2.bb_dims.x * c2.bb_dims.y
    	return abs(size - float(c1[4]))

    def calculateError(self, init, prev, cluster):
    	size = self.sizeW * self.sizeDiff(init,cluster)
    	hueDiff, satDiff, valDiff = self.hsvDiff(init,cluster)
    	hue = self.hueW * hueDiff
    	sat = self.satW * satDiff
    	val = self.valW * valDiff
    	total = float(hue + sat + val + size)
    	return total

    def getMatchingLabel(self, allLabels, labels, cluster):
    	minError = -1
    	match = None
    	idx = 0
    	for c in allLabels:
          e = self.calculateError(c, None, cluster)
          if match is None or e < minError:
            match = c
	    label = labels[idx]
            minError = e
	  idx = idx + 1
    	return cluster, label, minError

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
	    print "Waiting for object messages..."
	    time.sleep(1.0)
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

def main(args):
    global pf, display
    filename = get_param("feature_file_location")
    print "Reading object features from " + filename

    pf = filter()
    display = ui()
    display.master.after(10,display.startDrawing,pf)
    display.master.mainloop()

if __name__ == '__main__':
    rospy.init_node("object_labeling", anonymous=False)
    rospy.loginfo("Initializing the object labeling node")

    main(sys.argv)
