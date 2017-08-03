#!/usr/bin/env python

import os
import sys, time, math, cmath
from std_msgs.msg import String, Header
import numpy as np
import cv2
import roslib
import rospy
import pdb
import tf
import itertools
from Tkinter import *
from hlpr_perception_msgs.msg import ObjectFeatures, ExtractedFeaturesArray, LabeledObjects
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
        self.labeled = None
        self.tracked = None
	self.errors = None
        self.ids = None
	self.labels = None
	self.initialized = False
	self.run = True
	self.br = tf.TransformBroadcaster()

	self.hueW = get_param("hsv_hue_weight", 2)
	self.satW = get_param("hsv_sat_weight",1)
	self.valW = get_param("hsv_val_weight",1)
	self.sizeW = get_param("size_weight",50000)
        fileref = get_param("feature_file_location")
	topicref = get_param("feature_file_rostopic")
	self.filename = None
	if topicref is not None:
    	  self.rostopic = os.path.expanduser(topicref)
          self.fileSub = rospy.Subscriber(self.rostopic, String, self.cbFile, queue_size = 1)
	elif fileref is not None:
    	  self.filename = os.path.expanduser(fileref)
        self.subscriber = rospy.Subscriber("/beliefs/features", ExtractedFeaturesArray, self.cbClusters, queue_size = 1)
	self.pauseSub = rospy.Subscriber("/pause_labeling", String, self.cbPause, queue_size = 1)
	self.orderPub = rospy.Publisher("/beliefs/labels", LabeledObjects, queue_size = 1)

    def cbFile(self, ros_data):
	if self.filename is not ros_data.data:
	  self.filename = ros_data.data
	  self.loadObjects()
	  self.initialized = True
          print "Reading object features from " + self.filename

    def cbPause(self, ros_data):
	if ros_data.data == "pause":
	  self.run = False
	if ros_data.data == "play":
	  self.run = True

    def cbClusters(self, ros_data):
	#Wait for filename to be received (if receiving from rostopic)
	if self.filename is None:
	  return

 	if self.run is False:
	  self.pubMessages()
	  return

	#Initialize object feature values
        if self.initialized is False:
          print "Reading object features from " + self.filename
	  self.loadObjects()
	  self.initialized = True

	#Read cluster message
	clusterArr = ros_data
        clusters = ros_data.objects
        self.transforms = ros_data.transforms

	#Classify clusters
	#self.labeled, self.tracked, self.errors, self.ids = self.run_filter(self.initX, self.labels, clusters)
	self.tracked, self.ids, self.error = self.getMatchingLabels(self.initX, self.labels, clusters)

	#Publish labels
	if len(clusters) is 0 or self.tracked is None:
	    return
	self.outMsg = LabeledObjects()
	msgTime = rospy.Time.now()
	head = Header()
	head.stamp = msgTime
	self.outMsg.header = head
	self.outMsg.objects = self.tracked
	self.outMsg.labels = self.ids
	self.pubMessages()

    def pubMessages(self):
	if self.outMsg is None or self.transforms is None or self.ids is None:
	  return

	#Publish labels
	msgTime = rospy.Time.now()
	head = Header()
	head.stamp = msgTime
	self.outMsg.header = head
	self.orderPub.publish(self.outMsg)

	#Publish transforms
	idx = 0
        for l in self.ids:
  	  t = self.transforms[idx]
	  tl = (t.translation.x, t.translation.y, t.translation.z)
	  r = (t.rotation.x, t.rotation.y, t.rotation.z, t.rotation.w)
	  self.br.sendTransform(tl, r, self.outMsg.header.stamp, l.data, 'kinect_ir_optical_frame')
	  idx += 1

    def loadObjects(self):
      self.initX = []
      self.labels = []
      objFile = open(self.filename, 'r')
      for line in objFile.readlines():
	self.initX.append(line[:-1].split(','))
	self.labels.append(line.split(',')[0])
      print str(len(self.initX)) + ' objects loaded'
	  
    def hsvDiff(self, c1,c2):
      	hsv1 = c1[1:4]
      	r2 = c2.basicInfo.rgba_color.r
      	g2 = c2.basicInfo.rgba_color.g
      	b2 = c2.basicInfo.rgba_color.b
   	hsv2 = cv2.cvtColor(np.array([[(r2,g2,b2)]],dtype='float32'), cv2.COLOR_RGB2HSV)
	h1 = hsv2[0][0][0]
	h2 = float(hsv1[0])
	huediff = math.degrees(math.atan2(math.sin(math.radians(h1-h2)), math.cos(math.radians(h1-h2))))
    	#return abs(hsv2[0][0][0]-float(hsv1[0])), abs(hsv2[0][0][1]-float(hsv1[1])), abs(hsv2[0][0][2]-float(hsv1[2]))
    	return abs(huediff), abs(hsv2[0][0][1]-float(hsv1[1])), abs(hsv2[0][0][2]-float(hsv1[2]))

    def sizeDiff(self, c1,c2):
    	size = c2.obb.bb_dims.x * c2.obb.bb_dims.y
    	return abs(size - float(c1[4]))

    def calculateError(self, init, cluster):
    	size = self.sizeW * self.sizeDiff(init,cluster)
    	hueDiff, satDiff, valDiff = self.hsvDiff(init,cluster)
    	hue = self.hueW * hueDiff
    	sat = self.satW * satDiff
    	val = self.valW * valDiff
    	total = float(hue + sat + val + size)
    	return total

    def getMatchingLabels(self, expected, labels, clusters):
	## Evaluate all possible cluster-label pairs
	errorMatrix = []
    	for l in expected:
	  labelErrors = []
	  for c in clusters:
            e = self.calculateError(l, c)
	    labelErrors.append(e)
	  errorMatrix.append(labelErrors)
	## Find the label assignment that minimizes total error
	minMatch = None
	minError = -1
	objList = range(max(len(expected), len(clusters)))
	assn = itertools.permutations(objList, len(clusters))
	for a in assn:
	  e = 0
	  i = 0
	  newAssn = []
	  for idx in a:
	    if idx < len(expected):
	      e += errorMatrix[idx][i]
	      newAssn.append(idx)
	    else:
	      newAssn.append(-1)
	    i += 1
	  if minMatch is None or e < minError:
	    minMatch = newAssn
	    minError = e
	#print "best: " + str(minMatch) + " with error " + str(minError)

	## Convert the best assignment back to a list of labels
	match = []
	ordered = []
	for i in range(len(minMatch)):
	  #if minMatch[i] is -1:
	  #  match.append(None)
	  #  ordered.append(None)
	  #else:
	  if minMatch[i] is not -1:
	    #print str(errorMatrix[minMatch[i]][i]) + " + "
	    match.append(clusters[i])
	    sMsg = String()
	    sMsg.data = labels[minMatch[i]]
	    ordered.append(sMsg)
    	return match, ordered, minError

class ui:
    def __init__(self):
	self.master = Tk()
	self.canvas = Canvas(self.master,width=800,height=500)
	self.canvas.pack()
	self.timeout = 5
	self.waitCount = 0

    def startDrawing(self,labeling):
	self.drawClusters(labeling.tracked,labeling.ids)
	self.master.after(10, self.startDrawing, labeling)

    def drawClusters(self,clusters,ids):
	if clusters is None or ids is None:
	    time.sleep(1.0)
	    return
	self.canvas.delete("all")
	for idx in range(0,len(clusters)):
	    cluster = clusters[idx]
	    if cluster is None:
		continue
	    c = cluster.basicInfo
	    pts = [(c.points_min.x,c.points_min.y),(c.points_min.x,c.points_max.y),(c.points_max.x,c.points_max.y),(c.points_max.x,c.points_min.y)]
	    offset = complex(c.points_centroid.x,c.points_centroid.y)
	    cangle = 0 # cmath.exp(c.angle*1j)
	    rot = []
	    for x,y in pts:
		r = cangle * (complex(x,y)-offset) + offset
		rot.append((r.real + 0.5) * 500)
		rot.append((r.imag + 0.5) * 500)
 	    rgb = '#%02x%02x%02x' % (c.rgba_color.r,c.rgba_color.g,c.rgba_color.b)
	    poly = self.canvas.create_polygon(rot,outline=rgb,fill='white',width=5)
	    label = self.canvas.create_text((c.points_centroid.x+0.5)*500, (c.points_centroid.y + 0.5)*500,text=str(ids[idx].data),font="Verdana 10 bold")
	    #label = self.canvas.create_text((-c.points_centroid.x+0.5)*500, (-c.points_centroid.y + 0.5)*500,text=str(ids[idx].data),font="Verdana 10 bold")
	    self.canvas.pack()

def main(args):
    global pf, display

    pf = filter()
    display = ui()
    display.master.after(10,display.startDrawing,pf)
    display.master.mainloop()

if __name__ == '__main__':
    rospy.init_node("object_labeling", anonymous=False)
    rospy.loginfo("Initializing the object labeling node")

    main(sys.argv)
    rospy.spin()
