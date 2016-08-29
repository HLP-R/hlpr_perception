#!/usr/bin/env python

import os
import sys, time, math, cmath
from std_msgs.msg import String
import numpy as np
import cv2
import roslib
import rospy
import pdb
from Tkinter import *
from hlpr_feature_extraction.msg import PcFeatureArray

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
	rospy.init_node('tracker', anonymous=True)
        self.subscriber = rospy.Subscriber("/beliefs/features", PcFeatureArray, self.cbClusters, queue_size = 1)
	self.initialized = False
	self.labeled = False
	self.labeledIdx = 0
	self.initX = []
	self.finished = False
	self.filename = os.path.expanduser(get_param("feature_file_location", "tracked_object_data.txt"))
	self.minSize = get_param("min_object_size", 0.001)
	self.outf = open(self.filename, "w")

    def cbClusters(self, ros_data):
	clusterArr = ros_data
        clusters = ros_data.objects
	if len(clusters) is 0:
	    return
        if self.initialized is False:
	    self.initX = []
	    for c in clusters:
		size = c.bb_dims.x * c.bb_dims.y
		print 'Object size: ' + str(size)
		if size > self.minSize:
		    self.initX.append(c)
	    if len(self.initX) is 0:
	        return
	    print str(len(self.initX)) + ' objects detected'
	    self.initialized = True
	elif self.labeled is False:
	    var = raw_input("Enter label for object " + str(self.labeledIdx) + ": ")
	    print "You entered: ", var
	    c = self.initX[self.labeledIdx]
	    r = c.rgba_color.r
	    g = c.rgba_color.g
	    b = c.rgba_color.b
	    hsv = cv2.cvtColor(np.array([[(r,g,b)]],dtype='float32'), cv2.COLOR_RGB2HSV)

	    size = c.bb_dims.x * c.bb_dims.y
	    self.outf.write(var + "," + str(hsv[0][0][0]) + "," + str(hsv[0][0][1]) + "," + str(hsv[0][0][2]) + "," + str(size) + "\n")
	    self.labeledIdx = self.labeledIdx + 1
            self.labeled = self.labeledIdx == len(self.initX)
	elif self.finished is False:
	    self.outf.close()
	    self.finished = True
	elif self.finished is True:
	    print "Objects written to " + str(self.filename)
	    sys.exit()

class ui:
    def __init__(self):
	self.master = Tk()
	self.canvas = Canvas(self.master,width=800,height=500)
	self.canvas.pack()

    def startDrawing(self,tracker):
	self.drawClusters(tracker.initX)
	self.master.after(10, self.startDrawing, tracker)

    def drawClusters(self,clusters):
	if clusters is None:
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
	    label = self.canvas.create_text((-c.points_centroid.x+0.5)*500, (-c.points_centroid.y + 0.5)*500,text=str(idx),font="Verdana 10 bold")
	    self.canvas.pack()

def main(args):
    global pf, display
    pf = filter()
    display = ui()
    display.master.after(10,display.startDrawing,pf)
    display.master.mainloop()

if __name__ == '__main__':
    main(sys.argv)
