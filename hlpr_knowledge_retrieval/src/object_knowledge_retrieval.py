#!/usr/bin/env python

import os
import sys, time, math, cmath
from std_msgs.msg import String
import numpy as np
import roslib
import rospy

from hlpr_object_labeling.msg import LabeledObjects
from hlpr_knowledge_retrieval.msg import ObjectKnowledge

affFilename = None 
propFilename = None

class lookup:
    def __init__(self):
	self.affDictionary = np.load(affFilename).item()
	self.propDictionary = np.load(propFilename).item()
        self.subscriber = rospy.Subscriber("/beliefs/labels", LabeledObjects, self.cbLabels, queue_size = 1)
	self.knowPub = rospy.Publisher("/beliefs/knowledge", ObjectKnowledge)

    def cbLabels(self, ros_data):
	labels = ros_data.labels
	if labels is None:
	    return
	
	affList = []
	propList = []
	for label in labels:
	    #affList = affList + self.affDictionary[label.rsplit(None,1)[-1]] + ','
	    affStr = String()
	    affStr.data = self.affDictionary[label.data]
	    affList.append(affStr)
	    propStr = String()
	    propStr.data = self.propDictionary[label.data]
	    propList.append(propStr)
        msg = ObjectKnowledge()
        msg.affordances = affList
        msg.properties = propList
        msg.labels = labels
	self.knowPub.publish(msg)
def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

def main(args):
    global lkup, affFilename, propFilename
    rospy.init_node('knowledge_retrieval', anonymous=True)
    filename = get_param("knowledge_data_loc")
    affFilename = filename + "affordance_table.npy"
    propFilename = filename + "property_table.npy"
    print "reading from " + filename
    lkup = lookup()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
