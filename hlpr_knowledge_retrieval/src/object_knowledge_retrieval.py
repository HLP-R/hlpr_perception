#!/usr/bin/env python

import os
import sys, time, math, cmath
from std_msgs.msg import String
import numpy as np
import roslib
import rospy

from hlpr_object_labeling.msg import LabeledObjects
from hlpr_knowledge_retrieval.msg import ObjectKnowledge, ObjectKnowledgeArray

def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

class lookup:
    def __init__(self):
        self.subscriber = rospy.Subscriber("/beliefs/labels", LabeledObjects, self.cbLabels, queue_size = 1)
	self.knowPub = rospy.Publisher("/beliefs/knowledge", ObjectKnowledge)

        fileref = get_param("data_file_location")
        if fileref is not None:
          self.filename = os.path.expanduser(fileref)
        else:
          self.filename = None
        topicref = get_param("data_file_rostopic")
        if topicref is not None:
          self.rostopic = os.path.expanduser(topicref)
          self.fileSub = rospy.Subscriber(self.rostopic, String, self.cbFile, queue_size = 1)

    def cbFile(self, ros_data):
        if self.filename is not ros_data.data:
          self.filename = ros_data.data
	  self.readObjectKnowledge(self.filename)
          self.initialized = True
          print "Reading knowledge data from " + self.filename

    def readObjectKnowledge(self, filename):
      lines = None
      with open(filename) as f:
	lines = f.readlines()
      dictionaries = []
      knowledgeTypes = []
      for l in lines:
	newDict = dict()
	lineList = l[:-1].split(";")
	knowledgeTypes.append(lineList[0])
	for o in lineList[1:]:
	  ftList = o.split(":")[1]
	  newDict[o.split(":")[0]] = ftList
	dictionaries.append(newDict)
      self.knowledgeTypes = knowledgeTypes
      self.dictionaries = dictionaries

    def cbLabels(self, ros_data):
	labels = ros_data.labels
	if labels is None or self.filename is None:
	    return
	
	messages = []
	idx = 0
	for kType in self.knowledgeTypes:
	    #affList = affList + self.affDictionary[label.rsplit(None,1)[-1]] + ','
	    message = [] 
	    for label in labels:
		msg = String()
		if label.data in self.dictionaries[idx]:
		  msg.data = self.dictionaries[idx][label.data]
		  message.append(msg)
		else:
		  print "Object " + label.data + " not found in knowledge source"
	    idx += 1
	    arrMsg = ObjectKnowledgeArray()
	    arrMsg.data = message
	    messages.append(arrMsg)
        kmsg = ObjectKnowledge()
        kmsg.knowledge = messages
        kmsg.labels = labels
	self.knowPub.publish(kmsg)
def get_param(name, value=None):
    private = "~%s" % name
    if rospy.has_param(private):
        return rospy.get_param(private)
    elif rospy.has_param(name):
        return rospy.get_param(name)
    else:
        return value

def main(args):
    rospy.init_node('knowledge_retrieval', anonymous=True)
    lkup = lookup()
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
