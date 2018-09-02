#!/usr/bin/env python

from hlpr_perception_msgs.msg import ExtractedFeaturesArray, ObjectFeatures, LabeledObjects
from std_msgs.msg import String
from object_filters import *
import rospy

class ObjectsLabeler:
    def __init__(self):
        rospy.init_node('hlpr_color_object_labeler')
 
        self.features_topic = rospy.get_param('~features_topic', '/beliefs/features')
        self.labels_topic = rospy.get_param('~labels_topic', '/beliefs/labels')
        self.obj_filters = object_filters_from_yaml(rospy.get_param('~task_yaml'))
        self.labels_pub = None

    def publish_labels(self, msg):
        labels = LabeledObjects()
        labels.header = msg.header

        for name, obj_filter in self.obj_filters.iteritems():
            obj_idx = obj_filter(msg)
            if obj_idx:
                obj = msg.objects[obj_idx[0]]
                labels.objects.append(obj)
                labels.labels.append(String(name))

        if len(labels.objects) > 0: 
            self.labels_pub.publish(labels)

    def run(self):
        rospy.Subscriber(self.features_topic, ExtractedFeaturesArray, self.publish_labels)
        self.labels_pub = rospy.Publisher(self.labels_topic, LabeledObjects, queue_size=10)
        
        rospy.spin()

def main():
    object_labeler = ObjectsLabeler()
    object_labeler.run()

if __name__ == '__main__':
    main()

