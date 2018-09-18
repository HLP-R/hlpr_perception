#!/usr/bin/env python

from hlpr_perception_msgs.msg import LabeledObjects
import rospy
import tf

class ObjectsTF:
    def __init__(self):
        rospy.init_node('hlpr_color_object_viz')
        
        self.labels_topic = rospy.get_param('~labels_topic', '/beliefs/labels')
        self.tf = None

    def get_pose(self, obj):
        return obj.obb.bb_center, obj.obb.bb_rot_quat

    def publish_objects(self, msg):
        for obj, label in zip(msg.objects, msg.labels):
            p, q = get_pose(obj)
            self.tf.sendTransform((p.x, p.y, p.z),
                                  (p.x, p.y, p.z, p.w),
                                  msg.header.stamp,
                                  label,
                                  msg.header.frame_id)
    def run(self):
        self.tf = tf.TransformBroadcaster()
        rospy.Subscriber(self.labels_topic, LabeledObjects, self.publish_objects) 
        rospy.spin()

def main():
    objects_tf = ObjectsTF()
    objects_tf.run()

if __name__ == '__main__':
    main()

