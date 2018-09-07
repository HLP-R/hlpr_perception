#!/usr/bin/env python

from hlpr_perception_msgs.msg import LabeledObjects
from visualization_msgs.msg import MarkerArray, Marker
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import ColorRGBA
import rospy

class ObjectsViz:
    def __init__(self):
        rospy.init_node('hlpr_color_object_viz')
        
        self.labels_topic = rospy.get_param('~labels_topic', '/beliefs/labels')
        self.viz_pub = None

    def get_object_marker(self, name, obj):
        marker = Marker()

        marker.header = obj.header
        marker.ns = name
        marker.id = 0
        marker.type = Marker.CUBE

        marker.pose.position = Point(obj.obb.bb_center.x,
                                     obj.obb.bb_center.y,
                                     obj.obb.bb_center.z)
        marker.pose.orientation = obj.obb.bb_rot_quat

        marker.scale = Vector3(obj.obb.bb_dims.x,
                               obj.obb.bb_dims.y,
                               obj.obb.bb_dims.z)

        marker.color = obj.basicInfo.rgba_color

        return marker

    def get_object_name_marker(self, name, obj):
        marker = Marker()

        marker.header = obj.header
        marker.ns = name + '_text'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING

        marker.pose.position = Point(obj.obb.bb_center.x - obj.obb.bb_dims.x/2., 
                                     obj.obb.bb_center.y - obj.obb.bb_dims.y/2.,
                                     obj.obb.bb_center.z - obj.obb.bb_dims.z/2.)

        marker.scale.z = 0.05
        marker.color = ColorRGBA(1,1,1,1)

        marker.text = name

        return marker

    def publish_objects(self, msg):
        markers = []

        for obj, label in zip(msg.objects, msg.labels):
            markers.append(self.get_object_marker(label.data, obj))
            markers.append(self.get_object_name_marker(label.data, obj))
 
        self.viz_pub.publish(MarkerArray(markers))

    def run(self):
        rospy.Subscriber(self.labels_topic, LabeledObjects, self.publish_objects)
        self.viz_pub = rospy.Publisher('visualization_marker_array', MarkerArray, queue_size=10)
        
        rospy.spin()

def main():
    objects_viz = ObjectsViz()
    objects_viz.run()

if __name__ == '__main__':
    main()

