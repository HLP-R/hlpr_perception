#!/usr/bin/env python
import rospy
import sys
import cv2
from std_msgs.msg import String, Float32MultiArray, Bool
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, PointCloud2, CompressedImage
from hlpr_face_detection.msg import FaceDetectionTopic
import face_recognition

class FaceDetector:
  def __init__(self, tgtdir='.'):
    print "Starting Init for Face Detector"
    self.face_detector_pub = rospy.Publisher("face_detections",FaceDetectionTopic,queue_size=10)
    # rospy.sleep(2)
    self.bridge = CvBridge()
    self.rgb_image = None
    self.tgtdir = tgtdir
    image_topic = rospy.get_param("/image_topic_name")
    detection_sub = rospy.Subscriber(image_topic, Image, self.callback, queue_size=1, buff_size=52428800)

    rospy.sleep(5)
    print "Finished Init for Face Detector"


  def callback(self,rgb):
    self.rgb_image = rgb
    print "callback"
    cv_image = self.bridge.imgmsg_to_cv2(rgb, "bgr8")

    faces = self.detect_faces(cv_image)
    msg = FaceDetectionTopic()
    msg.faces = faces
    # print "faces {0}".format(faces)
    self.face_detector_pub.publish(msg)


  def detect_faces(self, img):
    use_gpu = rospy.get_param("/use_gpu")
    if(use_gpu):
      faces = face_recognition.face_locations(img, number_of_times_to_upsample=1, model="cnn")
    else:
      faces = face_recognition.face_locations(img, number_of_times_to_upsample=1)
    multi_array = []
    for face in faces:
      face_array = Float32MultiArray()
      face_array.data = face
      multi_array.append(face_array)

    return multi_array

if __name__ == '__main__':
  rospy.init_node('face_detector', anonymous=True)
  obj = FaceDetector()
  while not rospy.is_shutdown():
    rospy.spin()

