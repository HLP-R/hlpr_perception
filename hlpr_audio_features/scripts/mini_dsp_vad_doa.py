#!/usr/bin/env python
import sys
import rospy
import usb.core
import usb.util

from std_msgs.msg import Bool
from std_msgs.msg import Int32

TIMEOUT = 100#ms

if __name__=="__main__":
    rospy.init_node("minidsp_vad_monitor")

    vad_pub = rospy.Publisher("audio/vad", Bool, queue_size = 1)
    angle_pub = rospy.Publisher("audio/doa/angle", Int32, queue_size = 1)
    mic_pub = rospy.Publisher("audio/doa/mic_id", Int32, queue_size = 1)

    # decimal vendor and product values
    dev = usb.core.find(idVendor=0x2752, idProduct=0x1C)

    # first endpoint
    interface = 4
    endpoint = dev[0][(4,0)][0]
    # if the OS kernel already claimed the device, which is most likely true
    # thanks to http://stackoverflow.com/questions/8218683/pyusb-cannot-set-configuration
    if dev.is_kernel_driver_active(interface) is True:
        # tell the kernel to detach
      dev.detach_kernel_driver(interface)
      # claim the device
      usb.util.claim_interface(dev, interface)

      last_vad = 0
      while not rospy.is_shutdown():
          try:
              data = dev.read(endpoint.bEndpointAddress,endpoint.wMaxPacketSize, TIMEOUT)
              vad = data[2] == 1
              angle = data[3]<<8|data[4]
              mic = data[5]
              last_vad = vad
              last_angle = angle
              last_mic = mic
              vad_pub.publish(Bool(vad))
              angle_pub.publish(Int32(angle))
              mic_pub.publish(Int32(mic))
          except usb.core.USBError as e:
              data = None
              vad_pub.publish(Bool(last_vad))
              if last_vad:
                  angle_pub.publish(Int32(last_angle))
                  mic_pub.publish(Int32(last_mic))
                  
              if e.args[0] == 110:
                  continue
              else:
                  rospy.logerr("Error connecting to miniDSP: {}".format(e))
                  # release the device
    usb.util.release_interface(dev, interface)
    # reattach the device to the OS kernel
    dev.attach_kernel_driver(interface)
