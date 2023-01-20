#!/usr/bin/env python

# peak detection: https://qiita.com/wrblue_mica34/items/e174a71570abb710dcfb

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *

def callback(msg):
  print(msg.data)

rospy.init_node("peak_detection")
pub = rospy.Publisher("~peak_max_index", Int32MultiArray, queue_size=10)
pub = rospy.Publisher("~peak_min_index", Int32MultiArray, queue_size=10)
rospy.Subscriber("~data", Float32MultiArray, callback)

r = rospy.Rate(10)

while not rospy.is_shutdown():
  pub.publish(Float64(0))
  r.sleep()
