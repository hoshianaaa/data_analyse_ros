#!/usr/bin/env python

# peak detection: https://qiita.com/wrblue_mica34/items/e174a71570abb710dcfb

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from scipy import signal
import numpy as np

def callback(msg):
  print(msg.data)
  y = np.array(msg.data)

  maxid = signal.argrelmax(y, order=1) #最大値
  minid = signal.argrelmin(y, order=1) #最小値

  max_id_msg = Int32MultiArray()
  min_id_msg = Int32MultiArray()

  max_id_msg.data = maxid[0].tolist()
  min_id_msg.data = minid[0].tolist()

  max_pub.publish(max_id_msg)
  min_pub.publish(min_id_msg)

rospy.init_node("peak_detection")
max_pub = rospy.Publisher("~peak_max_index", Int32MultiArray, queue_size=10)
min_pub = rospy.Publisher("~peak_min_index", Int32MultiArray, queue_size=10)
rospy.Subscriber("/edge_detection/edge_graph", Float32MultiArray, callback)

r = rospy.Rate(10)

while not rospy.is_shutdown():
  r.sleep()
