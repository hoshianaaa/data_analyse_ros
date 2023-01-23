#!/usr/bin/env python

# peak detection: https://qiita.com/wrblue_mica34/items/e174a71570abb710dcfb

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import *
from scipy import signal
import numpy as np

sensitivity_ = 50

def callback(msg):
  global sensitivity_

  maxid_list = []

  y = np.array(msg.data)

  maxid = signal.argrelmax(y, order=1) #最大値
  minid = signal.argrelmin(y, order=1) #最小値

  maxid = maxid[0].tolist()
  minid = minid[0].tolist()

  for i in maxid:
    if y[i] > sensitivity_*255/100:
      maxid_list.append(i)

  max_id_msg = Int32MultiArray()
  min_id_msg = Int32MultiArray()

  max_id_msg.data = maxid_list
  min_id_msg.data = minid

  max_pub.publish(max_id_msg)
  min_pub.publish(min_id_msg)

def sensitivity_callback(msg):
#  print(msg)
  global sensitivity_
  sensitivity_ = msg.data

rospy.init_node("peak_detection")
max_pub = rospy.Publisher("~peak_max_index", Int32MultiArray, queue_size=10)
min_pub = rospy.Publisher("~peak_min_index", Int32MultiArray, queue_size=10)
sensitivity_state_pub = rospy.Publisher("/peak_detection/sensitivity_state", Float64, queue_size=10)
rospy.Subscriber("/edge_detection/edge_graph", Float32MultiArray, callback)
rospy.Subscriber("/edge_detection/edge_sensitivity", Float64, sensitivity_callback)

r = rospy.Rate(10)

while not rospy.is_shutdown():
  sensitivity_state_pub.publish(Float64(sensitivity_))
  print(sensitivity_)
  r.sleep()
