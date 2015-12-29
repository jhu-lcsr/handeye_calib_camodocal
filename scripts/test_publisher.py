#!/usr/bin/env python

import rospy;
from trajectory_msgs.msg import *
from vrep_common.msg import *
from vrep_common.srv import *

rospy.init_node('ros_test_publisher')

getHandle = rospy.ServiceProxy('vrep/simRosGetObjectHandle',simRosGetObjectHandle)
names = ['LBR_iiwa_14_R820_joint1',
 'LBR_iiwa_14_R820_joint2',
 'LBR_iiwa_14_R820_joint3',
 'LBR_iiwa_14_R820_joint4',
 'LBR_iiwa_14_R820_joint5',
 'LBR_iiwa_14_R820_joint6',
 'LBR_iiwa_14_R820_joint7']

handles = []
for name in names:
  handles.append(getHandle(name).handle)


rate = rospy.Rate(10)
while not rospy.is_shutdown():
  msg = JointSetStateData()
  msg.handles = handles;
  

  rate.sleep()

