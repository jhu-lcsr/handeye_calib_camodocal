#!/usr/bin/env python

import rospy;
from trajectory_msgs.msg import *
from vrep_common.msg import *
from vrep_common.srv import *
import copy

rospy.init_node('ros_test_publisher')

getJointStates = rospy.ServiceProxy('vrep/simRosGetJointState',simRosGetJointState)
setJointStates = rospy.ServiceProxy('vrep/simRosSetJointState',simRosSetJointState)
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

positions = []
for handle in handles:
  joint = getJointStates(handle)
  print joint
  positions.append(joint.state.position[0])

print positions

delta = 0.005
joint_to_move = 1

rate = rospy.Rate(30)
while not rospy.is_shutdown():
  #msg = JointSetStateData()
  #msg.handles = handles;
  #msg.setModes = [0]*len(handles)
  #msg.values = positions

  vals = copy.copy(positions)

  positions[joint_to_move]+=delta;
  if (positions[joint_to_move] >  0.2 and delta > 0) or (positions[joint_to_move] < -0.2 and delta < 0):

    delta *=-1;

  setJointStates(handles=handles,setModes=([0]*len(handles)),values=positions)

  rate.sleep()

