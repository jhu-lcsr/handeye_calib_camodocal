#!/usr/bin/env python

"""
VREP ROS ARM CONTROLLER TEST PUBLISHER
Set up with hard coded kuka joints for now from robone tracker sim setup
"""

import rospy;
from trajectory_msgs.msg import *
from vrep_common.msg import *
from vrep_common.srv import *
import geometry_msgs.msg
import copy
import tf

rospy.init_node('ros_test_publisher')
listener = tf.TransformListener()
getJointStates = rospy.ServiceProxy('vrep/simRosGetJointState',simRosGetJointState)
setJointStates = rospy.ServiceProxy('vrep/simRosSetJointState',simRosSetJointState)
setObjectPoses = rospy.ServiceProxy('vrep/simRosSetObjectPose',simRosSetObjectPose)

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
ARTagHandle = getHandle("Ros_tag").handle
CameraHandle = getHandle("OpticalTrackerBase#0").handle

while not rospy.is_shutdown():
  #msg = JointSetStateData()
  #msg.handles = handles;
  #msg.setModes = [0]*len(handles)
  #msg.values = positions
  try:
    now = rospy.Time.now()
    arTag = "camera_2/ar_marker_0"
    cameraTF = "camera_2/camera_link"
    listener.waitForTransform(arTag,cameraTF,now,rospy.Duration(0.5))
    transform = listener.lookupTransform(arTag,cameraTF,now)
    
    setObjectPoses(handles=CameraHandle, relativeToObjetHandle=ARTagHandle, pose=transform)
  except:
    print "Warning: Transform not found"
    #TODO make the vrep not using this position
    continue

  vals = copy.copy(positions)

  positions[joint_to_move]+=delta;
  if (positions[joint_to_move] >  0.2 and delta > 0) or (positions[joint_to_move] < -0.2 and delta < 0):

    delta *=-1;

  setJointStates(handles=handles,setModes=([0]*len(handles)),values=positions)

  rate.sleep()

