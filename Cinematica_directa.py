#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

from markers import *
from funciones import *

if __name__ == '__main__':

  rospy.init_node("testForwardKinematics")
  pub = rospy.Publisher('joint_states', JointState, queue_size=1)
  bmarker = BallMarker(color['GREEN'])

  # Joint names
  jnames = ['joint1', 'joint2','joint3', 'joint4', 'joint5','joint6','joint7']
  # Joint Configuration
  #q = [0.8, 0.6, 0.8, 0.6, 0.8, 0.6, 0.6, 0.0]
  #q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  #q = [0,0,0,0,0,0,0]
  q = [0.8, 0.6, 0.8, 1.8, 0.8, 0.8, 0.0]
  # End effector with respect to the base
  T = fkine_kuka(q)
  print( np.round(T, 3) )
  bmarker.position(T)

  # Object (message) whose type is JointState
  jstate = JointState()
  # Set values to the message
  jstate.header.stamp = rospy.Time.now()
  jstate.name = jnames
  # Add the head joint value (with value 0) to the joints
  jstate.position = q

  # Loop rate (in Hz)
  rate = rospy.Rate(20)
  # Continuous execution loop
  while not rospy.is_shutdown():
    # Current time (needed for ROS)     
    jstate.header.stamp = rospy.Time.now()
    # Publish the message
    pub.publish(jstate)
    bmarker.publish()
    # Wait for the next iteration
    rate.sleep()