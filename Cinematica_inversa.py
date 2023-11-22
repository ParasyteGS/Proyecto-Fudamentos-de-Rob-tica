#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState

from markers import *
from funciones import *

if __name__ == '__main__':

    rospy.init_node("testInvKine")
    pub = rospy.Publisher('joint_states', JointState, queue_size=1)

    bmarker      = BallMarker(color['RED'])
    bmarker_des  = BallMarker(color['GREEN'])

    # Joint names
    jnames = ['joint1', 'joint2','joint3', 'joint4', 'joint5','joint6','joint7']

    # Desired position
    #xd = np.array([0.84, 0.125, 0.249])
    #xd = np.array([0.44, 0.22, 0.5])
    xd = np.array([-0.44, 0.6, 0.8])
    # Initial configuration
    q0 = np.array([0.0, 0.8, 0.7, -0.2, -0.6, 0.2, 0.0])
    # Inverse kinematics
    #q = ikine_kuka(xd, q0)          # Método de Newton
    q = ik_gradient_kuka(xd, q0)     # Método de la Gradiente

    # Resulting position (end effector with respect to the base link)
    T = fkine_kuka(q)
    print('Obtained value:\n', np.round(T,3))

    # Red marker shows the achieved position
    bmarker.xyz(T[0:3,3])
    # Green marker shows the desired position
    bmarker_des.xyz(xd)

    # Objeto (mensaje) de tipo JointState
    jstate = JointState()
    # Asignar valores al mensaje
    jstate.header.stamp = rospy.Time.now()
    jstate.name = jnames
    # Add the head joint value (with value 0) to the joints
    jstate.position = q

    # Loop rate (in Hz)
    rate = rospy.Rate(100)
    
    # Continuous execution loop
    while not rospy.is_shutdown():
        # Current time (needed for ROS)
        jstate.header.stamp = rospy.Time.now()
        # Publish the message
        pub.publish(jstate)
        bmarker.publish()
        bmarker_des.publish()
        # Wait for the next iteration
        rate.sleep()