#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
import time
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *

# Rotation from joint_3 to joint_0
R3_0 = None

# evaluate arcus cosinus with arcus tangens
arccos = lambda D: atan2(sqrt(1 - D**2), D)

# cosine law
# returns the cosinus of the angle between a,b
# c denotes the side opposite to the angle
cosine_law = lambda c, a, b: (a**2 + b**2 - c**2) / (2 * a * b)

# Symbols for Denavit-Hartenberg parameters
q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # joint angles (theta_i)         
d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offsets
a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link lengths
alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angles

# Denavit-Hartenberg parameters for KUKA KR210
s = {alpha0:     0,  a0:        0, d1:  0.75,
     alpha1: -pi/2,  a1:     0.35, d2:     0, q2: q2-pi/2, 
     alpha2:     0,  a2:     1.25, d3:     0,
     alpha3: -pi/2,  a3:   -0.054, d4:   1.5,
     alpha4:  pi/2,  a4:        0, d5:     0,
     alpha5: -pi/2,  a5:        0, d6:     0,
     alpha6:     0,  a6:        0, d7: 0.303, q7:       0}

def forward_kinematics():
    """
        Generates all constant transformation matrices that are used by the service.
    """

    # measure the execution time
    print("Generating transformation matrices. Please wait...")
    start = time.clock()
    
    # Homogeneous Transforms between subsequent DH frames
    T0_1 = Matrix([[             cos(q1),            -sin(q1),            0,              a0],
                   [ sin(q1)*cos(alpha0), cos(q1)*cos(alpha0), -sin(alpha0), -sin(alpha0)*d1],
                   [ sin(q1)*sin(alpha0), cos(q1)*sin(alpha0),  cos(alpha0),  cos(alpha0)*d1],
                   [                   0,                   0,            0,               1]])
    T0_1 = T0_1.subs(s)

    T1_2 = Matrix([[             cos(q2),            -sin(q2),            0,              a1],
                   [ sin(q2)*cos(alpha1), cos(q2)*cos(alpha1), -sin(alpha1), -sin(alpha1)*d2],
                   [ sin(q2)*sin(alpha1), cos(q2)*sin(alpha1),  cos(alpha1),  cos(alpha1)*d2],
                   [                   0,                   0,            0,               1]])
    T1_2 = T1_2.subs(s)

    T2_3 = Matrix([[             cos(q3),            -sin(q3),            0,              a2],
                   [ sin(q3)*cos(alpha2), cos(q3)*cos(alpha2), -sin(alpha2), -sin(alpha2)*d3],
                   [ sin(q3)*sin(alpha2), cos(q3)*sin(alpha2),  cos(alpha2),  cos(alpha2)*d3],
                   [                   0,                   0,            0,               1]])
    T2_3 = T2_3.subs(s)
    
    # Transform from base_link to link_3
    T0_3 = trigsimp(T0_1 * T1_2 * T2_3)
    
    # Rotation from base_link to link_3
    R0_3 = T0_3[0:3, 0:3]
    
    # Rotation from link_3 to base_link
    R0_3_T = R0_3.transpose()

    print("Generating transformation matrices took {:6.2f} seconds.".format(time.clock() - start))
    return R0_3_T


def inverse_kinematics(p_in, R_in):
    """
    Calculate joint angles with inverse kinematics.
    """
    # convert inputs to sympy matrices
    p = Matrix(p_in)
    R = Matrix(R_in)
    
    ### WRIST CENTER ###

    # wrist center vector w in frame 0, s[d7] = 0.303 from DH table
    w = p - s[d7] * R[:, 0]

    # Calculate wrist center in xy-plane coordinates (xc, yc) from base frame coordinates w.
    # Then we evaluate a sss triangle in xy-plane, see writeup.
    # The sides of the triangle are l23, l25 and l35.
    # The relevant angles are phi2 between l23 and l25, and phi3 between l23 and l35.

    # xc is projection of wrist center in (X_0, Y_0) plane minus offset from joint 2
    xc = norm([w[0], w[1]]) - s[a1]

    # yc is w component in Z_0 direction minus offset from joint 2
    yc = w[2] - s[d1]

    # Calculate distances between joints
    l25 = norm([xc, yc])       # distance between wrist center w and joint 2
    l23 = s[a2]                # distance between joints 2 and 3, see DH table
    l35 = norm([s[a3], s[d4]]) # distance between joints 3 and 5, see DH table

    ### THETA 1 ###

    theta1 = atan2(p[1], p[0]).evalf()


    ### THETA 3 ###

    # Calculate phi3 (<l23,l35) with cosine law, D3 = cos(phi3)
    D3 = cosine_law(l25, l23, l35)
    phi3 = arccos(D3)

    # Offset for theta3 due to arm design
    delta = atan2(s[a3], s[d4])

    # theta3 from angle phi3 and offset delta
    theta3 = (pi/2 - phi3 - delta).evalf()


    ### THETA 2 ###

    # Calculate phi2 (<l23,l25) with cosine law, D2 = cos(phi2)
    D2 = cosine_law(l35, l23, l25)
    phi2 = arccos(D2)

    # alpha is the angle between x-axis and the line l25 in the xy-plane
    alpha = atan2(yc, xc)

    # theta3 from angle phi2 and angle alpha
    theta2 = (pi/2 - phi2 - alpha).evalf()

    ### THETA 4, 5, 6 ###

    # Evaluate wrist center rotation matrix using calculated angles
    R3_0_eval = R3_0.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

    # Rotation matrix from wrist to gripper
    R3_6 = R3_0_eval * R

    # Entries of the rotation matrix
    r11 = R3_6[0,0]
    r21 = R3_6[1,0]
    r31 = R3_6[2,0]
    r32 = R3_6[2,1]
    r33 = R3_6[2,2]

    # Euler angles, cf. conventions in writeup    
    theta6 = atan2( r21, r11).evalf()                    # alpha, rotation about z-axis
    theta5 = atan2(-r31, sqrt(r11**2 + r21**2)).evalf()  # beta,  rotation about y-axis
    theta4 = atan2( r32, r33).evalf()                    # gamma, rotation about x-axis

    return [theta1, theta2, theta3, theta4, theta5, theta6] 


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Initialize service response
        joint_trajectory_list = []
        for index in xrange(0, len(req.poses)):
            
            # Extract pose from service request
            pose = req.poses[index]
            
            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            p = [pose.position.x, pose.position.y, pose.position.z]

            # quaternion representation for end effector orientation
            quaternion = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
            
            # Rotation matrix from request 
            R_rpy = tf.transformations.quaternion_matrix(quaternion)[0:3, 0:3]
           
            # Populate response for the IK request            
            joint_trajectory_point = JointTrajectoryPoint()            
            joint_trajectory_point.positions = inverse_kinematics(p, R_rpy) # call inverse kinematics function
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("Length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    global R3_0
    
    # Initialize node
    rospy.init_node('IK_server')
    
    # generate transformation matrices
    R3_0 = forward_kinematics()
    
    # Declare calculate_ik service
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
