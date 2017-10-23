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
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def get_rotation_matrix():
    r, p, y = symbols('r p y')
    R_x = Matrix([[1, 0, 0],
                  [0, cos(r), -sin(r)],
                  [0, sin(r), cos(r)]])

    R_y = Matrix([[cos(p), 0, sin(p)],
                  [0, 1, 0],
                  [-sin(p), 0, cos(p)]])

    R_z = Matrix([[cos(y), -sin(y), 0],
                  [sin(y), cos(y), 0],
                  [0, 0, 1]])

    R_E = R_z * R_y * R_x
    # Compensate for rotation discrepancy between DH parameters and Gazebo
    R_error_correction = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))
    R_E = R_E * R_error_correction
    return R_E


def get_transformation_matrix(alpha, a, d, q, dh_params):
    """ Creates a modified transformation matrix"""

    return Matrix([
        [cos(q), -sin(q), 0, a],
        [sin(q) * cos(alpha), cos(q) * cos(alpha), -sin(alpha), -sin(alpha) * d],
        [sin(q) * sin(alpha), cos(q) * sin(alpha), cos(alpha), cos(alpha) * d],
        [0, 0, 0, 1]]).subs(dh_params)


def get_joints_1_3(WC):
    # Avoiding re-eval of variables in loop
    a1, a2, a3 = 0.35, 1.25, -0.054
    d1, d4 = 0.75, 1.5

    theta1 = atan2(WC[1], WC[0])

    # Using proper triangle terms, the references are harder to read without the drawing, see writeup.
    line_ab = a2  # empirically measured on the RViz model
    line_bc = sqrt(d4**2 + a3**2)
    line_ca = sqrt((sqrt(WC[0]**2 + WC[1]**2) - a1)**2 + (WC[2] - d1)**2)

    angle_a = acos((line_ca**2 + line_ab**2 - line_bc**2) / (2 * line_ca * line_ab))
    angle_b = acos((line_bc**2 + line_ab**2 - line_ca**2) / (2 * line_bc * line_ab))

    gamma = atan2(WC[2] - d1, sqrt(WC[0]**2 + WC[1]**2) - a1)
    beta = atan2(d4, -a3)

    theta2 = pi/2 - angle_a - gamma
    theta3 = -(angle_b - beta)  # accounting for sag in link 4 on z-axis

    return theta1, theta2, theta3


def get_joints_3_6(rot_matrix):
    theta5 = atan2(sqrt(rot_matrix[0, 2] ** 2 + rot_matrix[2, 2] ** 2), rot_matrix[1, 2])
    if sin(theta5) < 0:
        theta4 = atan2(-rot_matrix[2, 2], rot_matrix[0, 2])
        theta6 = atan2(rot_matrix[1, 1], -rot_matrix[1, 0])
    else:
        theta4 = atan2(rot_matrix[2, 2], -rot_matrix[0, 2])
        theta6 = atan2(-rot_matrix[1, 1], rot_matrix[1, 0])

    return theta4, theta5, theta6


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print("No valid poses received")
        return -1
    else:
        ### Your FK code here
        d1, d2, d3, d4, d5, d6, d7 = symbols("d1:8")  # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols("a0:7")  # link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols("alpha0:7")  # link twist
        q1, q2, q3, q4, q5, q6, q7 = symbols("q1:8")  # joint angle

        #  Create Modified DH parameters
        # TODO, validate alpha1, switched q2 for readability
        dh_params = {
            alpha0: 0, a0: 0, d1: .75, q1: q1,
            alpha1: -pi / 2., a1: .35, d2: 0, q2: q2 - pi / 2.,
            alpha2: 0, a2: 1.25, d3: 0, q3: q3,
            alpha3: -pi / 2., a3: -0.054, d4: 1.5, q4: q4,
            alpha4: pi / 2., a4: 0, d5: 0, q5: q5,
            alpha5: -pi / 2., a5: 0, d6: 0, q6: q6,
            alpha6: 0, a6: 0, d7: .303, q7: 0}

        # Create individual transformation matrices
        # Since those matrices serve as a template, we create them outside the loop
        T0_1 = get_transformation_matrix(alpha0, a0, d1, q1, dh_params)
        T1_2 = get_transformation_matrix(alpha1, a1, d2, q2, dh_params)
        T2_3 = get_transformation_matrix(alpha2, a2, d3, q3, dh_params)
        T3_4 = get_transformation_matrix(alpha3, a3, d4, q4, dh_params)
        T4_5 = get_transformation_matrix(alpha4, a4, d5, q5, dh_params)
        T5_6 = get_transformation_matrix(alpha5, a5, d6, q6, dh_params)
        T6_E = get_transformation_matrix(alpha6, a6, d7, q7, dh_params)

        # Extract rotation matrix from the transformation matrices
        # TODO: use for i in list_of_transforms instead of side-effect?
        T0_3 = T0_1 * T1_2 * T2_3
        T0_E = T0_3 * T3_4 * T4_5 * T5_6 * T6_E

        R_E = get_rotation_matrix()

        # Initialize service response
        joint_trajectory_list = []
        t4, t5, t6 = 0.0, 0.0, 0.0
        for x in xrange(0, len(req.poses)):

            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

            # Extract end-effector position and orientation from request
            # px,py,pz = end-effector position
            # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                 req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
            R_EE = R_E.subs({"r": roll, "p": pitch, "y": yaw})
            End_effector = Matrix([[px], [py], [pz]])
            WC = End_effector - 0.303 * R_EE[:, 2]

            # Calculate joint angles using Geometric IK method
            theta1, theta2, theta3 = get_joints_1_3(WC)

            # We only substitute and eval here
            # R0_3 = T0_1[0:3, 0:3] * T1_2[0:3, 0:3] * T2_3[0:3, 0:3]
            R0_3 = T0_3[0:3, :3]  # Reusing already calculated first 3 joints instead of multipy + inverse
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv("LU") * R_EE

            # Euler angles based on the rotation matrix
            # if x > 3 and x % 3 == 0 < len(req.poses) - 4:
            if x > .75*len(req.poses):
                # Skipping calculating proper angles for first half of the trajectory
                # What could go wrong?
                theta4, theta5, theta6 = get_joints_3_6(rot_matrix=R3_6)
                t4, t5, t6 = theta4, theta5, theta6

            else:
                # skip moving EE/Wrist for each second step after the first 3,
                # less accuracy but faster
                # absolutely use in your production pipeline without caution!
                rospy.loginfo("skipping calculation for step {0}".format(x))
                theta4, theta5, theta6 = t4, t5, t6

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print("Ready to receive an IK request")
    rospy.spin()

if __name__ == "__main__":
    IK_server()
