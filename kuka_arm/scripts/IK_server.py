#!/usr/bin/env python

# Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program

# Module imports
import numpy as np
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *
from sympy.matrices import Matrix
import datetime


# Define some utility functions

def rot_x(q):
    '''Creates rotation matrix about X-axis'''
    R_x = Matrix([[ 1,              0,        0],
                  [ 0,        cos(q),   -sin(q)],
                  [ 0,        sin(q),    cos(q)]])
    return R_x
    
def rot_y(q):              
    '''Creates rotation matrix about Y-axis'''
    R_y = Matrix([[ cos(q),        0,   sin(q)],
                  [      0,        1,        0],
                  [-sin(q),        0,   cos(q)]])
    return R_y

def rot_z(q):    
    '''Creates rotation matrix about Z-axis'''
    R_z = Matrix([[ cos(q), -sin(q),        0],
                  [ sin(q),  cos(q),        0],
                  [      0,       0,        1]])
    return R_z

def homogeneous_rot_x(q):
    '''Creates homogeneous rotation matrix about X-axis'''
    R_x = rot_x(q).row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
    return R_x
    
def homogeneous_rot_y(q):              
    '''Creates homogeneous rotation matrix about Y-axis'''
    R_y = rot_y(q).row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
    return R_y

def homogeneous_rot_z(q):    
    '''Creates homogeneous rotation matrix about Z-axis'''
    R_z = rot_z(q).row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))
    return R_z

def create_DH_trf_matrix(alpha, a, d , q):
    '''Define Modified DH Transformation matrix'''
    T = Matrix([[           cos(q),           -sin(q),            0,                a],
               [ sin(q)*cos(alpha), cos(q)*cos(alpha),  -sin(alpha),    -sin(alpha)*d],
               [ sin(q)*sin(alpha), cos(q)*sin(alpha),   cos(alpha),     cos(alpha)*d],
               [                 0,                 0,            0,                1]])
    return T

def get_rot_from_homogeneous(M):
    '''Extracts the rotation matrix from the homogeneous transformation matrix'''
    return M[:3,:3]

def handle_calculate_IK(req):
    '''This callback service method performs kinematic analysis and returns joint angles for the given list of end-effector pose and orientation'''

    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1

    ### Forward Kinematics Code
    # Create symbols for the joint variables
    q1, q2, q3, q4, q5, q6 = symbols('q1:7')
    d1, d2, d3, d4, d5, d6 = symbols('d1:7')
    a0, a1, a2, a3, a4, a5 = symbols('a0:6')
    alpha0, alpha1, alpha2, alpha3, alpha4, alpha5 = symbols('alpha0:6')
    qG, dG, aG, alphaG = symbols('qG dG aG alphaG')

    # Create Modified DH parameters
    dh_params = {
        alpha0:0,     a0:0,      d1:0.75,
        alpha1:-pi/2, a1:0.35,   d2:0,      q2:q2-pi/2,
        alpha2:0,     a2:1.25,   d3:0,
        alpha3:-pi/2, a3:-0.054, d4:1.5,
        alpha4:pi/2,  a4:0,      d5:0,
        alpha5:-pi/2, a5:0,      d6:0,
        alphaG:0,     aG:0,      dG:0.303,  qG:0}


	# Define Modified DH Transformation matrix

	# Create individual transformation matrices
    T0_1 = create_DH_trf_matrix(alpha0, a0, d1, q1)
    T0_1 = T0_1.subs(dh_params)
    T1_2 = create_DH_trf_matrix(alpha1, a1, d2, q2)
    T1_2 = T1_2.subs(dh_params)
    T2_3 = create_DH_trf_matrix(alpha2, a2, d3, q3)
    T2_3 = T2_3.subs(dh_params)
    T3_4 = create_DH_trf_matrix(alpha3, a3, d4, q4)
    T3_4 = T3_4.subs(dh_params)
    T4_5 = create_DH_trf_matrix(alpha4, a4, d5, q5)
    T4_5 = T4_5.subs(dh_params)
    T5_6 = create_DH_trf_matrix(alpha5, a5, d6, q6)
    T5_6 = T5_6.subs(dh_params)
    T6_G = create_DH_trf_matrix(alphaG, aG, dG, qG)
    T6_G = T6_G.subs(dh_params)

	# Extract rotation matrices from the transformation matrices
    R0_1 = get_rot_from_homogeneous(T0_1)
    R1_2 = get_rot_from_homogeneous(T1_2)
    R2_3 = get_rot_from_homogeneous(T2_3)
    R3_4 = get_rot_from_homogeneous(T3_4)
    R4_5 = get_rot_from_homogeneous(T4_5)
    R5_6 = get_rot_from_homogeneous(T5_6)
    R6_G = get_rot_from_homogeneous(T6_G)

    # Composition of Transformation matrices - Incrementally from base to end effector frame
    T0_2 = T0_1 * T1_2
    T0_3 = T0_2 * T2_3
    T0_4 = T0_3 * T3_4
    T0_5 = T0_4 * T4_5
    T0_6 = T0_5 * T5_6
    T0_G = T0_6 * T6_G

    # Correction matrix for the end effector frame
    # This compensates for rotation discrepancy between DH parameters and Gazebo
    R_corr = rot_z(np.pi) * rot_y(-np.pi/2)
    
    # Total homogeneous transform between base link and end effector with correction
    T_total = T0_G * R_corr.row_join(Matrix([[0], [0], [0]])).col_join(Matrix([[0, 0, 0, 1]]))

    # Initialize service response
    joint_trajectory_list = []
    error_data = []
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

        ### Inverse Kinematics Code
        
        # Compensate for rotation discrepancy between DH parameters and Gazebo
        # Rrpy is in the DH frame (not URDF frame). R_corr is same as R_corr_inverse since R_corr is symmetrical, so we use R_corr directly.
        Rrpy = rot_z(yaw) * rot_y(pitch) * rot_x(roll) * R_corr

        # Calculate wrist center
        # n_unit is a vector in the direction of gripper axis
        n_unit = Rrpy[:,2]
        wc = Matrix([[px], [py], [pz]]) - dh_params[dG] * n_unit
        wc_x = wc[0]
        wc_y = wc[1]
        wc_z = wc[2]

        # Calculate joint angles using Geometric IK method
        
        # Theta1
        theta1 = mpmath.atan2(wc_y, wc_x)

        # Theta2
        r = mpmath.sqrt(wc_x**2 + wc_y**2)
        AB_sq = dh_params[a2]**2
        AB = dh_params[a2]
        AC_sq = (r-dh_params[a1])**2 + (wc_z-dh_params[d1])**2
        AC = mpmath.sqrt(AC_sq)
        BC_sq = dh_params[d4]**2 + dh_params[a3]**2
        BC = mpmath.sqrt(BC_sq)
        theta2_dash = mpmath.acos((AB_sq + AC_sq - BC_sq)/(2*AB*AC))
        theta2_dash2 = mpmath.atan2((wc_z - dh_params[d1]), (r - dh_params[a1]))
        theta2 = mpmath.pi/2 - theta2_dash - theta2_dash2

        # Theta3
        theta3_dash = mpmath.atan2(abs(dh_params[a3]), dh_params[d4])
        theta3_dash2 = mpmath.acos((AB_sq + BC_sq - AC_sq)/(2*AB*BC))
        theta3 = mpmath.pi/2 - theta3_dash - theta3_dash2

        # Theta4,5,6
        R0_3 = R0_1 * R1_2 * R2_3
        R3_6 = R0_3.T.evalf(subs={q1:theta1, q2:theta2, q3:theta3}) * Rrpy

        # The following commented code can be used to get the matrix in terms of theta4,5,6 to derive the atan2 experssions
        # R3_6_thetas = R3_4 * R4_5 * R5_6
        # print('\n\nR3_6 in terms of theta is:', R3_6_thetas)

        theta4 = mpmath.atan2(R3_6[2,2] , -1*R3_6[0,2])
        theta5 = mpmath.atan2(mpmath.sqrt(R3_6[1,0]**2 + R3_6[1,1]**2) , R3_6[1,2])
        theta6 = mpmath.atan2(-1 * R3_6[1,1] , R3_6[1,0])

        # Dump FK Error Data to file
        FK_end_eff_pos = T0_G.evalf(subs={q1:theta1, q2:theta2, q3:theta3, q4:theta4, q5:theta5, q6:theta6})[:,3]
        FK_error = mpmath.sqrt(abs(px-FK_end_eff_pos[0])**2 + abs(py-FK_end_eff_pos[1])**2 + abs(pz-FK_end_eff_pos[2])**2)
        error_data.append(FK_error)

        # Populate response for the IK request
        joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
        joint_trajectory_list.append(joint_trajectory_point)

    # Append error data points to file (a separate script will read from it and plot it)
    with open('error_points.txt', 'a+') as f:
        for error in error_data:
            f.write(str(error) + ' ')

    rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    '''initializes node and declares calculate_ik service'''
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
