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
from time import time

def TF_Matrix(alpha, a, d, q):
    TF = Matrix([[      cos(q),     -sin(q),        0,      a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
               [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d],
               [               0,                  0,          0,          1]])
    return TF


def handle_calculate_IK(req):
    start_time = time()
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
            
            ### Your FK code here
            # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') #theta_i
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        DH_Table = {alpha0:    0,      a0:     0,      d1: 0.75,   q1:         q1,
             alpha1: -pi/2,     a1:  0.35,      d2:    0,   q2: -pi/2. + q2,
             alpha2:     0,     a2:  1.25,      d3:    0,   q3:         q3}#,
             #For the purposes of the IK calcs, the following values are not required - however they are
             #included here for completeness. They would be needed if calculating the error curves however I 
             #have not included the challenge part of the project
             # alpha3: -pi/2,     a3:-0.054,      d4:  1.50,  q4:         q4,
             # alpha4:  pi/2,     a4:     0,      d5:     0,  q5:         q5,
             # alpha5: -pi/2,     a5:     0,      d6:     0,  q6:         q6,
             # alpha6:     0,     a6:     0,      d7: 0.303,  q7: 0}

        # Define Modified DH Transformation matrix
        def TF_Matrix(alpha, a, d, q):
            TF = Matrix([[      cos(q),     -sin(q),        0,      a],
                        [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                       [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha), cos(alpha)*d],
                       [               0,                  0,          0,          1]])
            return TF

        #Creating transformation matrices via above function and substituting values from the DH Table.
        #This is faster computationally than using the below full symbolic matrices.
        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
        T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
        T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
        #Again for the purposes of the IK calcs, the following values are not required - however they are
        #included here for completeness. They would be needed if calculating the error curves, however I 
        #have not included the challenge part of the project
        #T3_4 = TF_Matrix(alpha3, a3, d4, q4).subs(DH_Table)
        #T4_5 = TF_Matrix(alpha4, a4, d5, q5).subs(DH_Table)
        #T5_6 = TF_Matrix(alpha5, a5, d6, q6).subs(DH_Table)
        #T6_EE = TF_Matrix(alpha6, a6, d7, q7).subs(DH_Table)

        #The following are the full matrix representations of the above transformations, put here
        #for completeness.
        # T0_1 = Matrix([[            cos(q1),            -sin(q1),           0,                   a0],
        #                [sin(q1)*cos(alpha0),    cos(q1)*cos(alpha0), -sin(alpha0),  -sin(alpha0)*d1],
        #                [sin(q1)*sin(alpha0),    cos(q1)*sin(alpha0),  cos(alpha0),   cos(alpha0)*d1],
        #                [                  0,                    0,              0,              1  ]])

        # T0_1 = T0_1.subs(s)




        # T1_2 = Matrix([[            cos(q2),            -sin(q2),           0,                   a1],
        #                 [sin(q2)*cos(alpha1),   cos(q2)*cos(alpha1), -sin(alpha1),  -sin(alpha1)*d2],
        #                 [sin(q2)*sin(alpha1),   cos(q2)*sin(alpha1),  cos(alpha1),   cos(alpha1)*d2],
        #                 [                 0,                    0,              0,              1  ]])

        # T1_2 = T1_2.subs(s)




        # T2_3 = Matrix([[                cos(q3),            -sin(q3),           0,               a2],
        #                 [sin(q3)*cos(alpha2),   cos(q3)*cos(alpha2), -sin(alpha2),  -sin(alpha2)*d3],
        #                 [sin(q3)*sin(alpha2),   cos(q3)*sin(alpha2),  cos(alpha2),   cos(alpha2)*d3],
        #                 [                 0,                    0,              0,              1  ]])

        # T2_3 = T2_3.subs(s)


        # T3_4 = Matrix([[            cos(q4),            -sin(q4),           0,                   a3],
        #                [sin(q4)*cos(alpha3),    cos(q4)*cos(alpha3), -sin(alpha3),  -sin(alpha3)*d4],
        #                [sin(q4)*sin(alpha3),    cos(q4)*sin(alpha3),  cos(alpha3),   cos(alpha3)*d4],
        #                [                  0,                    0,              0,              1  ]])

        # T3_4 = T3_4.subs(s)




        # T4_5 = Matrix([[            cos(q5),            -sin(q5),           0,                   a4],
        #                 [sin(q5)*cos(alpha4),   cos(q5)*cos(alpha4), -sin(alpha4),  -sin(alpha4)*d5],
        #                 [sin(q5)*sin(alpha4),   cos(q5)*sin(alpha4),  cos(alpha4),   cos(alpha4)*d5],
        #                 [                 0,                    0,              0,              1  ]])

        # T4_5 = T4_5.subs(s)

        


        # T5_6 = Matrix([[            cos(q6),            -sin(q6),           0,                   a5],
        #                 [sin(q6)*cos(alpha5),   cos(q6)*cos(alpha5), -sin(alpha5),  -sin(alpha5)*d6],
        #                 [sin(q6)*sin(alpha5),   cos(q6)*sin(alpha5),  cos(alpha5),   cos(alpha5)*d6],
        #                 [                 0,                    0,              0,              1  ]])

        # T5_6 = T5_6.subs(s)


        # T6_EE = Matrix([[            cos(q7),            -sin(q7),           0,                   a6],
        #                 [sin(q7)*cos(alpha6),   cos(q7)*cos(alpha6), -sin(alpha6),  -sin(alpha6)*d7],
        #                 [sin(q7)*sin(alpha6),   cos(q7)*sin(alpha6),  cos(alpha6),   cos(alpha6)*d7],
        #                 [                 0,                    0,              0,              1  ]])

        # T6_EE = T6_EE.subs(s)

        #Total transformation to end effector from the base. Again this is not required for the IK calcs
        #T0_EE = T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE

        # Extract rotation matrices from the transformation matrices
        # These are the 'sliced' rotation matrices from the homogeneous transforms above. Active code is below
        #at line 227, included here to fit in with the template structure.
        #T0_1R = T0_1[0:3, 0:3]
        #T1_2R = T1_2[0:3, 0:3]
        #T2_3R = T2_3[0:3, 0:3]
        #T3_4R = T3_4[0:3, 0:3]
        #T4_5R = T4_5[0:3, 0:3]
        #T5_6R = T5_6[0:3, 0:3]
        #T6_EER = T6_EE[0:3, 0:3]
        
            ###

        # Initialize service response
        joint_trajectory_list = []
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
                #calculate euler angles
            #this code allows us to use the current end effector values to carry out the inverse position calculations
            #They represent fundamental rotations about x y and z.
            r, p, y = symbols('r p y')

            ROT_x = Matrix([[1,         0,          0],
                            [0,         cos(r),     -sin(r)],
                            [0,         sin(r),     cos(r)]]) #ROLL
            
            ROT_y = Matrix([[cos(p),    0,          sin(p)], #PITCH
                            [0,         1,          0],
                            [-sin(p),   0,          cos(p)]])

            ROT_z = Matrix([[cos(y),    -sin(y),    0],
                            [sin(y),    cos(y),     0],
                            [0,         0,          1]])

            #calculate End Effector rotation matrix
            ROT_EE = ROT_z * ROT_y * ROT_x


            # Compensate for rotation discrepancy between DH parameters and Gazebo
            #apply the corrective matrix to account for difference between DH and urdf file parameters.
            R_corr = ROT_z.subs(y, radians(180)) * ROT_y.subs(p, radians(-90))
            ROT_EE = ROT_EE * R_corr #this is the total TF from frame 0 to G times the correction above
            #substitute in values of roll pitch and yaw
            ROT_EE = ROT_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            #create a matrix of the retrieved end effector position
            EE = Matrix([[px],
                         [py],
                         [pz]])

            #define the wrist centre in relation to the end effector
            WC = EE - (0.303) * ROT_EE[:,2]


                # Calculate joint angles using Geometric IK method
            #See writeup notes for geometric analysis

            theta1 = atan2(WC[1],WC[0])
            side_a = 1.501
            side_b = sqrt(pow((sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35), 2) + pow((WC[2] - 0.75), 2))
            side_c = 1.25

            angle_a = acos((side_b * side_b + side_c * side_c - side_a * side_a) / (2 * side_b * side_c))
            angle_b = acos((side_a * side_a + side_c * side_c - side_b * side_b) / (2 * side_a * side_c))
            angle_c = acos((side_a * side_a + side_b * side_b - side_c * side_c) / (2 * side_a * side_b))
            theta2 = pi / 2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35)
            theta3 = pi / 2 - (angle_b + 0.036) #0.036 accounts for the drop in link 4 of -0.054m

            #Extract rotation matrices from frame 0 to frame 3
            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3] #these are the "sliced" rotation matrices (duplicate)
            #substitute vlues for theta
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})

            #Given that our end effector position is known and represented by ROT_EE: R0_3 * R3_6 = ROT_EE, so we can find
            #the inverse position by the following:  
            R3_6 = R0_3.T * ROT_EE
            #Note in above, the transpose is used instead, as the 'LU' notation is inaccurate and caused the pick and place to fail, 
            #The transpose is equal to the inverse for orthogonal matrices. 'ADJ' also gave accurate results but is computationally less efficient than Transpose

            # Euler angles from rotation matrix. See writeup for geometric analysis
            # We find expressions for the remaining joint angles to allow us to find inverse orientation:
            theta4 = atan2(R3_6[2,2], -R3_6[0,2])
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2])
            theta6 = atan2(-R3_6[1,1], R3_6[1,0])
                
            # Populate response for the IK request
            # Populates the list with calculated joint angles
            joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
            joint_trajectory_list.append(joint_trajectory_point)

            rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
            rospy.loginfo("Time taken for calcs: %s" % (time()-start_time))
    return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
