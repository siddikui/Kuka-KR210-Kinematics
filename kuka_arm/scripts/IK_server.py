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
import os
import pickle


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        # Define DH Parameters
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        r, p , y = symbols('r p y')

        # Define DH Transformation Matrix
	DH_Table = {alpha0:      0,      a0: 0, 	d1: 0.75, 	q1: q1,
		    alpha1: -pi/2.,      a1: 0.35,	d2:    0, 	q2: -pi/2. + q2,
		    alpha2:      0,      a2: 1.25, 	d3:    0, 	q3: q3,
		    alpha3: -pi/2.,      a3: -0.054, 	d4:  1.5, 	q4: q4,
		    alpha4:   pi/2, 	 a4: 0, 	d5: 0, 		q5: q5,
		    alpha5: -pi/2.,      a5: 0, 	d6: 0, 		q6: q6,
		    alpha6:      0, 	 a6: 0, 	d7: 0.303, 	q7: 0}

    	def TF_Matrix(alpha, a, d, q):
		TF = Matrix([[           cos(q),	    -sin(q),             0,               a],
	     		     [sin(q)*cos(alpha),  cos(q)*cos(alpha),   -sin(alpha),   -sin(alpha)*d],
	     		     [sin(q)*sin(alpha),  cos(q)*sin(alpha), 	cos(alpha),    cos(alpha)*d],
	     		     [                0,                  0,	         0,	          1]])
  		return TF

        T0_1 = TF_Matrix(alpha0, a0, d1, q1).subs(DH_Table)
    	T1_2 = TF_Matrix(alpha1, a1, d2, q2).subs(DH_Table)
    	T2_3 = TF_Matrix(alpha2, a2, d3, q3).subs(DH_Table)
       
        if not os.path.exists("R_EE.p"):

            R_x = Matrix([[1,       0,       0],
		          [0,  cos(r), -sin(r)],
   		          [0,  sin(r), cos(r)]]) 

            R_y = Matrix([[cos(p), 	0,  sin(p)],
		          [0, 	1,       0],
   		          [-sin(p), 0,  cos(p)]]) 

            R_z = Matrix([[cos(y), -sin(y),  0],
		          [sin(y),  cos(y),  0],
		          [     0,       0,  1]]) 

            R_EE = simplify(R_z * R_y * R_x)

            R_Error = R_z.subs(y, radians(180)) * R_y.subs(p, radians(-90))

            R_EE = simplify(R_EE * R_Error)

            pickle.dump(R_EE, open("R_EE.p","wb"))

        else:
            R_EE = pickle.load(open("R_EE.p","rb")) 

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

            R_EE = R_EE.subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px], [py], [pz]])

            WC = EE - (0.303) * R_EE[:,2]
            theta1 = atan2(WC[1], WC[0])
            # side A connects joint3 and WC.
            # side B connects joint2 and WC.
            # side C connects joint2 and joint3. 
            A = 1.501

            Perp = WC[2] - 0.75 # 0.75 is the distance from base to joint2 origin.

            Base = sqrt(WC[0] * WC[0] + WC[1] * WC[1]) - 0.35 # 0.35 is the joint2 offset from base.

            phi = atan2(Perp, Base)
        

            B = sqrt(Perp * Perp + Base * Base) # B = Hypotenuse to Perp and Base.            

            C = 1.25

            a = acos((B * B + C * C - A * A) / (2 * B * C))
            b = acos((A * A + C * C - B * B) / (2 * A * C))
            c = acos((A * A + B * B - C * C) / (2 * A * B))

            theta2 = pi/2 - a - phi 

            gamma = atan2(0.054, 1.501) # Sag angle of joint3 to WC.

            theta3 = pi/2 - (b + gamma) 

            R0_3 = T0_1[0:3,0:3] * T1_2[0:3,0:3] * T2_3[0:3,0:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2:theta2, q3: theta3})
            
            R3_6 = R0_3.transpose() * R_EE

            
            theta5 = atan2(sqrt(R3_6[0,2]*R3_6[0,2] + R3_6[2,2]*R3_6[2,2]), R3_6[1,2]).evalf()

            if(sin(theta5)>0):
                theta4 = atan2(R3_6[2,2], -R3_6[0,2]).evalf()
                theta6 = atan2(-R3_6[1,1], R3_6[1,0]).evalf()
            elif(theta5==0):
                theta4 = 0
                theta6 = atan2(-R3_6[0,1], -R3_6[2,1]).evalf()
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2]).evalf()
                theta6 = atan2(R3_6[1,1], -R3_6[1,0]).evalf()       

            while(theta4 > pi):
                theta4 = theta4 - 2*pi
            while(theta4 < -pi):
                theta4 = theta4 + 2*pi 

            while(theta5 > pi):
                theta5 = theta5 - 2*pi
            while(theta5 < -pi):
                theta5 = theta5 + 2*pi 

            while(theta6 > pi):
                theta6 = theta6 - 2*pi
            while(theta6 < -pi):
                theta6 = theta6 + 2*pi 


	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
	IK_server()
