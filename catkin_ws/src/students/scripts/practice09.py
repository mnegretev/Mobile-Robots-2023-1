#!/usr/bin/env python3
#
# MOBILE ROBOTS - UNAM, FI, 2023-1
# PRACTICE 09 - INVERSE KINEMATICS
#
# Instructions:
# Calculate the inverse kinematics for both manipulators (left and right) given the
# URDF descriptions and a desired configuration. Solve the inverse kinematics using
# the Newton-Raphson method for root finding.
# Modify only sections marked with the 'TODO' comment
#

import math
import rospy
import tf
import tf.transformations as tft
import numpy
import urdf_parser_py.urdf
from geometry_msgs.msg import PointStamped
from custom_msgs.srv import *

NAME = "SANCHEZ TORRES SERGIO DANIEL"

def get_model_info():
    global joints, transforms
    robot_model = urdf_parser_py.urdf.URDF.from_parameter_server()
    joints = {'left': [None for i in range(8)], 'right': [None for i in range(8)]}
    transforms = {'left':[], 'right':[]}
    for joint in robot_model.joints:
        for i in range(1,8):
            joints['left' ][i-1] = joint if joint.name == ('la_'+str(i)+'_joint') else joints['left' ][i-1]
            joints['right'][i-1] = joint if joint.name == ('ra_'+str(i)+'_joint') else joints['right'][i-1]
        joints['left' ][7] = joint if joint.name == 'la_grip_center_joint' else joints['left' ][7]
        joints['right'][7] = joint if joint.name == 'ra_grip_center_joint' else joints['right'][7]
    for joint in joints['left']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['left'].append(tft.concatenate_matrices(T,R))
    for joint in joints['right']:
        T = tft.translation_matrix(joint.origin.xyz)
        R = tft.euler_matrix(joint.origin.rpy[0], joint.origin.rpy[1], joint.origin.rpy[2])
        transforms['right'].append(tft.concatenate_matrices(T,R))
    
def forward_kinematics(q, Ti, Wi):
    #
    # TODO:
    # Calculate the forward kinematics given the set of seven angles 'q'
    # You can use the following steps:
    #     H = I   # Assing to H a 4x4 identity matrix
    #     for all qi in q:
    #         H = H * Ti * Ri
    #     H = H * Ti[7]
    #     Get xyzRPY from the resulting Homogeneous Transformation 'H'
    # Where:
    #     Ti are the Homogeneous Transformations from frame i to frame i-1 when joint i is at zero position
    #     Ri are the Homogeneous Transformations with zero-translation and rotation qi around axis Wi.
    #     Ti[7] is the final Homogeneous Transformation from gripper center to joint 7.

    H = tft.identify_matrix()

    for i in range(len(q)):
        H = tft.concatenate_matrices(H, Ti[i], tft.rotation_matrix(q[i], Wi[i]))

    H = tft.concatenate_matrices(H, Ti[7])

    # Hints:
    #     Use the tft.identity_matrix() function to get the 4x4 I
    #     Use the tft.concatenate_matrices() function for multiplying Homogeneous Transformations
    #     Use the tft.rotation_matrix() matrices Ri.
    #     Use the tft.euler_from_matrix() function to get RPY from matrix H
    #     Check online documentation of these functions:
    #     http://docs.ros.org/en/jade/api/tf/html/python/transformations.html
    #
    x,y,z,R,P,Y = H[0][3], H[1][3], H[2][3], tft.euler_from_matrix(H, 'rxyz'), tft.euler_from_matrix(H, 'rxyz'), tft.euler_from_matrix(H, 'rxyz')
    return numpy.asarray([x,y,z,R,P,Y])

def jacobian(q, Ti, Wi):
    delta_q = 0.000001
    #
    # TODO:
    # Calculate the Jacobian given a kinematic description Ti and Wi
    # where:
    # Ti are the Homogeneous Transformations from frame i to frame i-1 when joint i is at zero position
    # Wi are the axis of rotation of i-th joint
    # Use the numeric approximation:   f'(x) = (f(x+delta) - f(x-delta))/(2*delta)
    #
    # You can do the following steps:
    #     J = matrix of 6x7 full of zeros
    #     q_next = [q1+delta       q2        q3   ....     q7
    #                  q1       q2+delta     q3   ....     q7
    #                              ....
    #                  q1          q2        q3   ....   q7+delta]
    #     q_prev = [q1-delta       q2        q3   ....     q7
    #                  q1       q2-delta     q3   ....     q7
    #                              ....
    #                  q1          q2        q3   ....   q7-delta]
    #     FOR i = 1,..,7:
    #           i-th column of J = ( FK(i-th row of q_next) - FK(i-th row of q_prev) ) / (2*delta_q)
    #     RETURN J
    #     
    J = numpy.asarray([[0.0 for a in q] for i in range(6)])            # J 6x7 full of zeros
    q_next = numpy.asarray(([q,] * len(q))) + (delta_q * numpy.identify(len(q)))
    q_prev = numpy.asarray(([q,] * len(q))) + (delta_q * numpy.identify(len(q)))

    for i in range(0,7):
        J[:,i] = (forward_kinematics(q_next[i,:], Ti, Wi) - forward_kinematics(q_prev[i,:], Ti, Wi)) / (2 * delta_q)
    
    return J

def inverse_kinematics_xyzrpy(x, y, z, roll, pitch, yaw, Ti, Wi):
    pd = numpy.asarray([x,y,z,roll,pitch,yaw])  # Desired configuration
    tolerance = 0.01
    max_iterations = 20
    iterations = 0
    #
    # TODO:
    # Solve the IK problem given a kinematic description (Ti, Wi) and a desired configuration.
    # where:
    # Ti are the Homogeneous Transformations from frame i to frame i-1 when joint i is at zero position
    # Wi are the axis of rotation of i-th joint
    # Use the Newton-Raphson method for root finding. (Find the roots of equation FK(q) - pd = 0)
    # You can do the following steps:
    #
    #    Set an initial guess for joints 'q'. Suggested: [-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3]
    q = numpy.asarray([-0.5, 0.6, 0.3, 2.0, 0.3, 0.2, 0.3])

    #    Calculate Forward Kinematics 'p' by calling the corresponding function
    #    Calcualte error = p - pd
    p = forward_kinematics(q, Ti, Wi)
    error = p - pd

    #    Ensure orientation angles of error are in [-pi,pi]
    #    WHILE |error| > TOL and iterations < maximum iterations:
    #        Calculate Jacobian
    #        Update q estimation with q = q - pseudo_inverse(J)*error
    #        Ensure all angles q are in [-pi,pi]
    #        Recalculate forward kinematics p
    #        Recalculate error and ensure angles are in [-pi,pi]
    #        Increment iterations
    #    Return calculated q if maximum iterations were not exceeded
    #    Otherwise, return None
    #
    while numpy.linalg.norm(error) > tolerance && iterations < max_iterations:
        J = jacobian(q, Ti, Wi)
        
        #Keeping the error between [-pi, pi]
        for i in range(len(error)):
            if error[i] >= math.pi:
                error[i] -= 2*math.pi
            elif error[i] <= (-math.pi):
                error[i] += 2*math.pi
        
        q = q - numpy.dot(numpy.linalg.pinv(J), error)
        
        #Keeping 'q' angles between [-pi, pi]
        for i in range(len(q)):
            if q[i] >= math.pi:  
                q[i] -= 2*math.pi   
            elif q[i] <= (-math.pi):
                q[i] += 2*math.pi
        
        #Recalculating forward kinematics p
        p = forward_kinematics(q, Ti, Wi)

        #Recalculating error
        error = p - pd
                
        iterations += 1

    if iterations < max_iteration:
        return q
    else:
        return None

def callback_la_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]  
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi)
    if q is None:
        return None
    resp = InverseKinematicsResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_ra_ik_for_pose(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    q = inverse_kinematics_xyzrpy(req.x, req.y, req.z, req.roll, req.pitch, req.yaw, Ti, Wi)
    if q is None:
        return False
    resp = InverseKinematicsResponse()
    [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7] = [q[0], q[1], q[2], q[3], q[4], q[5], q[6]]
    return resp

def callback_la_fk(req):
    global transforms, joints
    Ti = transforms['left']                               
    Wi = [joints['left'][i].axis for i in range(len(joints['left']))]  
    x = forward_kinematics([req.q[0], req.q[1], req.q[2], req.q[3], req.q[4], req.q[5], req.q[6]], Ti, Wi)
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def callback_ra_fk(req):
    global transforms, joints
    Ti = transforms['right']                               
    Wi = [joints['right'][i].axis for i in range(len(joints['right']))]  
    x = forward_kinematics([req.q[0], req.q[1], req.q[2], req.q[3], req.q[4], req.q[5], req.q[6]], Ti, Wi)
    resp = ForwardKinematicsResponse()
    [resp.x, resp.y, resp.z, resp.roll, resp.pitch, resp.yaw] = x
    return resp

def main():
    print("PRACTICE 09" + NAME)
    rospy.init_node("ik_geometric")
    get_model_info()
    rospy.Service("/manipulation/la_inverse_kinematics", InverseKinematics, callback_la_ik_for_pose)
    rospy.Service("/manipulation/ra_inverse_kinematics", InverseKinematics, callback_ra_ik_for_pose)
    rospy.Service("/manipulation/la_forward_kinematics", ForwardKinematics, callback_la_fk)
    rospy.Service("/manipulation/ra_forward_kinematics", ForwardKinematics, callback_ra_fk)
    loop = rospy.Rate(10)
    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    main()
