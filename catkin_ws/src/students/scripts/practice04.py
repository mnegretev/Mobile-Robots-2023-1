#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-1
# PRACTICE 4 - PATH SMOOTHING BY GRADIENT DESCEND
#
# Instructions:
# Write the code necessary to smooth a path using the gradient descend algorithm.
# MODIFY ONLY THE SECTIONS MARKED WITH THE 'TODO' COMMENT
#

import numpy
import heapq
import rospy
from nav_msgs.msg import Path
from geometry_msgs.msg import Pose, PoseStamped, Point
from custom_msgs.srv import SmoothPath
from custom_msgs.srv import SmoothPathResponse

NAME = "CHAVOLLA JIMENEZ RAUL DANIEL "

msg_smooth_path = Path()

def smooth_path(Q, alpha, beta):
    print("Smoothing path with params: " + str([alpha,beta]))
    #
    # TODO:
    # Write the code to smooth the path Q, using the gradient descend algorithm,
    # and return a new smoothed path P.
    # Path is composed of a set of points [x,y] as follows:
    # [[x0,y0], [x1,y1], ..., [xn,ym]].
    # The smoothed path must have the same shape.
    # Return the smoothed path.
    #
    P = numpy.copy(Q)
    tol     = 0.00001                   
    nabla   = numpy.full(Q.shape, float("inf"))
    epsilon = 0.1                       
    steps   = 0
    nabla[0], nabla[-1]=0, 0
    while numpy.linalg.norm(nabla)> tol*len(P) and steps < 1000000:
        for i in range(1, len(Q)-1):
            nabla[i]= alpha*(2*P[i]-P[i-1]-P[i+1])+beta*(P[i]-Q[i])
        P = P - epsilon*nabla
        steps +=1
        
    print("Path smoothed succesfully after " + str(steps) + " iterations")
    return P

def callback_smooth_path(req):
    alpha = rospy.get_param('/path_planning/smoothing_alpha')
    beta  = rospy.get_param('/path_planning/smoothing_beta' )
    P = smooth_path(numpy.asarray([[p.pose.position.x, p.pose.position.y] for p in req.path.poses]), alpha, beta)
    msg_smooth_path.poses = []
    for i in range(len(req.path.poses)):
        msg_smooth_path.poses.append(PoseStamped(pose=Pose(position=Point(x=P[i,0],y=P[i,1]))))
    return SmoothPathResponse(smooth_path=msg_smooth_path)

def main():
    print("PRACTICE 04 - " + NAME)
    rospy.init_node("practice04", anonymous=True)
    rospy.Service('/path_planning/smooth_path', SmoothPath, callback_smooth_path)
    pub_path = rospy.Publisher('/path_planning/smooth_path', Path, queue_size=10)
    loop = rospy.Rate(1)
    msg_smooth_path.header.frame_id = "map"
    while not rospy.is_shutdown():
        pub_path.publish(msg_smooth_path)
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
