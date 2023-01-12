#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2023-1
# PRACTICE 5 - PATH FOLLOWING
#
# Instructions:
# Write the code necessary to move the robot along a given path.
# Consider a differential base. Max linear and angular speeds
# must be 0.8 and 1.0 respectively.
#

import rospy
import tf
import math
from std_msgs.msg import Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from custom_msgs.srv import SmoothPath, SmoothPathRequest
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point

NAME = "TAPIA_SOLIS"

pub_goal_reached = None
pub_cmd_vel = None
loop        = None
listener    = None

def calculate_control(robot_x, robot_y, robot_a, goal_x, goal_y):
    cmd_vel = Twist()
    
    #
    # TODO:
    # Implement the control law given by:
    #
    # v = v_max*math.exp(-error_a*error_a/alpha)
    # w = w_max*(2/(1 + math.exp(-error_a/beta)) - 1)
    #
    # where error_a is the angle error and
    # v and w are the linear and angular speeds taken as input signals
    # and v_max, w_max, alpha and beta, are tunning constants.
    # Store the resulting v and w in the Twist message cmd_vel
    # and return it (check online documentation for the Twist message).
    # Remember to keep error angle in the interval (-pi,pi]
    #
    #Constantes para el modelo del control
    alpha = 0.1
    beta = 0.1
    #Constantes de velocidad
    v_max = 0.4
    w_max = 0.5
    #Error para el angulo
    error_a = math.atan2((goal_y - robot_y),(goal_x - robot_x)) - robot_a
    if(error_a > math.pi):
	error_a = error_a-2*math.pi
    if(error_a < -math.pi):
	error_a = error_a+2*math.pi
    #Leyes de control
    v = v_max*math.exp(-error_a*error_a/alpha)
    w = w_max*(2/(1+math.exp(-error_a/beta))-1)

    cmd_vel.linear.x = v
    cmd_vel.angular.z = w
    return cmd_vel

def follow_path(path):
    #
    # TODO:
    # Use the calculate_control function to move the robot along the path.
    # Path is given as a sequence of points [[x0,y0], [x1,y1], ..., [xn,yn]]
    # The publisher for the twist message is already declared as 'pub_cmd_vel'
    # You can use the following steps to perform the path tracking:
    #
    # Set local goal point as the first point of the path
    # Set global goal point as the last point of the path
    # Get robot position with [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    # Calculate global error as the magnitude of the vector from robot pose to global goal point
    # Calculate local  error as the magnitude of the vector from robot pose to local  goal point
    #
    # WHILE global error > tol and not rospy.is_shutdown() #This keeps the program aware of signals such as Ctrl+C
    #     Calculate control signals v and w and publish the corresponding message
    #     loop.sleep()  #This is important to avoid an overconsumption of processing time
    #     Get robot position
    #     Calculate local error
    #     If local error is less than 0.3 (you can change this constant)
    #         Change local goal point to the next point in the path
    #     Calculate global error
    # Send zero speeds (otherwise, robot will keep moving after reaching last point)
    # Publish a 'True' using the pub_goal_reached publisher
    #
    idx = 0
    [local_x, local_y] = path[idx]  #Primer elemento
    [global_x, global_y] = path[-1]  #Ultimo elemento
    [robot_x, robot_y, robot_a] = get_robot_pose(listener) #Posicion del robot
    global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2) 
    local_error = math.sqrt((local_x - robot_x)**2 + (local_y - robot_y)**2)
    
    while global_error > 0.1 and not rospy.is_shutdown():      
	pub_cmd_vel.publish(calculate_control(robot_x, robot_y, robot_a, local_x, local_y)) #Calcula el control
        loop.sleep()
        [robot_x, robot_y, robot_a] = get_robot_pose(listener)
        local_error = math.sqrt((local_x - robot_x)**2 + (local_y - robot_y)**2)
        if local_error < 0.3:
            idx+=1
            if idx >= len(path):
                idx = len(path) - 1
            [local_x, local_y] = path[idx]
        global_error = math.sqrt((global_x - robot_x)**2 + (global_y - robot_y)**2)
    pub_cmd_vel.publish(Twist())
    pub_goal_reached.publish(True)
    
    return
    
def callback_global_goal(msg):
    print "Calculating path from robot pose to " + str([msg.pose.position.x, msg.pose.position.y])
    [robot_x, robot_y, robot_a] = get_robot_pose(listener)
    req = GetPlanRequest(goal=PoseStamped(pose=msg.pose))
    req.start.pose.position = Point(x=robot_x, y=robot_y)
    path = rospy.ServiceProxy('/path_planning/a_star_search', GetPlan)(req).plan
    path = rospy.ServiceProxy('/path_planning/smooth_path',SmoothPath)(SmoothPathRequest(path=path)).smooth_path
    print "Following path with " + str(len(path.poses)) + " points..."
    follow_path([[p.pose.position.x, p.pose.position.y] for p in path.poses])
    print("Global goal point reached")

def get_robot_pose(listener):
    try:
        ([x, y, z], rot) = listener.lookupTransform('map', 'base_link', rospy.Time(0))
        a = 2*math.atan2(rot[2], rot[3])
        a = a - 2*math.pi if a > math.pi else a
        return [x, y, a]
    except:
        pass
    return [0,0,0]

def main():
    global pub_cmd_vel, pub_goal_reached, loop, listener
    print "PRACTICE 04 - " + NAME
    rospy.init_node("practice04")
    rospy.Subscriber('/move_base_simple/goal', PoseStamped, callback_global_goal)
    pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pub_goal_reached = rospy.Publisher('/navigation/goal_reached', Bool, queue_size=10)
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for service for path planning...")
    rospy.wait_for_service('/path_planning/a_star_search')
    print("Service for path planning is now available.")
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
