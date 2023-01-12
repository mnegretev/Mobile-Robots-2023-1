#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# FINAL PROJECT - SIMPLE SERVICE ROBOT
# 
# Instructions:
# Write the code necessary to make the robot to perform the following possible commands:
# * Robot take the <pringles|drink> to the <table|kitchen>
# You can choose where the table and kitchen are located within the map.
# Pringles and drink are the two objects on the table used in practice 07.
# The Robot must recognize the orders using speech recognition.
# Entering the command by text or similar way is not allowed.
# The Robot must announce the progress of the action using speech synthesis,
# for example: I'm going to grab..., I'm going to navigate to ..., I arrived to..., etc.
# Publishers and suscribers to interact with the subsystems (navigation,
# vision, manipulation, speech synthesis and recognition) are already declared. 
#

import rospy
import tf
import math
import time
from std_msgs.msg import String, Float64MultiArray, Float64, Bool
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan, GetPlanRequest
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, PointStamped
from sound_play.msg import SoundRequest
from custom_msgs.srv import *
from custom_msgs.msg import *

NAME = "TAPIA SOLIS"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    recognized_speech = msg.hypothesis[0]
    print("New command received: " + recognized_speech)

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    return obj, loc

#
# This function sends the goal articular position to the left arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_left_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubLaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the left gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal articular position to the right arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_right_arm(q1,q2,q3,q4,q5,q6,q7):
    global pubRaGoalPose
    msg = Float64MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(2.0)

#
# This function sends the goal angular position to the right gripper and sleeps 1 second
# to allow the gripper to reach the goal angle. 
#
def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal pan-tilt angles to the head and sleeps 1 second
# to allow the head to reach the goal position. 
#
def move_head(pan, tilt):
    global pubHdGoalPose
    msg = Float64MultiArray()
    msg.data.append(pan)
    msg.data.append(tilt)
    pubHdGoalPose.publish(msg)
    time.sleep(1.0)

#
# This function sends a linear and angular speed to the mobile base to perform
# low-level movements. The mobile base will move at the given linear-angular speeds
# during a time given by 't'
#
def move_base(linear, angular, t):
    global pubCmdVel
    cmd = Twist()
    cmd.linear.x = linear
    cmd.angular.z = angular
    pubCmdVel.publish(cmd)
    time.sleep(t)
    pubCmdVel.publish(Twist())

#
# This function publishes a global goal position. This topic is subscribed by
# pratice04 and performs path planning and tracking.
#
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

#
# This function sends a text to be synthetized.
#
def say(text):
    global pubSay
    msg = SoundRequest()
    msg.sound   = -3
    msg.command = 1
    msg.volume  = 1.0
    msg.arg2    = "voice_kal_diphone"
    msg.arg = text
    pubSay.publish(msg)

#
# This function calls the service for calculating inverse kinematics for left arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    resp = clt(req_ik)
    return [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]

#
# This function calls the service for calculating inverse kinematics for right arm (practice 08)
# and returns the calculated articular position.
#
def calculate_inverse_kinematics_left(x,y,z,roll, pitch, yaw):
    req_ik = InverseKinematicsRequest()
    req_ik.x = x
    req_ik.y = y
    req_ik.z = z
    req_ik.roll  = roll
    req_ik.pitch = pitch
    req_ik.yaw   = yaw
    clt = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    resp = clt(req_ik)
    return [resp.q1, resp.q2, resp.q3, resp.q4, resp.q5, resp.q6, resp.q7]

#
# Calls the service for finding object (practice 08) and returns
# the xyz coordinates of the requested object w.r.t. "realsense_link"
#
def find_object(object_name):
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)
    req_find_object = FindObjectRequest()
    req_find_object.cloud = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
    req_find_object.name  = object_name
    resp = clt_find_object(req_find_object)
    return [resp.x, resp.y, resp.z]

#
# Transforms a point xyz expressed w.r.t. source frame to the target frame
#
def transform_point(x,y,z, source_frame, target_frame):
    listener = tf.TransformListener()
    listener.waitForTransform(target_frame, source_frame, rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = source_frame
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    obj_p = listener.transformPoint(target_frame, obj_p)
    return [obj_p.point.x, obj_p.point.y, obj_p.point.z]

def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_exercise")
    rospy.Subscriber('/hri/sp_rec/recognized', RecognizedSpeech, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float64MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float64MultiArray, queue_size=10);
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float64MultiArray, queue_size=10);
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float64, queue_size=10);
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float64, queue_size=10);
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/manipulation/ra_inverse_kinematics')
    rospy.wait_for_service('/vision/find_object')
    print("Services are now available.")

    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "SM_INIT"
    requested_object   = ""
    requested_location = [0,0]
    #aux = [0,0]
    	
    #
    # FINAL PROJECT
    # 
    
    while not rospy.is_shutdown():
    	if current_state == "SM_INIT":
            print("Waiting for new task")
            current_state = "SM_WAITING_NEW_TASK"
	elif current_state == "MOVER_BASE":
	    move_head(0, -0.9)
	    move_base(-0.3, 0, 2.0)
            current_state = "SM_START_NAVIGATION"
	    #move_base(0.3, 0.3, 0.0)
	    #current_state = "SM_INIT"

        elif current_state == "SM_WAITING_NEW_TASK":
            if new_task:
                requested_object, requested_location = parse_command(recognized_speech)
                print("New task received: " + requested_object + " to  " + str(requested_location))
                say("Executing the command, " + recognized_speech)
                rospy.sleep(5)
                current_state = "SM_MOVE_HEAD"
                new_task = False
                executing_task = True
                
        elif current_state == "SM_MOVE_HEAD":
            print("Moving head to look at table...")
            move_head(0, -0.9)
            current_state = "SM_FIND_OBJECT"
            
        elif current_state == "SM_FIND_OBJECT":
            print("Trying to find object: " + requested_object)
            req_find_object = FindObjectRequest()
            req_find_object.cloud = rospy.wait_for_message("/kinect/points", PointCloud2)
            req_find_object.name  = requested_object
            resp_find_object = clt_find_object(req_find_object)
            print("Object found at: " + str([resp_find_object.x, resp_find_object.y, resp_find_object.z]))
            current_state = "SM_INVERSE_KINEMATICS"
            
        elif current_state == "SM_INVERSE_KINEMATICS":
            obj_p = PointStamped()
            obj_p.header.frame_id = "kinect_link"
            obj_p.header.stamp = rospy.Time(0)
            obj_p.point.x, obj_p.point.y, obj_p.point.z,  = resp_find_object.x, resp_find_object.y, resp_find_object.z
            target_frame = "shoulders_left_link" if requested_object == "pringles" else "shoulders_right_link"
            print("Transforming " + requested_object + " position to " + target_frame)
            obj_p = listener.transformPoint(target_frame, obj_p)
            print("Trying to get inverse kinematics for pose " + str([obj_p.point.x,obj_p.point.y,obj_p.point.z]))
            req_ik = InverseKinematicsRequest()
            req_ik.x = obj_p.point.x + 0.05
            req_ik.y = obj_p.point.y
            req_ik.z = obj_p.point.z + 0.1
            req_ik.roll  = 3.0
            req_ik.pitch = -1.57
            req_ik.yaw   = -3.0
            say("I'm going to grab the " + requested_object)
            if requested_object == "pringles":
                resp_ik = clt_la_inverse_kin(req_ik)
            else:
                resp_ik = clt_ra_inverse_kin(req_ik)
            current_state = "SM_MOVE_LEFT_ARM" if requested_object == "pringles" else "SM_MOVE_RIGHT_ARM"
            
        elif current_state == "SM_MOVE_LEFT_ARM":
            print("Moving left manipulator to stand by position")
            move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            move_left_gripper(0.7)
            print("Moving left manipulator to object position")
            move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
            move_left_gripper(-0.3)
            move_left_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4+0.1, resp_ik.q5, resp_ik.q6+0.1, resp_ik.q7)
            print("Moving backwards")
            move_base(-0.3, 0, 2.0)
            move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            current_state = "SM_START_NAVIGATION"
            
        elif current_state == "SM_MOVE_RIGHT_ARM":
            print("Moving right manipulator to stand by position")
            move_right_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            move_right_gripper(0.2)
            print("Moving right manipulator to object position")
            move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
            move_right_gripper(-0.2)
            move_right_arm(resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4+0.1, resp_ik.q5, resp_ik.q6+0.1, resp_ik.q7)
            print("Moving backwards")
            move_base(-0.3, 0, 2.0)
            move_right_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0)
            current_state = "SM_START_NAVIGATION"
            
        elif current_state == "SM_START_NAVIGATION":
            print("Sending goal position to " + str(requested_location))
            go_to_goal_pose(requested_location[0], requested_location[1])
            goal_reached = False
            if requested_location == [3.13, 9.0]:
            	say("I'm going to navigate to the Kitchen")
            else :
            	say("I'm going to navigate to the table")	
            current_state = "SM_WAIT_FOR_MOVEMENT_FINISHED"
            
        elif current_state == "SM_WAIT_FOR_MOVEMENT_FINISHED":
            if goal_reached:
                print("Goal point reached")
                goal_reached = False
                current_state = "SM_LEAVE_OBJECT"

        elif current_state == "SM_LEAVE_OBJECT":
        ############################################################
            if requested_location == [3.13, 9.0] :
		move_base(0, 1.3, 2.0)
		move_base(1.0, 0, 1.0)
		current_state = "SM_LEAVE_IN_KITCHEN"
	    else :
	    	#move_base(0, 1.3, 2.0)
		move_base(1.0, -2.3, 1.4)
		current_state = "SM_LEAVE_IN_TABLE"
		
	elif current_state == "SM_LEAVE_IN_KITCHEN":
	    say("I'm going to leave the " + requested_object)
            if requested_object == 'pringles':
                move_left_arm(resp_ik.q1+0.6, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_left_gripper(0.3)
                move_left_arm(-1.2, 0, 0, 1.8, 0, 0.6, 0) #0.9
                move_left_gripper(-0.7)
                move_left_arm(0,0,0,0,0,0,0)
   		#####
   		print("Move backward")
	       	move_base(-0.3, 0, 3.0)
            else:
                move_right_arm(resp_ik.q1+0.6, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_right_gripper(0.3)
                move_right_arm(-1.2, 0, 0, 1.8, 0, 0.6, 0)
                move_right_gripper(-0.7)
                move_right_arm(0,0,0,0,0,0,0)
		print("Move backward")
	       	move_base(-0.3, 0, 3.0)
            go_to_goal_pose(3.39, 6.56)
            goal_reached = False
	    say("I'll be back")
            current_state = "SM_WAIT_FOR_RETURN"
            
            
        elif current_state == "SM_LEAVE_IN_TABLE":
       	    say("I'm going to leave the " + requested_object)
            if requested_object == 'pringles':
                move_left_arm(resp_ik.q1+0.6, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_left_gripper(0.3)
                move_left_arm(-0.6, 0, 0, 1.8, 0, 0.6, 0) 
                move_left_gripper(-0.7)
                move_left_arm(0,0,0,0,0,0,0)
		move_base(-0.3, 0, 1.0)
            else:
                move_right_arm(resp_ik.q1+0.6, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7)
                move_right_gripper(0.7)
                move_right_arm(-1.2, 0, 0, 1.8, 0, 0.6, 0)
                move_right_gripper(-0.7)
                move_right_arm(0,0,0,0,0,0,0)
		move_base(-0.3, 0, 3.0)
            go_to_goal_pose(3.39, 6.56)
            goal_reached = False
            say("I'll be back")
            current_state = "SM_WAIT_FOR_RETURN"
        

        elif current_state == "SM_WAIT_FOR_RETURN":
            if goal_reached:
                print("Return point reached")
                goal_reached = False
                current_state = "SM_APPROACH_TO_TABLE"

        elif current_state == "SM_APPROACH_TO_TABLE":
            go_to_goal_pose(3.39, 5.76)
            goal_reached = False
            current_state = "SM_WAIT_FOR_APPROACHING"

        elif current_state == "SM_WAIT_FOR_APPROACHING":
            if goal_reached:
                print("Start point reached")
                goal_reached = False
                move_base(0.2, 0, 1.3)
                current_state = "SM_INIT"
                executing_task = False
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
