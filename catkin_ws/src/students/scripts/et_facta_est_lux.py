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

NAME = "CHAVOLLA JIMENEZ RAUL DANIEL"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.data

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data

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
    msg = Float32MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubLaGoalPose.publish(msg)
    time.sleep(5.0)

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
    msg = Float32MultiArray()
    msg.data.append(q1)
    msg.data.append(q2)
    msg.data.append(q3)
    msg.data.append(q4)
    msg.data.append(q5)
    msg.data.append(q6)
    msg.data.append(q7)
    pubRaGoalPose.publish(msg)
    time.sleep(5.0)

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
    msg = Float32MultiArray()
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

def transform_point_to_left_arm(x,y,z):
    listener = tf.TransformListener()
    listener.waitForTransform("shoulders_left_link", "kinect_link", rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = "kinect_link"
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z 

    target_frame = "shoulders_left_link"
    obj_p = listener.transformPoint(target_frame, obj_p)
    return obj_p.point.x, obj_p.point.y, obj_p.point.z

def transform_point_to_right_arm(x,y,z):
    listener = tf.TransformListener()
    listener.waitForTransform("shoulders_left_link", "kinect_link", rospy.Time(), rospy.Duration(4.0))
    obj_p = PointStamped()
    obj_p.header.frame_id = "kinect_link"
    obj_p.header.stamp = rospy.Time(0)
    obj_p.point.x, obj_p.point.y, obj_p.point.z = x,y,z
    target_frame = "shoulders_right_link"
    obj_p = listener.transformPoint(target_frame, obj_p)
    return obj_p.point.x, obj_p.point.y, obj_p.point.z

def find_object(obj_name):
    time.sleep(2.0)
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)
    req_find_object = FindObjectRequest()
    req_find_object.cloud = rospy.wait_for_message("/kinect/points", PointCloud2)
    req_find_object.name  = obj_name
    resp_find_object = clt_find_object(req_find_object)
    print("Object found at: " + str([resp_find_object.x, resp_find_object.y, resp_find_object.z]))
    return resp_find_object.x, resp_find_object.y, resp_find_object.z

def ik_left_arm(x,y,z):
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    req_ik = InverseKinematicsRequest()
    req_ik.x, req_ik.y, req_ik.z = x,y,z
    req_ik.roll, req_ik.pitch, req_ik.yaw = 3.0, -1.57, -3.0
    resp_ik = clt_la_inverse_kin(req_ik)
    return resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7

def ik_right_arm(x,y,z):
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    req_ik = InverseKinematicsRequest()
    req_ik.x, req_ik.y, req_ik.z = x,y,z
    req_ik.roll, req_ik.pitch, req_ik.yaw = 3.0, -1.57, -3.0
    resp_ik = clt_ra_inverse_kin(req_ik)
    return resp_ik.q1, resp_ik.q2, resp_ik.q3, resp_ik.q4, resp_ik.q5, resp_ik.q6, resp_ik.q7


def main():
    global new_task, recognized_speech, executing_task, goal_reached
    global pubLaGoalPose, pubRaGoalPose, pubHdGoalPose, pubLaGoalGrip, pubRaGoalGrip
    global pubGoalPose, pubCmdVel, pubSay
    print("FINAL PROJECT - " + NAME)
    rospy.init_node("final_exercise")
    rospy.Subscriber('/recognized', String, callback_recognized_speech)
    rospy.Subscriber('/navigation/goal_reached', Bool, callback_goal_reached)
    pubGoalPose = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=10)
    pubCmdVel   = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    pubSay      = rospy.Publisher('/robotsound', SoundRequest, queue_size=10)
    pubLaGoalPose = rospy.Publisher("/hardware/left_arm/goal_pose" , Float32MultiArray, queue_size=10);
    pubRaGoalPose = rospy.Publisher("/hardware/right_arm/goal_pose", Float32MultiArray, queue_size=10);
    pubHdGoalPose = rospy.Publisher("/hardware/head/goal_pose"     , Float32MultiArray, queue_size=10);
    pubLaGoalGrip = rospy.Publisher("/hardware/left_arm/goal_gripper" , Float32, queue_size=10);
    pubRaGoalGrip = rospy.Publisher("/hardware/right_arm/goal_gripper", Float32, queue_size=10);
    listener = tf.TransformListener()
    loop = rospy.Rate(10)
    print("Waiting for services...")
    rospy.wait_for_service('/manipulation/la_inverse_kinematics')
    rospy.wait_for_service('/vision/find_object')
    print("Services are now available.")
    clt_la_inverse_kin = rospy.ServiceProxy("/manipulation/la_inverse_kinematics", InverseKinematics)
    clt_ra_inverse_kin = rospy.ServiceProxy("/manipulation/ra_inverse_kinematics", InverseKinematics)
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)

    new_task = False
    executing_task = False
    recognized_speech = ""
    goal_reached = False

    current_state = "SM_INIT"
    requested_object   = ""
    requested_location = [0,0]

    #
    # FINAL PROJECT
    # SE AGREGA LA MAQUINA DE ESTADOS
    #
  
    while not rospy.is_shutdown():
        if current_state == "SM_INIT": #State 0
            print("Starting state machine :D ...")
            if(new_task == True):
                new_task = False
                current_state = "SM_SAY_HELLO"

        elif current_state == "SM_SAY_HELLO": #STATE 1
            print("Saying hello")
	    
            say("Hello world")
            #current_state = "SM_MOVE_ARM"
            current_state = "SM_OBJECT_DETECTION"

        elif current_state == "SM_OBJECT_DETECTION":#STATE 2
            print("Recognizing Object")
            say("Robot is recognizing object")
            obj,loc = parse_command(recognized_speech) #Se reconce el voz
            print(obj)
            print(loc)
            current_state = "SM_HEAD_DOWN"

        elif current_state == "SM_HEAD_DOWN":#STATE 3
            print("Moving head")
            say("Im sad") #CAMBIAR
            move_head(0,-0.9)
            current_state = "SM_FIND_OBJECT"

        elif current_state == "SM_FIND_OBJECT":
            print("Finding object")
            say("Finding object")
            x_obj, y_obj, z_obj = find_object(obj)
            print(x_obj, y_obj, z_obj)
            current_state = "SM_POINT_TRANSFORM"

        elif current_state == "SM_POINT_TRANSFORM":
	    print("transformacion")
            if obj == "pringles":
                xt, yt, zt = transform_point_to_left_arm(x_obj, y_obj, z_obj)
		
            else:
                xt, yt, zt = transform_point_to_right_arm(x_obj, y_obj, z_obj)
            print(xt, yt, zt)
            current_state = "SM_INVERSE_KINEMATICS"

        elif current_state == "SM_INVERSE_KINEMATICS":
	    print("SIK")
            if obj == "pringles":
                q = ik_left_arm(xt,yt,zt)
		print("Llegue")
            else:
                q = ik_right_arm(xt,yt,zt)
            print(q)

            current_state = "SM_MOVE_ARM_TO_START"

        elif current_state == "SM_MOVE_ARM_TO_START":
             print("Moving the robot's arm")
             print("posicion inicial")
	     move_left_arm(-0.5,0,0,2.3,0,0,0)
             current_state = "SM_MOVE_ARM_TO_TAKE_OBJ"

        elif current_state == "SM_MOVE_ARM_TO_TAKE_OBJ":
            print("moving_arm_obj")
            say("Robot is moving its arm")
            if obj == "pringles":
		move_left_gripper(0.5)
                move_left_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            else:
		move_right_gripper(0.5)
                move_right_arm(q[0],q[1],q[2],q[3],q[4],q[5],q[6])
            current_state = "SM_CLOSE_HAND"
        
        elif current_state == "SM_CLOSE_HAND":
            print("closing hand")
            if obj == "pringles":
                move_left_arm(-0,0,0,1.7,0,0,0)
                move_left_gripper(-0.5)
		move_left_arm(0,0,0,3,0,0,-1)
 	    else:
		move_right_arm(-0,0,0,1.7,0,0,0)
                move_right_gripper(-0.5)
		move_right_arm(0,0,0,3,0,0,-0.8)
		
	    say("I got it")
            #else:
                #move_right_gripper(-3)
	    
            
	   
	    current_state = "SM_MOVE_TO_GOAL_POSE"
	elif current_state == "SM_MOVE_TO_GOAL_POSE" :
	   go_to_goal_pose(loc[0],loc[1])
	   if goal_reached:
	   	if obj == "pringles":
	   		move_left_gripper(0.5)
	   current_state = "SM_RETURN"
        elif current_state == "SM_RETURN" :
            go_to_goal_pose(3.32,5.86)
	    print("estoy girando")
	    move_base(0,1,15)
	    print("ya gire")
	    move_left_gripper(0)

            
            current_state = "SM_FINISH"
	
         
 
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
  
