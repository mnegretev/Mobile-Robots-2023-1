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

NAME = "TRONCOSO MORENO JAVIER ADAN"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if executing_task:
        return
    new_task = True
    recognized_speech = msg.hypothesis[0]
    print("New command received: " + recognized_speech)

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
#
def callback_goal_reached(msg):
    global goal_reached
    goal_reached = msg.data
    print("Received goal reached: " + str(goal_reached))
    goal_flag = True

def parse_command(cmd):
    obj = "pringles" if "PRINGLES" in cmd else "drink"
    loc = [5.8,6.7] if "TABLE" in cmd else [3.22, 9.2]
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
    req_ik = InverseKinematicsRequest()
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
def calculate_inverse_kinematics_right(x,y,z,roll, pitch, yaw):
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

    #
    # FINAL PROJECT 
    #
    goal_reached = False
    new_task = False
    recognized_speech = ""
    executing_task = False
    state = "SM_INIT"
    while not rospy.is_shutdown():
        if state == "SM_INIT":
            say("Waiting for command")
            print("Initializing final project...")
            print("Waiting for spoken command...")
            state = "SM_WAIT_FOR_COMMAND"
        elif state == "SM_WAIT_FOR_COMMAND":
            if new_task:
                new_task = False
                executing_task = True
                state = "SM_PARSING"
        elif state == "SM_PARSING":
            obj, loc = parse_command(recognized_speech)
            print("Requested object: " + obj)
            print("Requested location: " + str(loc))
            state = "SM_MOVE_HEAD"
        elif state == "SM_MOVE_HEAD":
            move_head(0,-1)
            if obj == "pringles":
                state = "SM_MOVE_LEFT_ARM"
                say("Taking the pringles")
            else:
                state = "SM_MOVE_RIGHT_ARM"
                say("Taking the drink")
        elif state == "SM_MOVE_LEFT_ARM":
            move_left_arm(-1.3, 0.2, 0, 1.6, 0, 1.2, 0)
            move_base(0.2,0,0)
            move_left_gripper(0.5) 
            x,y,z = find_object(obj)
            print("Coordenas camara"+str([x,y,z]))
            x,y,z = transform_point(x,y,z,'realsense_link','shoulders_left_link')
            print("Coordenas transformadas"+str([x,y,z]))
            state = "SM_INVERSE_KINEMATICS"
            
            
        elif state == "SM_MOVE_RIGHT_ARM":
            move_right_arm(-1, -0.2, 0, 1.4, 1.1, 0,0)
            move_right_gripper(1.2)
            move_base(-0.2,0,0.8)
            print("Retrocediendo")
            move_base(0,0.8,0.28)
            print("Acomodando")
            time.sleep(2.0)
            x,y,z = find_object(obj)
            print("Coordenadas camara"+str([x,y,z]))
            x,y,z = transform_point(x,y,z,'realsense_link', 'shoulders_right_link')
            print("Coordenadas reales"+str([x,y,z]))
            state="SM_INVERSE_KINEMATICS"#Prueba
        
        elif state == "SM_INVERSE_KINEMATICS":
           if obj == "pringles":   
               try:
                    print("Before inverse_kinematics")
                    print("Objeto encontrado"+str([x,y,z]))
                    [q0,q1,q2,q3,q4,q5,q6] = calculate_inverse_kinematics_left(x+0.03, y+0.01, z+0.09, 0, -1.5, 0)
                    print("After inverse_kinematics")
                    print("Objeto encontrado"+str([x,y,z]))
                    move_left_arm(q0,q1,q2,q3,q4,q5,q6)
                    move_left_gripper(-0.3)
                    move_left_arm(-0.153,0.065,-0.197,2.550,-0.342,-0.608,0.362)
                    state = "SM_GO_BACK"
               except:
                   state = "SM_END"
                   print("An exception has ocurred")
           else:
                [q0,q1,q2,q3,q4,q5,q6] = calculate_inverse_kinematics_right(x, y, z+0.10, 0, -1.5, 0)
                move_right_arm(q0,q1,q2,q3,q4,q5,q6)
                state="SM_TOMAR"
           #state= "SM_GO_BACK"
           #state="SM_TOMAR"
        elif state == "SM_GO_BACK":
            say("Moving Backward")
            print("I'm gonna move backward")
            move_base(-0.2, 0, 2)
            print("I moved backward")
            move_head(0, 0)
            state = "SM_GO_FORWARD"
            
        elif state == "SM_GO_FORWARD":
            print("I am going to move forward")
            if loc == [3.22, 9.2]:#Para Kitchen
             go_to_goal_pose(loc[0], loc[1])
             print("Going to kitchen")
             say("Going to the kitchen")
             state = "SM_WAIT_FOR_GOAL_REACH"
            else:
             go_to_goal_pose(loc[0], loc[1])
             print("Going to table")
             say("Going to the table")
             state = "SM_WAIT_FOR_GOAL_REACH_TABLE"
        
        elif state == "SM_WAIT_FOR_GOAL_REACH":
         if goal_reached:
            state = "SM_GOAL_REACHED"
            
        elif state == "SM_WAIT_FOR_GOAL_REACH_TABLE":
          if goal_reached:
            state = "SM_GOAL_REACHED_TABLE"
        
        elif state == "SM_GOAL_REACHED_TABLE":
            say("Im next to the table")
            print("Destino alcanzado")
            print("Moviendo brazo")
            move_left_arm(1.4850,0.0940,-0.1140,2.0416,0.0060,-1.6820,0)
            state = "SM_DROP_TABLE"
        
        elif state == "SM_DROP_TABLE":
            say("Moving to dropping position")
            print("Acomodando Base")
            move_base(0,-5.14,4)
            move_head(0,-0.4)
            say("Dropping the pringles") 
            move_left_gripper(0.5)
            state = "SM_END"               
          
        elif state == "SM_GOAL_REACHED":
            print("Destino alcanzado")
            say("Im in the kitchen")
            move_base(0,3.14,2)
            print("Acomodando orientacion")  
            state = "SM_TABLE"
        
        elif state == "SM_TABLE": 
            say("Dropping the pringles")
            move_base(0.2,0,1.8)
            state = "SM_DROP"
            
        elif state == "SM_DROP":
            move_head(0,-1) 
            move_left_gripper(0.5)
            state = "SM_END"           
#Estados para brazo derecho        
        elif state == "SM_TOMAR":    
            move_base(0.2, 0, 0.5)
            print("Acercandome a la mesa")
            move_right_gripper(-0.6)
            move_right_arm(0.9170,0.135,-0.050,1.442,-0.315,0.008,0.150)
            state = "SM_RIGHT_ARM_GO_TO"
            
        elif state == "SM_RIGHT_ARM_GO_TO":    
            print("GOING TO REQUESTED POSITION")
            if loc == [3.22, 9.2]:#Para Kitchen
             go_to_goal_pose(loc[0], loc[1])
             say("Going to the kitchen")
             print("Going to kitchen")
             state = "SM_WAIT_FOR_GOAL_REACH_RIGHT_ARM"
            else:
             go_to_goal_pose(loc[0], loc[1])#PARA TABLE
             print("Going to table")
             say("Going to the table")
             state = "SM_WAIT_FOR_GOAL_REACH_TABLE_RIGHT_ARM"   
           
        elif state == "SM_WAIT_FOR_GOAL_REACH_RIGHT_ARM": #Espero hasta llegar a la cocina
         if goal_reached:
            say("I am in the kitchen")
            state = "SM_GOAL_REACHED_KITCHEN"
        
        elif state == "SM_GOAL_REACHED_KITCHEN":    #Cuando ya estoy en la cocina
            print("LLegue a la cocina")
            move_base(0,3.14,2)
            print("Acomodando orientacion")  
            state = "SM_DROP_KITCHEN_RIGHT_ARM"
        
        elif state =="SM_DROP_KITCHEN_RIGHT_ARM":
            say("Moving to dropping position")
            move_base(0.2,0,1.8)
            print("Acercandome a la mesa")
            state = "SM_DROP2_KITCHEN_RIGHT_ARM"
            
        elif state == "SM_DROP2_KITCHEN_RIGHT_ARM":
            say("Dropping the drink")
            move_right_gripper(0.5)
            state = "SM_END"    
            
        elif state =="SM_WAIT_FOR_GOAL_REACH_TABLE_RIGHT_ARM":
         if goal_reached:
            say("I am next to the table")
            state = "SM_GOAL_REACHED_TABLE_RIGHT_ARM"
        
        elif state == "SM_GOAL_REACHED_TABLE_RIGHT_ARM":
            print("He llegado a TABLE")
            #move_right_arm(1.4,0.135,-0.050,1.442,-0.315,0.008,0.150)
            #move_right_arm(1.4,0.135,-0.050,1.442,-0.915,0.008,0.150)
            move_right_arm(1.55,0.135,-0.050,1.442,-0.95,0.008,0.150)
            say("Moving to dropping position")
            state = "SM_TURN"
        
        elif state == "SM_TURN":
            print("Girando")
            move_base(0,7,5.3)
            state="SM_D_TABLE_RIGHT_ARM"
        
        elif state == "SM_D_TABLE_RIGHT_ARM":
            print("Soltando lata")
            say("Dropping the drink")
            move_head(0,-0.3) 
            move_right_gripper(0.5)
            state="SM_END"
                                       
        elif state == "SM_END":
            say(" I have acomplished the task")
            print("\n\nEL programa ha terminado!\n\n")
        else:
            print("FATAL ERROR!!! :'(")
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
