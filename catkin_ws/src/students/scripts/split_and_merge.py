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
import numpy as np

NAME = "ALEJANDRO_MIRANDA_HERNANDEZ"

#
# Global variable 'speech_recognized' contains the last recognized sentence
#
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    if not executing_task:
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
    loc = 'table' if "TABLE" in cmd else 'kitchen'
    loc_coords = [8.0,8.5] if "TABLE" in cmd else [3.22, 9.72]
    return obj, loc, loc_coords

#
# This function sends the goal articular position to the left arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
#
def move_left_arm(q):
    global pubLaGoalPose
    q1,q2,q3,q4,q5,q6,q7 = q
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
def move_right_arm(q):
    global pubRaGoalPose
    q1,q2,q3,q4,q5,q6,q7 = q
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
    print('-' +     text)
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

def callback_test(msg):
    global lst, new_task
    str = msg.data
    lst = str.split()
    new_task = True


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
    #init vars
    recognized_speech = None
    new_task = False
    executing_task = False
    target_obj, target_loc, target_loc_poss = None, None, None
    target_object_x,target_object_y,target_object_z= None, None, None
    goal_reached = False


    while not rospy.is_shutdown():
        

        #Recibir comando
        if not new_task:
            if recognized_speech:
                target_obj, target_loc, target_loc_poss = parse_command(recognized_speech)
                new_task = True
                executing_task = 'Say_comand'
        
        
        #Decir comando
        elif executing_task == 'Say_comand':
            say('New command detected')
            time.sleep(2)
            say('Task')
            time.sleep(2)
            say('Take '+ target_obj + ' to the ' + target_loc)
            executing_task = 'Move_base_back'
            time.sleep(3)


        #Mover base atras
        elif executing_task == 'Move_base_back': 
            say('Moving back')
            move_base(-0.1, 0, 1.9)
            executing_task = 'Move_head'

        
        #Mover cabeza
        elif executing_task == 'Move_head':
            say('Looking down')
            move_head(0,-1.0)
            executing_task = 'Seek_obj'


        #Buscar objeto
        elif executing_task == 'Seek_obj':
            say('Looking for ' + target_obj)
            target_object_x,target_object_y,target_object_z = find_object(target_obj)
            target_object_x,target_object_y,target_object_z = find_object(target_obj)
            print('Recived coords',target_object_x,target_object_y,target_object_z)
            executing_task = 'move_predef_taking_obj'

        
        #Mover brazo posicion predefinida
        elif executing_task == 'move_predef_taking_obj':
            say('Moving arm to position one')
            if target_obj == 'pringles':
                q2 = [-1.2, 0,0,1.6, 0, 0.8, 0]
                move_left_arm(q2)
                q2 = [-0.4, 0,0,1.6, 0, 0.8, 0]
                move_left_arm(q2)
                #q2 = [0 , 0 ,0 ,1.6, 0, 0.4, 0]
                #move_left_arm(q2)
            else:
                q2 = [0,0,0,0,0,0,0]
                move_right_arm(q2)
                q2 = [-1.2, -0.2,0,1.6, 1, 0, 0]
                move_right_arm(q2)
                #q2 = [-0.4, -0.2,0,1.6, 1, 0, 0]
                #move_right_arm(q2)
                #q2 = [0 , -0.2 ,-0.1 ,1.4, 1, 0.2, 0]
                q2 = [0.1 , 0 ,0 ,1.7, 0, 0.4, 0]
                move_right_arm(q2)
                
            executing_task = 'open_griper_taking_obj'
        
       
        #Abrir Griper
        elif executing_task == 'open_griper_taking_obj':
            say('Open griper')
            if target_obj == 'pringles':
                move_left_gripper(0.3)
            else:
                move_right_gripper(0.3)
            executing_task = 'Transf_coords'
        
        
        #Transformar coordenadas
        elif executing_task == 'Transf_coords':
            say('Processing coordinates')
            if target_obj == 'pringles':
                target_object_x,target_object_y,target_object_z = transform_point(target_object_x,
                                                                                target_object_y,
                                                                                target_object_z,
                                                                                "realsense_link",
                                                                                'shoulders_left_link')
                target_object_x += 0.085
                target_object_z += 0.1
                target_object_y -= 0.02
            else:
                target_object_x,target_object_y,target_object_z = transform_point(target_object_x,
                                                                                target_object_y,
                                                                                target_object_z,
                                                                                'realsense_link',
                                                                                'shoulders_right_link')
                target_object_x += 0.07
                target_object_y -= 0.04
                target_object_z += 0.16
            executing_task = 'kinematics'
            print('New coords',target_object_x,target_object_y,target_object_z)
        
        #Cinematica inversa
        elif executing_task == 'kinematics':
            say('Doing math')
            if target_obj == 'pringles':
                q = calculate_inverse_kinematics_left(target_object_x,
                                                target_object_y,
                                                target_object_z,
                                                0, -1.5, 0)
            else:
                q = calculate_inverse_kinematics_right(target_object_x,
                                                target_object_y,
                                                target_object_z,
                                                0, -1.5, 0)
            executing_task = 'move_target_taking_obj'


        #Mover brazo posicion objetivo
        elif executing_task == 'move_target_taking_obj':
            say('Moving arm to target')
            q1 = np.array(q)
            if target_obj == 'pringles':
                q2 = np.array([0 , 0 ,0 ,1.6, 0, 0.4, 0])
                for k in range(4):
                    move_left_arm(q2.tolist())
                    q2 = (q2 + q1)/2
                move_left_arm(q)
            else:
                q2 = np.array([0.18 , 0 ,0 ,1.7, 0, 0.4, 0])
                for k in range(4):
                    move_right_arm(q2.tolist())
                    q2 = (q2 + q1)/2
                move_right_arm(q)
            executing_task = 'close_griper_taking_obj'


        #Cerrar Griper
        elif executing_task == 'close_griper_taking_obj':
            say('Taking object')
            if target_obj == 'pringles':
                move_left_gripper(-0.15)
            else:
                #move_base(0.05, 0, 0.2)
                move_right_gripper(-0.2)
            executing_task = 'move_predef_obj'

        
        #Mover brazo posicion predefinida
        elif executing_task == 'move_predef_obj':
            say('Moving arm to position two')
            if target_obj == 'pringles':
                q2 = np.array(q)
                q2[3] += 0.8
                q1 = np.array([-1.3, 0.2, 0.0, 1.6, 0.0, 1.2, 0.0])
                for k in range(4):
                    move_left_arm(q2.tolist())
                    q2 = (q2 + q1)/2
                move_left_arm(q1.tolist())
            else:
                q2 = np.array(q)
                q2[3] += 1
                q1 = np.array([-1.2, -0.2,0,1.6, 1, 0, 0])
                for k in range(4):
                    move_right_arm(q2.tolist())
                    q2 = (q2 + q1)/2
                move_right_arm(q1.tolist())
                #q2 = [-1.0, -0.2, 0.0, 1.4, 1.1, 0.0, 0.0]
                #move_right_arm(q2)
            executing_task = 'move_goal'


        #Mover hacia meta
        elif executing_task == 'move_goal':
            say('Moving to goal')
            go_to_goal_pose(target_loc_poss[0],target_loc_poss[1])
            executing_task = 'wait_goal_reached'


        #Esperar indicacion de llegar
        elif executing_task == 'wait_goal_reached':
            if(goal_reached):
                say('Goal reached')
                executing_task = 'open_griper_leaving_obj'

        #Abrir Griper
        elif executing_task == 'open_griper_leaving_obj':
            say('Droping object')
            if target_obj == 'pringles':
                move_left_gripper(0.1)
            else:
                move_right_gripper(0.1)
            executing_task = 'end'

        
        time.sleep(3)
        loop.sleep()
    
    
    #end While





        

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
