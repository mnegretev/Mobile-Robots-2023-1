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

NAME = "Suarez Martinez Enrique"

#
# Global variable 'speech_recognized' contains the last recognized sentence
# VE
# La variable global 'speech_recognized' contiene la última oración reconocida
def callback_recognized_speech(msg):
    global recognized_speech, new_task, executing_task
    recognized_speech = msg.hypothesis[0]
    print("New command received: " + recognized_speech)

#
# Global variable 'goal_reached' is set True when the last sent navigation goal is reached
# VE
# La variable global 'goal_reached' se establece en True cuando se alcanza el último 
# objetivo de navegación enviado
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

#VE
#Esta función envía la posición articular de destino al brazo izquierdo y duerme 2 segundos 
#para permitir que el brazo alcance la posición de destino.
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
#VE
# Esta función envía la posición angular del objetivo a la pinza izquierda y duerme 1 segundo
# para permitir que la pinza alcance el ángulo objetivo.
def move_left_gripper(q):
    global pubLaGoalGrip
    pubLaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal articular position to the right arm and sleeps 2 seconds
# to allow the arm to reach the goal position. 
# VE
# Esta funcion envia la posicion articular de meta al brazo derecho y duerme 2 segundos
# para permitir que el brazo alcance la posición deseada.
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
# VE
# Esta función envía la posición angular objetivo a la pinza derecha y duerme 1 segundo
# para permitir que la pinza alcance el ángulo objetivo.

def move_right_gripper(q):
    global pubRaGoalGrip
    pubRaGoalGrip.publish(q)
    time.sleep(1.0)

#
# This function sends the goal pan-tilt angles to the head and sleeps 1 second
# to allow the head to reach the goal position. 
# VE
# Esta función envía los ángulos de giro e inclinación del objetivo a la cabeza y duerme 1 segundo
# para permitir que la cabeza alcance la posición de destino.

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
# VE
# Esta función envía una velocidad lineal y angular a la base móvil para realizar
# movimientos de bajo nivel. La base móvil se moverá a las velocidades angulares lineales dadas
# durante un tiempo dado por 't'
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
# VE
# Esta función publica una posición de objetivo global. Este tema está suscrito por
# pratice04 y realiza la planificación y el seguimiento de rutas.
def go_to_goal_pose(goal_x, goal_y):
    global pubGoalPose
    goal_pose = PoseStamped()
    goal_pose.pose.orientation.w = 1.0
    goal_pose.pose.position.x = goal_x
    goal_pose.pose.position.y = goal_y
    pubGoalPose.publish(goal_pose)

#
# This function sends a text to be synthetized.
# VE
# Esta función envía un texto a sintetizar.
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
# VE
# Esta función llama al servicio de cálculo de cinemática inversa para brazo izquierdo (práctica 08)
# y devuelve la posición articular calculada.
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
# VE
# Esta función llama al servicio de cálculo de cinemática inversa para brazo derecho (práctica 08)
# y devuelve la posición articular calculada.
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
# VE
# Llama al servicio de búsqueda de objeto (práctica 08) y devuelve
# las coordenadas xyz del objeto solicitado w.r.t. "realsense_enlace"
def find_object(object_name):
    clt_find_object = rospy.ServiceProxy("/vision/find_object", FindObject)
    req_find_object = FindObjectRequest()
    req_find_object.cloud = rospy.wait_for_message("/hardware/realsense/points", PointCloud2)
    req_find_object.name  = object_name
    resp = clt_find_object(req_find_object)
    return [resp.x, resp.y, resp.z]

#
# Transforms a point xyz expressed w.r.t. source frame to the target frame
# VE
# Transforma un punto xyz expresado w.r.t. fotograma de origen al fotograma de destino
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
    print("Waiting for services1...")
    rospy.wait_for_service('/manipulation/ra_inverse_kinematics')
    print("Waiting for services2...")
    rospy.wait_for_service('/vision/find_object')
    print("Waiting for services3...")
    print("Services are now available.")

    #
    # FINAL PROJECT 
    #
    new_task = False
    recognized_speech = ""
    executing_task = False
    state = "St_inicio"

   

    while not rospy.is_shutdown():
        #Estado 0: Inicio
        if state == "St_inicio":
            say("Starting")
            print("iniciando proyecto final...")
            print("Esperando por un comando...")
            state = "St_comando"
        #Estado 1: Esperando comando 

        elif state == "St_comando":
            say("Command detected")
            if new_task:
                print("Comando recibido...")
                new_task = False
                executing_task = True
                state = "St_analizando"
        #Estado 2: Reconociendo comando 
        elif state == "St_analizando":
            print("Reconociendo Comando...")
            say("Parsing command")
            obj, loc = parse_command(recognized_speech)
            print("Objecto: " + obj)
            print("Localizacion: " + str(loc))
            state = "St_moverCabeza"

        #Estado 3: Mover la cabeza para identificar objetos
        elif state == "St_moverCabeza":
            say("Looking down")
            print("Moviendo Cabeza...")
            move_head(0,-0.9)
            state = "St_BuscarObjeto"

        #Estado 4: Buscar objetos 
        elif state == "St_BuscarObjeto":
            say("Found object")
            print("Buscando Objetos...")
            obj_x, obj_y, obj_z = find_object(obj)
            state = "St_Transfor_Coor"

        #Estado 5: Transformar Coordendas 
        elif state == "St_Transfor_Coor":
            say("transforming coordinates")
            print("Trasformando Coordenadas...")
            if obj == "pringles":
                print("Trasformando Coordenadas Brazo Izquierdo...")
                obj_x, obj_y, obj_z = transform_point(obj_x, obj_y, obj_z, "realsense_link", "shoulders_left_link")
            
            else:
                print("Trasformando Coordenadas Brazo Derecho...")
                obj_x, obj_y, obj_z = transform_point(obj_x, obj_y, obj_z, "realsense_link", "shoulders_right_link")

            state = "St_Cinematica_Inv"

        #Estado 6: Calculando Cinematica Inversa 
        elif state == "St_Cinematica_Inv":
            say("Calculating inverse kinematics")
            print("Calculando Cinematica Inversa...")
            if obj == "pringles":
                print("Calculando Cinematica Inversa Brazo Izquierdo...")
                CinematicaInv = calculate_inverse_kinematics_left (obj_x, obj_y, obj_z, 0, -1.5,0)
            else:
                print("Calculando Cinematica Inversa Brazo Derecho...")
                CinematicaInv = calculate_inverse_kinematics_right (obj_x, obj_y, obj_z, 0, -1.5,0)

            state = "St_pos_Init_Brazos"

        #Estado 7: Moviendo brazos a una posicion definida y abriendo gripper 
        elif state == "St_pos_Init_Brazos":
            say("Moving arms")
            print("Moviendo Brazos y abriendo gripper...")
            if obj == "pringles":
                move_left_gripper(0.1)
                move_left_arm(-1.3, 0.2, 0.0, 1.6, 0.0, 1.2, 0.0)
                print("Moviendo y abriendo gripper Brazo Izquierdo...")
            else:
                move_right_gripper(0.1)
                move_right_arm(-1.0, -0.2, 0.0, 1.4, 1.1, 0.0, 0.0)
                print("Moviendo y abriendo gripper Brazo Derecho...")
            state = "St_pos_Obj_Brazos"

        #Estado 8: Mover brazo al objeto 
        elif state == "St_pos_Obj_Brazos":
            print("Moviendo Brazo al objeto...")
            if obj == "pringles":
                print("Moviendo Brazo Izquierdo al objeto...")
                move_left_arm(CinematicaInv)
            else:
                print("Moviendo Brazo Derecho al objeto...")
                move_right_arm(CinematicaInv)
            state = "St_Cerrar_Griper"

        #Estado 9: Tomar el objeto 
        elif state == "St_Cerrar_Griper":
            say("Grabbed the object")
            print("Tomando Objetos...")
            if obj == "pringles":
                print("Tomando Objetos Brazo Izquierdo...")
                move_left_gripper(-0.1)               
            else:
                print("Tomando Objetos Brazo Derecho...")
                move_right_gripper(-0.1)
            state = "St_regresando_Pos_Init_Br"

        #Estado 10: Regresar brazos a posicion definida 
        elif state == "St_regresando_Pos_Init_Br":
            print("Regresando Brazos y moviendo cabeza...")
            if obj == "pringles":
                print("Regresando Brazos Izquierdo...")
                move_left_arm(-1.3, 0.2, 0.0, 1.6, 0.0, 1.2, 0.0)
                move_head (0, 0)
               
            else:
                print("Regresando Brazos Derecho...")
                move_right_arm(-1.0, -0.2, 0.0, 1.4, 1.1, 0.0, 0.0)
                move_head (0, 0)
            state = "St_transportar"

        #Estado 11: Llevar el objeto a otra localizacion 
        elif state == "St_transportar":
            say("Transport object")
            print("Transportando objeto...")
            move_base(-0.2, 0, 2)
            go_to_goal_pose(loc[0],loc[1])
            state = "St_Parar"

        #Estado 12: Soltar objeto 
        elif state == "St_Parar":
            say("Drop object")
            print("Parar y soltar obejto...")
            if goal_reached:
                if obj == "pringles":
                    move_left_gripper(0.1)
                else:
                    move_right_gripper(0.1)
                
                state = "St_regresando_loc"    

        #Estado 13: Regresano a localizacion original
        elif state == "St_regresando_loc":
            print("Regresando...")
            say("Returning")
            go_to_goal_pose(3,5.8)
            move_base(0,1,15)
            if obj == "pringles":
                move_left_gripper(0)
            else:
                move_right_gripper(0)
            state = "SM_END"
                

        #Estado 14: Fin 
        elif state == "SM_END":
            say("I finished")


        else:
            print("FATAL ERROR!!! :'(")


        loop.sleep()

    while not rospy.is_shutdown():
        loop.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
