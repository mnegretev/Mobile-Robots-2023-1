#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float32MultiArray, Float32, Float64, Float64MultiArray
from sensor_msgs.msg import JointState

def callback_joint_state(msg):
    global pubLaCurrentPose, pubRaCurrentPose, pubLaCurrentGrip, pubRaCurrentGrip, pubHdCurrentPose
    hd_pose = Float64MultiArray()
    la_pose = Float64MultiArray()
    ra_pose = Float64MultiArray()
    hd_pose.data.append(msg.position[0])
    hd_pose.data.append(msg.position[1])
    for i in range(7):
        la_pose.data.append(msg.position[2+i])
    for i in range(7):
        ra_pose.data.append(msg.position[11+i])
    pubHdCurrentPose.publish(hd_pose)
    pubLaCurrentPose.publish(la_pose)
    pubRaCurrentPose.publish(ra_pose)

def callback_la_goal_pose(msg):
    global pubLaAngle1, pubLaAngle1, pubLaAngle2, pubLaAngle3, pubLaAngle4, pubLaAngle5, pubLaAngle6, pubLaAngle7 
    if len(msg.data) != 7:
        print("TopicResender.->ERROR!!! left arm goal pose must be a 7-value float32 array. ")
        return
    pubLaAngle1.publish(msg.data[0])
    pubLaAngle2.publish(msg.data[1])
    pubLaAngle3.publish(msg.data[2])
    pubLaAngle4.publish(msg.data[3])
    pubLaAngle5.publish(msg.data[4])
    pubLaAngle6.publish(msg.data[5])
    pubLaAngle7.publish(msg.data[6])

def callback_ra_goal_pose(msg):
    global pubRaAngle1, pubRaAngle1, pubRaAngle2, pubRaAngle3, pubRaAngle4, pubRaAngle5, pubRaAngle6, pubRaAngle7
    if len(msg.data) != 7:
        print("TopicResender.->ERROR!!! right arm goal pose must be a 7-value float32 array. ")
        return
    pubRaAngle1.publish(msg.data[0])
    pubRaAngle2.publish(msg.data[1])
    pubRaAngle3.publish(msg.data[2])
    pubRaAngle4.publish(msg.data[3])
    pubRaAngle5.publish(msg.data[4])
    pubRaAngle6.publish(msg.data[5])
    pubRaAngle7.publish(msg.data[6])

def callback_la_goal_gripper(msg):
    pubLaAngleGr.publish(msg.data)
    pubLaAngleGl.publish(msg.data)

def callback_ra_goal_gripper(msg):
    pubRaAngleGr.publish(msg.data)
    pubRaAngleGl.publish(msg.data)

def callback_head_goal_pose(msg):
    if len(msg.data) != 2:
        print("TopicResender.->ERROR!!! head goal pose must be a 2-value float32 array. ")
        return
    pubHdPan.publish(msg.data[0])
    pubHdTilt.publish(msg.data[1])    

def main():
    global pubLaAngle1, pubLaAngle1, pubLaAngle2, pubLaAngle3, pubLaAngle4, pubLaAngle5, pubLaAngle6, pubLaAngle7 
    global pubRaAngle1, pubRaAngle1, pubRaAngle2, pubRaAngle3, pubRaAngle4, pubRaAngle5, pubRaAngle6, pubRaAngle7 
    global pubLaAngleGl, pubLaAngleGr, pubRaAngleGl, pubRaAngleGr, pubHdPan, pubHdTilt
    global pubLaCurrentPose, pubRaCurrentPose, pubLaCurrentGrip, pubRaCurrentGrip, pubHdCurrentPose
    print("INITIALIZING TOPIC RESENDER...")
    rospy.init_node("topic_resender")
    rospy.Subscriber("/joint_states", JointState, callback_joint_state)
    rospy.Subscriber("/hardware/left_arm/goal_pose",  Float64MultiArray, callback_la_goal_pose)
    rospy.Subscriber("/hardware/right_arm/goal_pose", Float64MultiArray, callback_ra_goal_pose)
    rospy.Subscriber("/hardware/left_arm/goal_gripper",  Float64, callback_la_goal_gripper)
    rospy.Subscriber("/hardware/right_arm/goal_gripper", Float64, callback_ra_goal_gripper)
    rospy.Subscriber("/hardware/head/goal_pose", Float64MultiArray, callback_head_goal_pose)
    pubLaCurrentPose = rospy.Publisher("/hardware/left_arm/current_pose" , Float64MultiArray, queue_size=10)
    pubRaCurrentPose = rospy.Publisher("/hardware/right_arm/current_pose", Float64MultiArray, queue_size=10)
    pubLaCurrentGrip = rospy.Publisher("/hardware/left_arm/current_gripper" , Float64, queue_size=10)
    pubRaCurrentGrip = rospy.Publisher("/hardware/right_arm/current_gripper", Float64, queue_size=10)
    pubHdCurrentPose = rospy.Publisher("/hardware/head/current_pose", Float64MultiArray, queue_size=10);
    pubLaAngle1  = rospy.Publisher("/la_1_controller/command", Float64, queue_size=10)
    pubLaAngle2  = rospy.Publisher("/la_2_controller/command", Float64, queue_size=10)
    pubLaAngle3  = rospy.Publisher("/la_3_controller/command", Float64, queue_size=10)
    pubLaAngle4  = rospy.Publisher("/la_4_controller/command", Float64, queue_size=10)
    pubLaAngle5  = rospy.Publisher("/la_5_controller/command", Float64, queue_size=10)
    pubLaAngle6  = rospy.Publisher("/la_6_controller/command", Float64, queue_size=10)
    pubLaAngle7  = rospy.Publisher("/la_7_controller/command", Float64, queue_size=10)
    pubRaAngle1  = rospy.Publisher("/ra_1_controller/command", Float64, queue_size=10)
    pubRaAngle2  = rospy.Publisher("/ra_2_controller/command", Float64, queue_size=10)
    pubRaAngle3  = rospy.Publisher("/ra_3_controller/command", Float64, queue_size=10)
    pubRaAngle4  = rospy.Publisher("/ra_4_controller/command", Float64, queue_size=10)
    pubRaAngle5  = rospy.Publisher("/ra_5_controller/command", Float64, queue_size=10)
    pubRaAngle6  = rospy.Publisher("/ra_6_controller/command", Float64, queue_size=10)
    pubRaAngle7  = rospy.Publisher("/ra_7_controller/command", Float64, queue_size=10)
    pubLaAngleGl = rospy.Publisher("/la_grip_left_controller/command" , Float64, queue_size=10)
    pubLaAngleGr = rospy.Publisher("/la_grip_right_controller/command", Float64, queue_size=10)
    pubRaAngleGl = rospy.Publisher("/ra_grip_left_controller/command" , Float64, queue_size=10)
    pubRaAngleGr = rospy.Publisher("/ra_grip_right_controller/command", Float64, queue_size=10)
    pubHdPan     = rospy.Publisher("/head_pan_controller/command",  Float64, queue_size=10)
    pubHdTilt    = rospy.Publisher("/head_tilt_controller/command", Float64, queue_size=10)
    rospy.spin()


if __name__ == '__main__':
    main()
    
