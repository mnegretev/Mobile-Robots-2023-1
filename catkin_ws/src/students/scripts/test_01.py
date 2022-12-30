#!/usr/bin/env python3
#
# MOBILE ROBOTS - FI-UNAM, 2022-2
# Testing proyect 01 - THE PLATFORM ROS 
#

import rospy
from std_msgs.msg import String

NAME = "Alejandro Miranda Hernandez"

def main():
    print("Testing 01  taking pringles to the table - " + NAME)
    rospy.init_node("test_01")
    pub = rospy.Publisher("/test", String, queue_size=10)
    loop = rospy.Rate(10)

    while not rospy.is_shutdown():
        msg=String()
        msg.data = 'PRINGLES TABLE'
        pub.publish(msg)
        #print('Sended')
        loop.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
    
