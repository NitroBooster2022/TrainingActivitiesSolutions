#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
from utils.msg import IMU
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError

class superIMUHandler():
    # ===================================== INIT==========================================
    def __init__(self):
        rospy.init_node('superIMUnod', anonymous=True)
        self.imu_sub = rospy.Subscriber("/automobile/IMU", IMU, self.callback)
        self.super_control = rospy.Publisher("/automobile/command", String, queue_size=1 )
        rospy.spin()

    def callback(self, data):
        if(0 < data.yaw < np.pi):
            self.super_control.publish('{"action":"1","speed":0.1}')
        else:
            self.super_control.publish('{"action":"1","speed":-0.1}')
            

        
    
            
if __name__ == '__main__':
    try:
        nod = superIMUHandler()
    except rospy.ROSInterruptException:
        pass