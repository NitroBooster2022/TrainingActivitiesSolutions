#! /usr/bin/env python3

import rospy
import json
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class LaneFollower():
    def __init__(self):
        self.point = np.array([0.5, 0.5])
        self.res = 3
        self.threshold = 40
        self.minlength = 7
        self.error_p = np.array([0.0, 0.9])
        self.error_w = 0.5
        self.p = 0.006
        self.maxspeed = 0.1
        """
        Initialize the lane follower node
        """
        rospy.init_node('lane_follower_node', anonymous=True)
        self.bridge = CvBridge()
        self.cmd_vel_pub = rospy.Publisher("/automobile/command", String, queue_size=1)
        self.image_sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback)

    def image_callback(self, data):
        """
        Callback function for the image processed topic
        :param data: Image data in the ROS Image format
        """
        # Convert the image to the OpenCV format
        image = self.bridge.imgmsg_to_cv2(data, "bgr8")

        # Extract the lanes from the image
        lanes = self.extract_lanes(image)

        # Determine the steering angle based on the lanes
        steering_angle = self.get_steering_angle(lanes)

        # Publish the steering command
        self.publish_cmd_vel(steering_angle)

    def extract_lanes(self, image):
        """
        Extract the lanes from the image
        :param image: Image data in the OpenCV format
        :return: Tuple containing the left and right lanes (each a list of points)
        """
        # Convert the image to grayscale and apply a blur to remove high frequency noise
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply a Canny edge detector to find the edges in the image
        edges = cv2.Canny(blur, 50, 150)

        # Create a mask for the region of interest (ROI) in the image where the lanes are likely to be
        mask = np.zeros_like(edges)
        ignore_mask_color = 255
        imshape = image.shape
        h = imshape[0]
        w = imshape[1]
        vertices = np.array([[(0,h*0.8), (self.point[0]*w, self.point[1]*h), (w,0.8*h)]], dtype=np.int32)
        cv2.fillPoly(mask, vertices, ignore_mask_color)
        masked_edges = cv2.bitwise_and(edges, mask)

        # Use Hough transform to detect lines in the image
        lines = cv2.HoughLinesP(masked_edges,self.res,np.pi/180,self.threshold,minLineLength=self.minlength)

        # Separate the lines into left and right lanes
        left=[]
        right=[]
        if lines is not None:
            for line in lines:
                x1,y1,x2,y2 = line.reshape(4)
                if abs(y2 - y1) > 0.01:
                    m = (x2-x1)/(y2-y1)
                    p_y = int(self.error_p[1]*h)
                    p_x = int(x1 + (p_y-y1)*m)
                    if p_x < w/2:
                        if p_x < int((self.error_p[0]+self.error_w)*w) and p_x > int(self.error_p[0]*w):
                            left.append(p_x)
                    else:
                        if p_x < int((1-self.error_p[0])*w) and p_x > int((1-self.error_p[0]-self.error_w)*w):
                            right.append(p_x)
        if len(left) == 0:
            left_lane = 0
        else:
            left_lane = np.mean(left)
        if len(right) == 0:
            right_lane = w
        else:
            right_lane = np.mean(right)
        return (left_lane+right_lane)/2


    def get_steering_angle(self, lanes):
        """
        Determine the steering angle based on the lanes
        :param lanes: Tuple containing the left and right lanes (each a list of points)
        :return: Steering angle in radians
        """
        # Calculate the center of the lanes
        lane_center = lanes
        # print("lane center:",lane_center)
        # Calculate the steering angle
        image_center = 640 / 2
        steering_angle = -(lane_center - image_center) * self.p
        steering_angle = np.clip(steering_angle, -0.53, 0.53)

        return steering_angle


    def publish_cmd_vel(self, steering_angle):
        """
        Publish the steering command to the cmd_vel topic
        :param steering_angle: Steering angle in radians
        """
        msg = String()
        msg2 = String()
        x = self.maxspeed + self.maxspeed*abs(steering_angle)/0.53
        msg.data = '{"action":"1","speed":'+str(x)+'}'
        msg2.data = '{"action":"2","steerAngle":'+str(-steering_angle*180/3.1415926)+'}'
        self.cmd_vel_pub.publish(msg)
        self.cmd_vel_pub.publish(msg2)

if __name__ == '__main__':
    try:
        node = LaneFollower()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
