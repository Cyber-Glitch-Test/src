#!/usr/bin/env python

import rospy
import tf
import cv2
import numpy as np
import math
import mediapipe as mp
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_from_euler

# Set up ROS node and TransformBroadcaster
rospy.init_node('Stereo_Cam')
broadcaster = tf.TransformBroadcaster()
translation = (0.00, 0.18, 0.84)
rotation = quaternion_from_euler(-math.pi/2-((17*math.pi)/180), 0, math.pi)

# Initialize MediaPipe Pose
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils
bridge = CvBridge()

# Global variables for storing latest frames
depth_image = None
color_image = None

# Callback functions
def color_callback(msg):
    global color_image
    try:
        color_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except Exception as e:
        rospy.logwarn(f"Error converting color image: {e}")

def depth_callback(msg):
    global depth_image
    try:
        depth_image = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
    except Exception as e:
        rospy.logwarn(f"Error converting depth image: {e}")

# Subscribe to image topics
rospy.Subscriber("/camera/color/image_raw", Image, color_callback)
rospy.Subscriber("/camera/depth/image_rect_raw", Image, depth_callback)

rate = rospy.Rate(30)
listener = tf.TransformListener()

while not rospy.is_shutdown():
    if color_image is None or depth_image is None:
        continue

    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(color_image_rgb)

    if results.pose_landmarks:
        mpDraw.draw_landmarks(color_image, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

    cv2.imshow("Pose Landmarks", color_image)
    cv2.waitKey(1)
    rate.sleep()

cv2.destroyAllWindows()
