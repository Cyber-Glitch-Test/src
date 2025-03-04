#!/usr/bin/env python
import random
import rospy # type: ignore
import tf  # type: ignore
import pyrealsense2 as rs  # type: ignore
import mediapipe as mp  # type: ignore
import cv2  # type: ignore
import numpy as np  # type: ignore
import math
from geometry_msgs.msg import PointStamped  # type: ignore
from tf.transformations import quaternion_from_euler  # type: ignore
from filterpy.kalman import KalmanFilter  # type: ignore
from std_srvs.srv import Empty  # type: ignore

def create_kalman_filter():
    kf = KalmanFilter(dim_x=6, dim_z=3)
    kf.F = np.array([[1, 0, 0, 1, 0, 0],  
                      [0, 1, 0, 0, 1, 0],  
                      [0, 0, 1, 0, 0, 1],  
                      [0, 0, 0, 1, 0, 0],  
                      [0, 0, 0, 0, 1, 0],  
                      [0, 0, 0, 0, 0, 1]])
    kf.H = np.array([[1, 0, 0, 0, 0, 0],  
                      [0, 1, 0, 0, 0, 0],  
                      [0, 0, 1, 0, 0, 0]])
    kf.P *= 700  # Initial hohe Unsicherheit
    kf.R = np.eye(3) * 0.1  # Messrauschen
    kf.Q = np.eye(6) * 0.1  # Prozessrauschen
    kf.x = np.zeros((6, 1))  # Anfangszustand
    return kf

kf_shoulder = create_kalman_filter()
kf_elbow = create_kalman_filter()
kf_hand = create_kalman_filter()

# Set up your ROS node and TransformBroadcaster
rospy.init_node('Stereo_Cam')
#rospy.Service('/shoulder_tracking_service', Empty, lambda req: None)
broadcaster = tf.TransformBroadcaster()
translation = (-0.25, -0.28, 0.80)   # Position der Kamera im Weltkoordinatensystem X/Y/Z
rotation = quaternion_from_euler(((-22/180)*math.pi),((0/180)*math.pi),((0/180)*math.pi)) # Orientierung der Kamera im Weltkoordinatensystem Roll/Pitch/Yaw
#rotation = quaternion_from_euler(-math.pi, ((17*math.pi)/180), math.pi) 

# Initialisierte Realsense Kamera
realsense_ctx = rs.context()
connected_devices = []  # List of serial numbers for present cameras
for i in range(len(realsense_ctx.devices)):
    detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    print(f"{detected_camera}")
    connected_devices.append(detected_camera)
device = connected_devices[0]
pipeline = rs.pipeline()
config = rs.config()

# Initialize MediaPipe Pose
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# Configure your Realsense Camera stream
config.enable_device(device)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8,30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# Depth scale for conversion
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Camera Intrinsics (for coordinate transformation)
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy = intrinsics.fx, intrinsics.fy  # Focal lengths
cx, cy = intrinsics.ppx, intrinsics.ppy  # Principal point

# Start the ROS loop
rate = rospy.Rate(30)  # 30 Hz
listener = tf.TransformListener()  # TransformListener initialisieren

while not rospy.is_shutdown():

        right_shoulder_point = PointStamped()
        right_shoulder_point.header.frame_id = "camera_link"
        right_shoulder_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel

        #if not all(x == 0 for x in elbow_trans):
            # Create a PointStamped message for the right elbow in camera frame
        right_elbow_point = PointStamped()
        right_elbow_point.header.frame_id = "camera_link"
        right_elbow_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
 
        #if not all(x == 0 for x in hand_trans):
            # Create a PointStamped message for the right hand in camera frame
        right_hand_point = PointStamped()
        right_hand_point.header.frame_id = "camera_link"
        right_hand_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel


        "zum testen feste Koords"
        right_shoulder_point.point.x = 0
        right_shoulder_point.point.y = 0.6
        right_shoulder_point.point.z = 0.6

        right_elbow_point.point.x = 0
        right_elbow_point.point.y = 0.5
        right_elbow_point.point.z = 0.45

        right_hand_point.point.x = 0
        right_hand_point.point.y = 0.45
        right_hand_point.point.z = 0.45

        # Veröffentliche den Frame "camera_link" im TF-Baum
        broadcaster.sendTransform(
            translation,  # Position der Kamera im Weltkoordinatensystem
            rotation,     # Orientierung der Kamera im Weltkoordinatensystem
            rospy.Time.now(),  # Zeitstempel
            "camera_link",  # Child Frame (Kamera)
            "base"         # Parent Frame (Weltkoordinatensystem)
        )

        # Warte auf die TF-Daten
        try:
            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("base", "camera_link", right_shoulder_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", right_shoulder_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "right_shoulder",
                "base"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("base", "camera_link", right_elbow_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", right_elbow_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "right_elbow",
                "base"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("base", "camera_link", right_hand_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", right_hand_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "right_hand",
                "base"
            )

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Error transforming point: {e}")

    # Display image with pose landmarks




