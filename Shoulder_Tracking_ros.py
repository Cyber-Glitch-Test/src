#!/usr/bin/env python

import rospy
import tf
import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np
import math
from geometry_msgs.msg import PointStamped
from tf.transformations import quaternion_from_euler

# Set up your ROS node and TransformBroadcaster
rospy.init_node('Stereo_Cam')
broadcaster = tf.TransformBroadcaster()
translation = (0.09, -0.18, 0.84)  # Position der Kamera im Weltkoordinatensystem X/Y/Z
rotation = quaternion_from_euler(-math.pi/2-((15*math.pi)/180), 0, math.pi)  # Orientierung der Kamera im Weltkoordinatensystem Roll/Pitch/Yaw

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
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
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
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not color_frame:
        continue

    # Convert frames to numpy arrays
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image = cv2.flip(depth_image,-1)
    color_image = np.asanyarray(color_frame.get_data())
    color_image = cv2.flip(color_image,-1)

    # Process Pose using MediaPipe
    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(color_image_rgb)

    if results.pose_landmarks:
        mpDraw.draw_landmarks(color_image, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

        # Get coordinates of the right shoulder (index 12)
        right_shoulder_landmark = results.pose_landmarks.landmark[12]
        x_right_shoulder = int(right_shoulder_landmark.x * color_image.shape[1])
        y_right_shoulder = int(right_shoulder_landmark.y * color_image.shape[0])

        # Get coordinates of the right elbow (index 14)
        right_elbow_landmark = results.pose_landmarks.landmark[14]
        x_right_elbow = int(right_elbow_landmark.x * color_image.shape[1])
        y_right_elbow = int(right_elbow_landmark.y * color_image.shape[0])

        # Get coordinates of the hand (index 16)
        right_hand_landmark = results.pose_landmarks.landmark[16]
        x_right_hand = int(right_hand_landmark.x * color_image.shape[1])
        y_right_hand = int(right_hand_landmark.y * color_image.shape[0])

        # Calculate the 3D position of the right shoulder
        right_shoulder_distance = depth_image[y_right_shoulder , x_right_shoulder ] * depth_scale
        z_right_shoulder  = right_shoulder_distance
        x_world_right_shoulder  = (x_right_shoulder  - cx) * z_right_shoulder  / fx
        y_world_right_shoulder  = (y_right_shoulder  - cy) * z_right_shoulder  / fy

        # Calculate the 3D position of the right elbow
        right_elbow_distance = depth_image[y_right_elbow , x_right_elbow ] * depth_scale
        z_right_elbow  = right_elbow_distance
        x_world_right_elbow  = (x_right_elbow  - cx) * z_right_elbow  / fx
        y_world_right_elbow  = (y_right_elbow  - cy) * z_right_elbow  / fy

        # Calculate the 3D position of the right hand
        right_hand_distance = depth_image[y_right_hand , x_right_hand ] * depth_scale
        z_right_hand  = right_hand_distance
        x_world_right_hand  = (x_right_hand  - cx) * z_right_hand  / fx
        y_world_right_hand  = (y_right_hand  - cy) * z_right_hand  / fy

        # Create a PointStamped message for the right shoulder in camera frame
        right_shoulder_point = PointStamped()
        right_shoulder_point.header.frame_id = "camera_link"
        right_shoulder_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_shoulder_point.point.x = x_world_right_shoulder 
        right_shoulder_point.point.y = y_world_right_shoulder
        right_shoulder_point.point.z = z_right_shoulder

        # Create a PointStamped message for the right elbow in camera frame
        right_elbow_point = PointStamped()
        right_elbow_point.header.frame_id = "camera_link"
        right_elbow_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_elbow_point.point.x = x_world_right_elbow 
        right_elbow_point.point.y = y_world_right_elbow
        right_elbow_point.point.z = z_right_elbow 

        # Create a PointStamped message for the right hand in camera frame
        right_hand_point = PointStamped()
        right_hand_point.header.frame_id = "camera_link"
        right_hand_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_hand_point.point.x = x_world_right_hand 
        right_hand_point.point.y = y_world_right_hand
        right_hand_point.point.z = z_right_hand 

        # Veröffentliche den Frame "camera_link" im TF-Baum
        broadcaster.sendTransform(
            translation,  # Position der Kamera im Weltkoordinatensystem
            rotation,     # Orientierung der Kamera im Weltkoordinatensystem
            rospy.Time.now(),  # Zeitstempel
            "camera_link",  # Child Frame (Kamera)
            "world"         # Parent Frame (Weltkoordinatensystem)
        )

        # Warte auf die TF-Daten
        try:
            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("world", "camera_link", right_shoulder_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("world", right_shoulder_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "right_shoulder",
                "world"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("world", "camera_link", right_elbow_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("world", right_elbow_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "right_elbow",
                "world"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("world", "camera_link", right_hand_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("world", right_hand_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "right_hand",
                "world"
            )

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Error transforming point: {e}")

    # Display image with pose landmarks
    cv2.imshow("Pose Landmarks", color_image)
    cv2.waitKey(1)

    rate.sleep()

# Stop the pipeline when done
pipeline.stop()