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
translation = (0.00, 0.18, 0.84)  # Position der Kamera im Weltkoordinatensystem X/Y/Z
rotation = quaternion_from_euler(-math.pi/2-((17*math.pi)/180), 0, math.pi)  # Orientierung der Kamera im Weltkoordinatensystem Roll/Pitch/Yaw

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

        # Get coordinates of the left shoulder (index 12)
        left_shoulder_landmark = results.pose_landmarks.landmark[11]
        x_left_shoulder = int(left_shoulder_landmark.x * color_image.shape[1])
        y_left_shoulder = int(left_shoulder_landmark.y * color_image.shape[0])

        # Ensure valid coordinates (clamp within image bounds)
        x_left_shoulder = max(0, min(x_left_shoulder, color_image.shape[1] - 1))
        y_left_shoulder = max(0, min(y_left_shoulder, color_image.shape[0] - 1))

        # Get coordinates of the left elbow (index 14)
        left_elbow_landmark = results.pose_landmarks.landmark[13]
        x_left_elbow = int(left_elbow_landmark.x * color_image.shape[1])
        y_left_elbow = int(left_elbow_landmark.y * color_image.shape[0])

        # Ensure valid coordinates (clamp within image bounds)
        x_left_elbow = max(0, min(x_left_elbow, color_image.shape[1] - 1))
        y_left_elbow = max(0, min(y_left_elbow, color_image.shape[0] - 1))

        # Get coordinates of the left hand (index 16)
        left_hand_landmark = results.pose_landmarks.landmark[15]
        x_left_hand = int(left_hand_landmark.x * color_image.shape[1])
        y_left_hand = int(left_hand_landmark.y * color_image.shape[0])

        # Ensure valid coordinates (clamp within image bounds)
        x_left_hand = max(0, min(x_left_hand, color_image.shape[1] - 1))
        y_left_hand = max(0, min(y_left_hand, color_image.shape[0] - 1))

        # Calculate the 3D position of the left shoulder
        left_shoulder_distance = depth_image[y_left_shoulder , x_left_shoulder ] * depth_scale
        z_left_shoulder  = left_shoulder_distance
        x_world_left_shoulder  = (x_left_shoulder  - cx) * z_left_shoulder  / fx
        y_world_left_shoulder  = (y_left_shoulder  - cy) * z_left_shoulder  / fy

        # Calculate the 3D position of the left elbow
        left_elbow_distance = depth_image[y_left_elbow , x_left_elbow ] * depth_scale
        z_left_elbow  = left_elbow_distance
        x_world_left_elbow  = (x_left_elbow  - cx) * z_left_elbow  / fx
        y_world_left_elbow  = (y_left_elbow  - cy) * z_left_elbow  / fy

        # Calculate the 3D position of the left hand
        left_hand_distance = depth_image[y_left_hand , x_left_hand ] * depth_scale
        z_left_hand  = left_hand_distance
        x_world_left_hand  = (x_left_hand  - cx) * z_left_hand  / fx
        y_world_left_hand  = (y_left_hand  - cy) * z_left_hand  / fy

        # Create a PointStamped message for the left shoulder in camera frame
        left_shoulder_point = PointStamped()
        left_shoulder_point.header.frame_id = "camera_link"
        left_shoulder_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        left_shoulder_point.point.x = x_world_left_shoulder 
        left_shoulder_point.point.y = y_world_left_shoulder
        left_shoulder_point.point.z = z_left_shoulder

        # Create a PointStamped message for the left elbow in camera frame
        left_elbow_point = PointStamped()
        left_elbow_point.header.frame_id = "camera_link"
        left_elbow_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        left_elbow_point.point.x = x_world_left_elbow 
        left_elbow_point.point.y = y_world_left_elbow
        left_elbow_point.point.z = z_left_elbow 

        # Create a PointStamped message for the left hand in camera frame
        left_hand_point = PointStamped()
        left_hand_point.header.frame_id = "camera_link"
        left_hand_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        left_hand_point.point.x = x_world_left_hand 
        left_hand_point.point.y = y_world_left_hand
        left_hand_point.point.z = z_left_hand 

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
            listener.waitForTransform("world", "camera_link", left_shoulder_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("world", left_shoulder_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "left_shoulder",
                "world"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("world", "camera_link", left_elbow_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("world", left_elbow_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "left_elbow",
                "world"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("world", "camera_link", left_hand_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("world", left_hand_point)

            # Publish the transformed point
            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (keine Rotation)
                rospy.Time.now(),
                "left_hand",
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