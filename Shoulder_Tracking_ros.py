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
translation = (0.09, -0.18, 0.84)  # Position der Kamera im Weltkoordinatensystem
rotation = quaternion_from_euler(-math.pi/2-((15*math.pi)/180), 0, -math.pi/2)  # Orientierung der Kamera im Weltkoordinatensystem Roll/Pitch/Yaw

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
        xr = int(right_shoulder_landmark.x * color_image.shape[1])
        yr = int(right_shoulder_landmark.y * color_image.shape[0])
        zr = right_shoulder_landmark.z

        # Calculate the 3D position of the right shoulder
        right_shoulder_distance = depth_image[yr, xr] * depth_scale
        z = right_shoulder_distance
        x_world = (xr - cx) * z / fx
        y_world = (yr - cy) * z / fy

        # Create a PointStamped message for the right shoulder in camera frame
        right_shoulder_point = PointStamped()
        right_shoulder_point.header.frame_id = "camera_link"
        right_shoulder_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_shoulder_point.point.x = x_world
        right_shoulder_point.point.y = y_world
        right_shoulder_point.point.z = z

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
                (0.0, 0.0, 0.0, 1.0),  # Quaternion (no rotation)
                rospy.Time.now(),
                "right_shoulder",
                "world"
            )

            print(f"Transformed Point: ({transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z})")

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Error transforming point: {e}")

    # Display image with pose landmarks
    cv2.imshow("Pose Landmarks", color_image)
    cv2.waitKey(1)

    rate.sleep()

# Stop the pipeline when done
pipeline.stop()