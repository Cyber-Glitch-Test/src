#!/usr/bin/env python

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
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not color_frame:
        continue

    # Convert frames to numpy arrays
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    #depth_image = cv2.flip(depth_image,-1)
    color_image = np.asanyarray(color_frame.get_data())
    #color_image = cv2.flip(color_image,-1)

    # Process Pose using MediaPipe
    color_image_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
    results = pose.process(color_image_rgb)

    if results.pose_landmarks:
        mpDraw.draw_landmarks(color_image, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

        # Get coordinates of the right shoulder (index 12)
        right_shoulder_landmark = results.pose_landmarks.landmark[11]
        x_right_shoulder = int(right_shoulder_landmark.x * color_image.shape[1])
        y_right_shoulder = int(right_shoulder_landmark.y * color_image.shape[0])

        # Ensure valid coordinates (clamp within image bounds)
        x_right_shoulder = max(0, min(x_right_shoulder, color_image.shape[1] - 1))
        y_right_shoulder = max(0, min(y_right_shoulder, color_image.shape[0] - 1))

        # Get coordinates of the right elbow (index 14)
        right_elbow_landmark = results.pose_landmarks.landmark[13]
        x_right_elbow = int(right_elbow_landmark.x * color_image.shape[1])
        y_right_elbow = int(right_elbow_landmark.y * color_image.shape[0])

        # Ensure valid coordinates (clamp within image bounds)
        x_right_elbow = max(0, min(x_right_elbow, color_image.shape[1] - 1))
        y_right_elbow = max(0, min(y_right_elbow, color_image.shape[0] - 1))

        # Get coordinates of the right hand (index 16)
        right_hand_landmark = results.pose_landmarks.landmark[15]
        x_right_hand = int(right_hand_landmark.x * color_image.shape[1])
        y_right_hand = int(right_hand_landmark.y * color_image.shape[0])

        # Ensure valid coordinates (clamp within image bounds)
        x_right_hand = max(0, min(x_right_hand, color_image.shape[1] - 1))
        y_right_hand = max(0, min(y_right_hand, color_image.shape[0] - 1))

        # Calculate the 3D position of the right shoulder
        right_shoulder_distance = depth_image[y_right_shoulder , x_right_shoulder ] * depth_scale
        z_right_shoulder  = right_shoulder_distance
        x_world_right_shoulder  = (x_right_shoulder  - cx) * z_right_shoulder  / fx
        y_world_right_shoulder  = (y_right_shoulder  - cy) * z_right_shoulder  / fy
        rospy.loginfo(f"schulter position in Pcam{x_world_right_shoulder,x_world_right_shoulder, y_world_right_shoulder}")

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

        #Kalman Filter für Schulter, Elbogen und Hand
        
        #shoulder_trans_kf = kf_shoulder.update([x_world_right_shoulder,y_world_right_shoulder,z_right_shoulder])
        #elbow_trans = kf_elbow.update([x_world_right_elbow,y_world_right_elbow,z_right_elbow])
        #hand_trans = kf_hand.update([x_world_right_hand,y_world_right_hand,z_right_hand])
        measurement_shoulder = np.array([[x_world_right_shoulder], [y_world_right_shoulder], [z_right_shoulder]])
        measurement_elbow = np.array([[x_world_right_elbow], [y_world_right_elbow], [z_right_elbow]])
        measurement_hand = np.array([[x_world_right_hand], [y_world_right_hand], [z_right_hand]])
        
        # Vorhersage und Update des Kalman-Filters
        kf_shoulder.predict()
        kf_shoulder.update(measurement_shoulder)
        shoulder_trans = kf_shoulder.x[:3].flatten()
        
        kf_elbow.predict()
        kf_elbow.update(measurement_elbow)
        elbow_trans = kf_elbow.x[:3].flatten()
        
        kf_hand.predict()
        kf_hand.update(measurement_hand)
        hand_trans = kf_hand.x[:3].flatten()
        # shoulder_trans = [x_world_right_shoulder,y_world_right_shoulder,z_right_shoulder]
        # elbow_trans = [x_world_right_elbow,y_world_right_elbow,z_right_elbow]
        # hand_trans = [x_world_right_hand,y_world_right_hand,z_right_hand]
        #if not (sum(shoulder_trans)==0):
        # Create a PointStamped message for the right shoulder in camera frame
        right_shoulder_point = PointStamped()
        right_shoulder_point.header.frame_id = "camera_link"
        right_shoulder_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_shoulder_point.point.x = -shoulder_trans[0]    
        right_shoulder_point.point.y = shoulder_trans[2]
        right_shoulder_point.point.z = shoulder_trans[1]
        #if not all(x == 0 for x in elbow_trans):
            # Create a PointStamped message for the right elbow in camera frame
        right_elbow_point = PointStamped()
        right_elbow_point.header.frame_id = "camera_link"
        right_elbow_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_elbow_point.point.x = -elbow_trans[0]  
        right_elbow_point.point.y = elbow_trans[2]  
        right_elbow_point.point.z = elbow_trans[1]  
        #if not all(x == 0 for x in hand_trans):
            # Create a PointStamped message for the right hand in camera frame
        right_hand_point = PointStamped()
        right_hand_point.header.frame_id = "camera_link"
        right_hand_point.header.stamp = rospy.Time.now()  # Aktueller Zeitstempel
        right_hand_point.point.x = -hand_trans[0]
        right_hand_point.point.y = hand_trans[2]
        right_hand_point.point.z = hand_trans[1]

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
    cv2.imshow("Pose Landmarks", cv2.flip(color_image,-1))
    cv2.waitKey(1)

    rate.sleep()

# Stop the pipeline when done
pipeline.stop()