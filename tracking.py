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



rospy.init_node('Stereo_Cam')

broadcaster = tf.TransformBroadcaster()
translation = (0.0894, -0.28, 0.80)   # Position der Kamera im Weltkoordinatensystem X/Y/Z
rotation = quaternion_from_euler(((-22/180)*math.pi),((0/180)*math.pi),((0/180)*math.pi)) # Orientierung der Kamera im Weltkoordinatensystem Roll/Pitch/Yaw
#rotation = quaternion_from_euler(-math.pi, ((17*math.pi)/180), math.pi) 

def calc_midPoint(x1,x2,y1,y2,z1,z2):  
  return ((x1 + x2)/2, (y1 + y2)/2,(z1+z2)/2)


# Initialisierte Realsense Kamera
realsense_ctx = rs.context()
connected_devices = [] 
for i in range(len(realsense_ctx.devices)):
    detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    print(f"{detected_camera}")
    connected_devices.append(detected_camera)
device = connected_devices[0]
pipeline = rs.pipeline()
config = rs.config()


mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils


config.enable_device(device)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Camera Intrinsics
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy = intrinsics.fx, intrinsics.fy  # Focal lengths
cx, cy = intrinsics.ppx, intrinsics.ppy  # Principal point

# Start the ROS loop
rate = rospy.Rate(30) 
listener = tf.TransformListener()  

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

        #links

        left_shoulder_landmark = results.pose_landmarks.landmark[11]
        x_left_shoulder = int(left_shoulder_landmark.x * color_image.shape[1])
        y_left_shoulder = int(left_shoulder_landmark.y * color_image.shape[0])

        x_left_shoulder = max(0, min(x_left_shoulder, color_image.shape[1] - 1))
        y_left_shoulder = max(0, min(y_left_shoulder, color_image.shape[0] - 1))

        left_elbow_landmark = results.pose_landmarks.landmark[13]
        x_left_elbow = int(left_elbow_landmark.x * color_image.shape[1])
        y_left_elbow = int(left_elbow_landmark.y * color_image.shape[0])


        x_left_elbow = max(0, min(x_left_elbow, color_image.shape[1] - 1))
        y_left_elbow = max(0, min(y_left_elbow, color_image.shape[0] - 1))


        left_hand_landmark = results.pose_landmarks.landmark[15]
        x_left_hand = int(left_hand_landmark.x * color_image.shape[1])
        y_left_hand = int(left_hand_landmark.y * color_image.shape[0])


        x_left_hand = max(0, min(x_left_hand, color_image.shape[1] - 1))
        y_left_hand = max(0, min(y_left_hand, color_image.shape[0] - 1))

        #rechts

        right_shoulder_landmark = results.pose_landmarks.landmark[12]
        x_right_shoulder = int(right_shoulder_landmark.x * color_image.shape[1])
        y_right_shoulder = int(right_shoulder_landmark.y * color_image.shape[0])

        x_right_shoulder = max(0, min(x_right_shoulder, color_image.shape[1] - 1))
        y_right_shoulder = max(0, min(y_right_shoulder, color_image.shape[0] - 1))

        right_elbow_landmark = results.pose_landmarks.landmark[14]
        x_right_elbow = int(right_elbow_landmark.x * color_image.shape[1])
        y_right_elbow = int(right_elbow_landmark.y * color_image.shape[0])


        x_right_elbow = max(0, min(x_right_elbow, color_image.shape[1] - 1))
        y_right_elbow = max(0, min(y_right_elbow, color_image.shape[0] - 1))


        right_hand_landmark = results.pose_landmarks.landmark[16]
        x_right_hand = int(right_hand_landmark.x * color_image.shape[1])
        y_right_hand = int(right_hand_landmark.y * color_image.shape[0])


        x_right_hand = max(0, min(x_right_hand, color_image.shape[1] - 1))
        y_right_hand = max(0, min(y_right_hand, color_image.shape[0] - 1))

        #Brechne die Poisition von  Schulter, Elbogen und Hand mittels Intrinsics der Kamera
        # Linke Schulter
        left_shoulder_distance = depth_image[y_left_shoulder, x_left_shoulder] * depth_scale
        z_left_shoulder = left_shoulder_distance
        x_world_left_shoulder = (x_left_shoulder - cx) * z_left_shoulder / fx
        x_world_left_shoulder = 0.78  * x_world_left_shoulder
        y_world_left_shoulder = (y_left_shoulder - cy) * z_left_shoulder / fy
        

        # Linker Ellbogen
        left_elbow_distance = depth_image[y_left_elbow, x_left_elbow] * depth_scale
        z_left_elbow = left_elbow_distance
        x_world_left_elbow = (x_left_elbow - cx) * z_left_elbow / fx
        x_world_left_elbow = 1.25 * x_world_left_elbow
        y_world_left_elbow = (y_left_elbow - cy) * z_left_elbow / fy

        # Linke Hand
        left_hand_distance = depth_image[y_left_hand, x_left_hand] * depth_scale
        z_left_hand = left_hand_distance
        x_world_left_hand = (x_left_hand - cx) * z_left_hand / fx
        x_world_left_hand = 2.41 * x_world_left_hand
        y_world_left_hand = (y_left_hand - cy) * z_left_hand / fy

        # Rechte Schulter
        right_shoulder_distance = depth_image[y_right_shoulder, x_right_shoulder] * depth_scale
        z_right_shoulder = right_shoulder_distance
        x_world_right_shoulder = (x_right_shoulder - cx) * z_right_shoulder / fx
        x_world_right_shoulder = 0.78* x_world_right_shoulder
        y_world_right_shoulder = (y_right_shoulder - cy) * z_right_shoulder / fy

        # Rechter Ellbogen
        right_elbow_distance = depth_image[y_right_elbow, x_right_elbow] * depth_scale
        z_right_elbow = right_elbow_distance
        x_world_right_elbow = (x_right_elbow - cx) * z_right_elbow / fx
        x_world_right_elbow = 2.41 * x_world_right_elbow
        y_world_right_elbow = (y_right_elbow - cy) * z_right_elbow / fy

        # Rechte Hand
        right_hand_distance = depth_image[y_right_hand, x_right_hand] * depth_scale
        z_right_hand = right_hand_distance
        x_world_right_hand = (x_right_hand - cx) * z_right_hand / fx
        x_world_right_hand = 2.41 * x_world_right_hand
        y_world_right_hand = (y_right_hand - cy) * z_right_hand / fy

        #Kalman Filter für Schulter, Elbogen und Hand
        
        #shoulder_trans_kf = kf_shoulder.update([x_world_right_shoulder,y_world_right_shoulder,z_right_shoulder])
        #elbow_trans = kf_elbow.update([x_world_right_elbow,y_world_right_elbow,z_right_elbow])
        #hand_trans = kf_hand.update([x_world_right_hand,y_world_right_hand,z_right_hand])
        left_shoulder_trans =   [x_world_left_shoulder,y_world_left_shoulder,z_left_shoulder]
        left_elbow_trans =      [x_world_left_elbow,y_world_left_elbow,z_left_elbow]
        left_hand_trans =       [x_world_left_hand,y_world_left_hand,z_left_hand]

        right_shoulder_trans =  [x_world_right_shoulder,y_world_right_shoulder,z_right_shoulder]
        right_elbow_trans =     [x_world_right_elbow,y_world_right_elbow,z_right_elbow]
        right_hand_trans =      [x_world_right_hand,y_world_right_hand,z_right_hand]

        shoulder_trans =    calc_midPoint(x_world_right_shoulder,x_world_left_shoulder,y_world_right_shoulder,y_world_left_shoulder,z_right_shoulder,z_left_shoulder)
        elbow_trans =       calc_midPoint(x_world_right_elbow,x_world_left_elbow,y_world_right_elbow,y_world_left_elbow,z_right_elbow,z_left_elbow)
        hand_trans =        calc_midPoint(x_world_right_hand,x_world_left_hand,y_world_right_hand,y_world_left_hand,z_right_hand,z_left_hand)
        rospy.logwarn(f"mittelpunkt: {shoulder_trans[0]}")
        #Erstelle punkte für den Publisher
        shoulder_point = PointStamped()
        shoulder_point.header.frame_id = "camera_link"
        shoulder_point.header.stamp = rospy.Time.now()  
        shoulder_point.point.x = -shoulder_trans[0]    
        shoulder_point.point.y = shoulder_trans[2]
        shoulder_point.point.z = shoulder_trans[1]
        #rospy.logwarn(f"Schulter: {-shoulder_trans[0],shoulder_trans[2],shoulder_trans[1] }")

        elbow_point = PointStamped()
        elbow_point.header.frame_id = "camera_link"
        elbow_point.header.stamp = rospy.Time.now()  
        elbow_point.point.x = -elbow_trans[0]  
        elbow_point.point.y = elbow_trans[2]  
        elbow_point.point.z = elbow_trans[1]  

        hand_point = PointStamped()
        hand_point.header.frame_id = "camera_link"
        hand_point.header.stamp = rospy.Time.now() 
        hand_point.point.x = -hand_trans[0]
        hand_point.point.y = hand_trans[2]
        hand_point.point.z = hand_trans[1]

        left_shoulder_point = PointStamped()
        left_shoulder_point.header.frame_id = "camera_link"
        left_shoulder_point.header.stamp = rospy.Time.now()  
        left_shoulder_point.point.x = -left_shoulder_trans[0]    
        left_shoulder_point.point.y = left_shoulder_trans[2]
        left_shoulder_point.point.z = left_shoulder_trans[1]


        left_elbow_point = PointStamped()
        left_elbow_point.header.frame_id = "camera_link"
        left_elbow_point.header.stamp = rospy.Time.now()  
        left_elbow_point.point.x = -left_elbow_trans[0]  
        left_elbow_point.point.y = left_elbow_trans[2]  
        left_elbow_point.point.z = left_elbow_trans[1]  

        left_hand_point = PointStamped()
        left_hand_point.header.frame_id = "camera_link"
        left_hand_point.header.stamp = rospy.Time.now() 
        left_hand_point.point.x = -left_hand_trans[0]
        left_hand_point.point.y = left_hand_trans[2]
        left_hand_point.point.z = left_hand_trans[1]

        right_shoulder_point = PointStamped()
        right_shoulder_point.header.frame_id = "camera_link"
        right_shoulder_point.header.stamp = rospy.Time.now()  
        right_shoulder_point.point.x = -right_shoulder_trans[0]    
        right_shoulder_point.point.y = right_shoulder_trans[2]
        right_shoulder_point.point.z = right_shoulder_trans[1]


        right_elbow_point = PointStamped()
        right_elbow_point.header.frame_id = "camera_link"
        right_elbow_point.header.stamp = rospy.Time.now()  
        right_elbow_point.point.x = -right_elbow_trans[0]  
        right_elbow_point.point.y = right_elbow_trans[2]  
        right_elbow_point.point.z = right_elbow_trans[1]  

        right_hand_point = PointStamped()
        right_hand_point.header.frame_id = "camera_link"
        right_hand_point.header.stamp = rospy.Time.now() 
        right_hand_point.point.x = -right_hand_trans[0]
        right_hand_point.point.y = right_hand_trans[2]
        right_hand_point.point.z = right_hand_trans[1]

        # Veröffentliche den Frame "camera_link" im TF-Baum
        broadcaster.sendTransform(
            translation,  
            rotation,    
            rospy.Time.now(), 
            "camera_link",  # Child Frame (Kamera)
            "base"         # Parent Frame (Weltkoordinatensystem)
        )

        # Warte auf die TF-Daten
        try:
            # Warte auf die Transformation
            listener.waitForTransform("base", "camera_link", shoulder_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", shoulder_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0), 
                rospy.Time.now(),
                "shoulder",
                "base"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("base", "camera_link", elbow_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", elbow_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  
                rospy.Time.now(),
                "elbow",
                "base"
            )

            listener.waitForTransform("base", "camera_link", hand_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", hand_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0), 
                rospy.Time.now(),
                "hand",
                "base"
            )

            # Warte auf die Transformation
            listener.waitForTransform("base", "camera_link", right_shoulder_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", right_shoulder_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0), 
                rospy.Time.now(),
                "right_shoulder",
                "base"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("base", "camera_link", right_elbow_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", right_elbow_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  
                rospy.Time.now(),
                "right_elbow",
                "base"
            )

            listener.waitForTransform("base", "camera_link", right_hand_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", right_hand_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0), 
                rospy.Time.now(),
                "right_hand",
                "base"
            )

            # Warte auf die Transformation
            listener.waitForTransform("base", "camera_link", left_shoulder_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", left_shoulder_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0), 
                rospy.Time.now(),
                "left_shoulder",
                "base"
            )

            # Warte auf die Transformation mit dem richtigen Zeitstempel
            listener.waitForTransform("base", "camera_link", left_elbow_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", left_elbow_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0),  
                rospy.Time.now(),
                "left_elbow",
                "base"
            )

            listener.waitForTransform("base", "camera_link", left_hand_point.header.stamp, rospy.Duration(1.0))
            transformed_point = listener.transformPoint("base", left_hand_point)

            broadcaster.sendTransform(
                (transformed_point.point.x, transformed_point.point.y, transformed_point.point.z),
                (0.0, 0.0, 0.0, 1.0), 
                rospy.Time.now(),
                "left_hand",
                "base"
            )

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"Error transforming point: {e}")

    cv2.imshow("Pose Landmarks", cv2.flip(color_image,-1))
    cv2.waitKey(1)

    rate.sleep()

pipeline.stop()