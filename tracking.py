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
#from filterpy.kalman import KalmanFilter  # type: ignore
from std_srvs.srv import Empty  # type: ignore



rospy.init_node('Stereo_Cam')

broadcaster = tf.TransformBroadcaster()
translation = (0.0894, -0.28, 0.80)   # Position der Kamera im Weltkoordinatensystem X/Y/Z
rotation = quaternion_from_euler(((-22/180)*math.pi),((0/180)*math.pi),((0/180)*math.pi)) # Orientierung der Kamera im Weltkoordinatensystem Roll/Pitch/Yaw
#rotation = quaternion_from_euler(-math.pi, ((17*math.pi)/180), math.pi) 

def calc_midPoint(x1,x2,y1,y2,z1,z2):  
  return ((x1 + x2)/2, (y1 + y2)/2,(z1+z2)/2)

# def get_box_corners(p1, p2):
#     x_vals = [p1[0], p2[0]]
#     y_vals = [p1[1], p2[1]]
#     z_vals = [p1[2], p2[2]]

#     corners = []
#     for x in x_vals:
#         for y in y_vals:
#             for z in z_vals:
#                 corners.append(np.array([x, y, z]))
#     return corners

# def world_to_camera(pt, trans, rot_matrix):
#     # Invertierte Rotation & Translation
#     R_inv = np.linalg.inv(rot_matrix)
#     t = np.array(trans).reshape(3, 1)
#     cam_coords = R_inv @ (pt.reshape(3, 1) - t)
#     return cam_coords.flatten()

# def camera_to_pixel(pt_cam, fx, fy, cx, cy):
#     x, y, z = pt_cam
#     if z <= 0: return None  # hinter der Kamera
#     u = int((x / z) * fx + cx)
#     v = int((y / z) * fy + cy)
#     return (u, v)

# def draw_box_on_image(image, pixel_pts):
#     box_lines = [
#         (0, 1), (0, 2), (0, 4),
#         (1, 3), (1, 5),
#         (2, 3), (2, 6),
#         (3, 7),
#         (4, 5), (4, 6),
#         (5, 7),
#         (6, 7)
#     ]
#     for i, j in box_lines:
#         if pixel_pts[i] and pixel_pts[j]:
#             cv2.line(image, pixel_pts[i], pixel_pts[j], (0, 255, 0), 2)

# Initialisierte Realsense Kamera
target_serial = "831612072790"
realsense_ctx = rs.context()
connected_devices = [] 
for i in range(len(realsense_ctx.devices)):
    detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    print(f"Serial nummer: {detected_camera}")
    connected_devices.append(detected_camera)


if target_serial not in connected_devices:
    raise RuntimeError(f"Kamera mit Seriennummer {target_serial} nicht gefunden.")


#device = connected_devices[0]
pipeline = rs.pipeline()
config = rs.config()


mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils


config.enable_device(target_serial)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Kamera Intrinsics
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy = intrinsics.fx, intrinsics.fy  # Focal lengths
cx, cy = intrinsics.ppx, intrinsics.ppy  # Principal point

# Starte ROS übertragung
rate = rospy.Rate(30) 
listener = tf.TransformListener()  

while not rospy.is_shutdown():
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()

    if not aligned_depth_frame or not color_frame:
        continue


    # Konvertiere in Arraays
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    #depth_image = cv2.flip(depth_image,-1)
    color_image = np.asanyarray(color_frame.get_data())
    #color_image = cv2.flip(color_image,-1)

    # # Sicherheitsecke in Welt-Koordinaten
    # safety_koord_1 = np.array([0.20, 0.6, 0.0])
    # safety_koord_2 = np.array([0.24, 0.04,0.7])


    # rot_matrix = tf.transformations.quaternion_matrix(rotation)[:3, :3]

    # box_corners_world = get_box_corners(safety_koord_1, safety_koord_2)


    # box_pixel_points = []
    # for corner in box_corners_world:
    #     corner_cam = world_to_camera(corner, translation, rot_matrix)
    #     pixel = camera_to_pixel(corner_cam, fx, fy, cx, cy)
    #     box_pixel_points.append(pixel)


    # draw_box_on_image(color_image, box_pixel_points)

    # Pose und Mediapipe iniitieren
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
        x_world_left_elbow = 0.78 * x_world_left_elbow
        y_world_left_elbow = (y_left_elbow - cy) * z_left_elbow / fy

        # Linke Hand
        left_hand_distance = depth_image[y_left_hand, x_left_hand] * depth_scale
        z_left_hand = left_hand_distance
        x_world_left_hand = (x_left_hand - cx) * z_left_hand / fx
        x_world_left_hand = 0.78* x_world_left_hand
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
        x_world_right_elbow = 0.78 * x_world_right_elbow
        y_world_right_elbow = (y_right_elbow - cy) * z_right_elbow / fy

        # Rechte Hand
        right_hand_distance = depth_image[y_right_hand, x_right_hand] * depth_scale
        z_right_hand = right_hand_distance
        x_world_right_hand = (x_right_hand - cx) * z_right_hand / fx
        x_world_right_hand = 0.78 * x_world_right_hand
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

        # Veröffentliche den Frame "camera_link" in TF
        broadcaster.sendTransform(
            translation,  
            rotation,    
            rospy.Time.now(), 
            "camera_link",  # Child Frame
            "base"         # Parent Frame 
        )

        # Warten auf TF
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

            # Warte auf die Transformation mit Zeitstempel
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

            # Warte auf die Transformation mit Zeitstempel
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

            # Warte auf die Transformation mit Zeitstempel
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

