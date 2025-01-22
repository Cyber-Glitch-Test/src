#! /usr/bin/env python

from tf import TransformBroadcaster
import rospy
from rospy import Time 
import pyrealsense2 as rs
import mediapipe as mp
import cv2
import numpy as np
import datetime as dt

font = cv2.FONT_HERSHEY_SIMPLEX
org = (20, 100)
fontScale = .5
color = (0,50,255)
thickness = 1



#======= ROS ======
rospy.init_node('Stereo_Cam')
    
b = TransformBroadcaster()
    
translation = (0.0, 0.0, 0.0)
rotation = (0.0, 0.0, 0.0, 1.0)
rate = rospy.Rate(5)  # 5hz


# ====== Realsense ======
realsense_ctx = rs.context()
connected_devices = [] # List of serial numbers for present cameras
for i in range(len(realsense_ctx.devices)):
    detected_camera = realsense_ctx.devices[i].get_info(rs.camera_info.serial_number)
    print(f"{detected_camera}")
    connected_devices.append(detected_camera)
device = connected_devices[0] 
pipeline = rs.pipeline()
config = rs.config()
background_removed_color = 153 # Grey

# ====== Mediapipe ======
mpPose = mp.solutions.pose
pose = mpPose.Pose()
mpDraw = mp.solutions.drawing_utils

# ====== Enable Streams ======
config.enable_device(device)

# # For worse FPS, but better resolution:
# stream_res_x = 1280
# stream_res_y = 720
# # For better FPS. but worse resolution:
stream_res_x = 640
stream_res_y = 480

stream_fps = 30

config.enable_stream(rs.stream.depth, stream_res_x, stream_res_y, rs.format.z16, stream_fps)
config.enable_stream(rs.stream.color, stream_res_x, stream_res_y, rs.format.bgr8, stream_fps)
profile = pipeline.start(config)

align_to = rs.stream.color
align = rs.align(align_to)

# ====== Get depth Scale ======
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()
print(f"\tDepth Scale for Camera SN {device} is: {depth_scale}")

# ====== Set clipping distance ======
clipping_distance_in_meters = 4
clipping_distance = clipping_distance_in_meters / depth_scale
print(f"\tConfiguration Successful for SN {device}")

# ====== Get and process images ====== 
print(f"Starting to capture images on SN: {device}")

while True:
    start_time = dt.datetime.today().timestamp()

    # Get and align frames
    frames = pipeline.wait_for_frames()
    aligned_frames = align.process(frames)
    aligned_depth_frame = aligned_frames.get_depth_frame()
    color_frame = aligned_frames.get_color_frame()
    
    if not aligned_depth_frame or not color_frame:
        continue

    # Process images
    depth_image = np.asanyarray(aligned_depth_frame.get_data())
    depth_image_flipped = cv2.flip(depth_image,1)
    color_image = np.asanyarray(color_frame.get_data())

    depth_image_3d = np.dstack((depth_image,depth_image,depth_image)) # Depth image is 1 channel, while color image is 3
    background_removed = np.where((depth_image_3d > clipping_distance) | (depth_image_3d <= 0), background_removed_color, color_image)

    depth_colormap = cv2.applyColorMap(cv2.convertScaleAbs(depth_image, alpha=0.03), cv2.COLORMAP_JET)

    images = cv2.flip(background_removed,1)
    color_image = cv2.flip(color_image,1)
    color_images_rgb = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)

    # Process pose
    results = pose.process(color_images_rgb)
    if results.pose_landmarks:
        # Draw pose landmarks
        mpDraw.draw_landmarks(images, results.pose_landmarks, mpPose.POSE_CONNECTIONS)

        # Get coordinates of the right shoulder (index 12)
        right_shoulder_landmark = results.pose_landmarks.landmark[12]  
        xr = int(right_shoulder_landmark.x * images.shape[1])
        yr = int(right_shoulder_landmark.y * images.shape[0])
        zr = right_shoulder_landmark.z

        left_shoulder_landmark = results.pose_landmarks.landmark[11]  
        xl = int(left_shoulder_landmark.x * images.shape[1])
        yl = int(left_shoulder_landmark.y * images.shape[0])
        zl = left_shoulder_landmark.z  


        
        right_shoulder_distance = depth_image_flipped[yr, xr] * depth_scale

        left_shoulder_distance = depth_image_flipped[yl, xl] * depth_scale  
        

        #Calculate Chest Position and Distance

        x=int((xr+xl)/2)
        y=int((yl+yr)/2)

        chest_marker_pos= (x,y)

        chest_distance = depth_image_flipped[y, x] * depth_scale

        images = cv2.circle(images, chest_marker_pos, radius=5, color=(0,255,0),thickness=1)

        #send ROS mgs
        if not rospy.is_shutdown():
            translation = (x, y, chest_distance)
            b.sendTransform(translation, rotation, Time.now(), 'ignite_robot', '/base_link')

        # Display the distances on the image
        distance_text_right_shoulder = f"Distance to right Shoulder({right_shoulder_distance:0.3} m)"
        distance_text_left_shoulder = f"Distance to left Shoulder({left_shoulder_distance:0.3} m)"
        distance_text = f"Distance to Chest ({chest_distance:0.3} m)"
        images = cv2.putText(images, distance_text_right_shoulder, (20, 100), font, fontScale, color, thickness, cv2.LINE_AA)
        images = cv2.putText(images, distance_text_left_shoulder, (20, 120), font, fontScale, color, thickness, cv2.LINE_AA)
        images = cv2.putText(images, distance_text, (20, 140), font, fontScale, color, thickness, cv2.LINE_AA)
    else:
        images = cv2.putText(images, "No Pose Detected", org, font, fontScale, color, thickness, cv2.LINE_AA)


    # Display FPS
    time_diff = dt.datetime.today().timestamp() - start_time
    fps = int(1 / time_diff)
    org3 = (20, org[1] + 60)
    images = cv2.putText(images, f"FPS: {fps}", org3, font, fontScale, color, thickness, cv2.LINE_AA)

    name_of_window = 'SN: ' + str(device)

    # Display images 
    cv2.namedWindow(name_of_window, cv2.WINDOW_AUTOSIZE)
    cv2.imshow(name_of_window, images)
    key = cv2.waitKey(1)
    # Press esc or 'q' to close the image window
    if key & 0xFF == ord('q') or key == 27:
        print(f"User pressed break key for SN: {device}")
        break

print(f"Application Closing")
pipeline.stop()
print(f"Application Closed.")
