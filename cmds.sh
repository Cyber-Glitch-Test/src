

#############################################################

source /home/ca/catkin_ws/devel/setup.bash

roslaunch schledluer Smach_UR.launch use_calibration:=true


#Robot Control

source /opt/ros/noetic/setup.bash

#ohne cali:

#roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100 

#mit cali

roslaunch ur_robot_driver ur5_bringup.launch limited:=true robot_ip:=192.168.0.100  kinematics_config:=${HOME}/my_robot_calibration.yaml

#roslaunch ur_robot_driver ur5e_bringup.launch robot_ip:=192.168.11.11 kinematics_config:=${HOME}/ur5e_calibration.yaml

######################

source /opt/ros/noetic/setup.bash

roslaunch ur5_moveit_config moveit_planning_execution.launch

######################

source /home/ca/catkin_ws/devel/setup.bash

roslaunch ur5_moveit_config moveit_rviz.launch

#############################################################

#Demo launch

source /home/ca/catkin_ws/devel/setup.bash

roslaunch ur5_moveit_config demo.launch 

#############################################################

#Intel Realsense Launch

roslaunch realsense2_camera demo_pointcloud.launch

roslaunch rgbd_launch rs_rgbd.launch 

roslaunch realsense2_camera rs_aligned_depth.launch

#############################################################

#Python UR5 Control Launch

source /home/ca/catkin_ws/devel/setup.bash

rosrun schledluer schledluer_ros.py

#Robot Control

source /home/ca/catkin_ws/devel/setup.bash

chmod +x /home/ca/catkin_ws/src/schledluer/src/smach_ros_moveit.py

rosrun schledluer smach_ros_moveit.py

#Intel Realsense Control

source /home/ca/catkin_ws/devel/setup.bash

chmod +x /home/ca/catkin_ws/src/schledluer/src/Shoulder_Tracking_ros.py

rosrun schledluer Shoulder_Tracking_ros.py

#Get koords

source /home/ca/catkin_ws/devel/setup.bash

chmod +x /home/ca/catkin_ws/src/schledluer/src/schledluer_ros_with_karth.py

rosrun schledluer schledluer_ros_with_karth.py

#Übergabe Debug

source /home/ca/catkin_ws/devel/setup.bash

chmod +x /home/ca/catkin_ws/src/schledluer/src/Übergabe_test.py

rosrun schledluer Übergabe_test.py

#Tracking Debug

source /home/ca/catkin_ws/devel/setup.bash

chmod +x /home/ca/catkin_ws/src/schledluer/src/shoulder_tracking_ros_sim.py

rosrun schledluer shoulder_tracking_ros_sim.py

rosrun schledluer Shoulder_Tracking_ros.py

#############################################################

#Debug schledluer

chmod +x /home/ca/catkin_ws/src/schledluer/src/schledluer_ros.py

chmod +x /home/ca/catkin_ws/src/schledluer/src/schledluer_ros_with_karth.py

chmod +x /home/ca/catkin_ws/src/schledluer/src/Shoulder_Tracking_ros.py

chmod +x /home/ca/catkin_ws/src/schledluer/src/smach_ros_moveit.py

#############################################################

#Show ROS Node Graph

rqt_graph

#############################################################

#Start Robotiq node

source /home/ca/catkin_ws/devel/setup.bash

rosrun robotiq_2f_gripper_control Robotiq2FGripperRtuNode.py /dev/ttyUSB0

#Controll Node 

source ~/catkin_ws/devel/setup.bash

rosrun robotiq_2f_gripper_control Robotiq2FGripperSimpleController.py

realsense-viewer

