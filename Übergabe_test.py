#!/usr/bin/env python

import numpy as np
import csv
import rospy
import moveit_commander
import sys
import smach
import smach_ros
import rtde_control
import tf
import math
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PointStamped
from std_msgs.msg import String
from moveit_msgs.msg import Grasp, PlaceLocation
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from geometry_msgs.msg import PoseStamped
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg


#======Posen für Roboterarm====== 
rb_arm_home             = np.array([-0.28531283917512756, 0.08176575019716574, 0.3565888897535509,0.021838185570339213,-0.9997536365149914,0.0006507883874787611,0.003916171666392069])
rb_arm_over_m1          = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335,-0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m1            = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288,-0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
rb_arm_on_hum_static    = np.array([-0.2872170720236103, -0.27175826228875855, 0.38259507410129007,0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398])
rb_arm_over_m2          = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288,-0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m2            = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335 ,-0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
rb_arm_over_m3          = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288,-0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m3            = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335 ,-0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
rb_arm_over_m4          = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288,-0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m4            = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335 ,-0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])


forearmlenghdin = 0.3335 #aus DIN 33402-2 gemittel aus Mann und Frau über alle altersklassen
upperarmlenghtdin = 0.342 #aus DIN 33402-2 gemittel aus Mann und Frau über alle altersklassen
Hum_det = True
savety_koord_1 = np.array([0.2, 0.68, 0.8])
savety_koord_2 = np.array([-0.2, 0.5, 0.3])
tcp_coversion=0.20



class RobotControl:
    def __init__(self, group_name):
        self.group_name = group_name
        self.move_group = MoveGroupCommander(self.group_name)
        #self.gripper_controller = GripperController()
        self.scene = PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        # Setze die maximale Geschwindigkeit und Beschleunigung
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

    def convert_to_pose(self, koords):
        target_pose = Pose()
        target_pose.position.x = koords[0]
        target_pose.position.y = koords[1]
        target_pose.position.z = koords[2]
        target_pose.orientation.x = koords[3]
        target_pose.orientation.y = koords[4]
        target_pose.orientation.z = koords[5]
        target_pose.orientation.w = koords[6]
        # broadcaster = tf.TransformBroadcaster()
        # broadcaster.sendTransform(
        #     (target_pose.position.x,target_pose.position.y,target_pose.position.z),  # Position der Kamera im Weltkoordinatensystem
        #     (target_pose.orientation.x,target_pose.orientation.y,target_pose.orientation.z,target_pose.orientation.w),     # Orientierung der Kamera im Weltkoordinatensystem
        #     rospy.Time.now(),  # Zeitstempel
        #     "übergabepunkt",  # Child Frame (Kamera)
        #     "world"         # Parent Frame (Weltkoordinatensystem)
        # )
        return target_pose

    def move_to_target(self, target_pose, speed):
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)
        self.move_group.set_pose_target(target_pose)
        rospy.loginfo("Bewege Roboter zu: x={}, y={}, z={}".format(target_pose.position.x, target_pose.position.y, target_pose.position.z))

        success = self.move_group.go(wait=True)
        if success:
            rospy.loginfo("Bewegung erfolgreich!")
            return True
        else:
            rospy.logwarn("Bewegung fehlgeschlagen!")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return False
        
    def move_to_target_carth(self, target_pose, speed):
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)
        waypoints = []
        waypoints.append(self.move_group.get_current_pose().pose)
        waypoints.append(target_pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(
                waypoints, 0.01  # waypoints to follow  # eef_step
                    )
        rospy.loginfo("Bewege Roboter in einer Linie zu: x={}, y={}, z={}".format(target_pose.position.x, target_pose.position.y, target_pose.position.z))

        success = self.move_group.execute(plan, wait=True)
        if success:
            rospy.loginfo("Bewegung erfolgreich!")
            return True
        else:
            rospy.logwarn("Bewegung fehlgeschlagen!")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return False

    def move_to_joint_goal(self, joint_goal, speed):
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)
        success = self.move_group.go(joint_goal, wait=True)
        if success:
            rospy.loginfo("Bewegung erfolgreich!")
            return True
        else:
            rospy.logwarn("Bewegung fehlgeschlagen!")
            self.move_group.stop()
            self.move_group.clear_pose_targets()
            return False

    def stop_robot(self):
        self.move_group.stop()
        rospy.loginfo("Roboter gestoppt!")

    def reset_robot(self):
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)
        rospy.loginfo("Roboter auf 'Home' Position zurückgesetzt!")

    def calc_handover_position(self):
        try:
            hand_over_position = Pose()
            hm = get_Hum_mertics()

            rospy.loginfo("Schulter, Ellbogen und Hand erkannt")
            rospy.loginfo("Unterarmlänge:")
            rospy.loginfo(hm.forearmlenght)
            rospy.logwarn("x: %s", hm.shoulderkoords[0])
            rospy.logwarn("y: %s", hm.shoulderkoords[1])
            rospy.logwarn("z: %s", hm.shoulderkoords[2])
            rospy.loginfo("Oberarmlänge:")
            rospy.loginfo(hm.uperarmlenght)



            rospy.loginfo("Schulter erkannt")
            hand_over_position_x = -hm.shoulderkoords[0]
            hand_over_position_y = -(hm.shoulderkoords[1] - (hm.forearmlenght + tcp_coversion))
            hand_over_position_z = hm.shoulderkoords[2] - hm.uperarmlenght
            hand_over_position = self.convert_to_pose(np.array([hand_over_position_x, hand_over_position_y, hand_over_position_z,0.4940377021038103, 0.5228192716826835, -0.483399996859536, 0.4989100130217637]))
           
            hand_over_position = self.convert_to_pose(np.array([hand_over_position_x, hand_over_position_y, hand_over_position_z, 0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398]))


            rospy.logwarn("xUe: %s", hand_over_position_x) 
            rospy.logwarn("yUe: %s", hand_over_position_y)
            rospy.logwarn("zUe: %s", hand_over_position_z)

            broadcaster = tf.TransformBroadcaster()
            broadcaster.sendTransform(
                (hand_over_position_x,hand_over_position_y,hand_over_position_z),  # Position der Kamera im Weltkoordinatensystem
                (0, 0 ,0, 1),     # Orientierung der Kamera im Weltkoordinatensystem
                rospy.Time.now(),  # Zeitstempel
                "übergabepunkt",  # Child Frame (Kamera)
                "base"         # Parent Frame (Weltkoordinatensystem)
            )
            # broadcaster.sendTransform(
            #     (hm.shoulderkoords[0],hm.shoulderkoords[1],hm.shoulderkoords[2]),  # Position der Kamera im Weltkoordinatensystem
            #     (0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398),     # Orientierung der Kamera im Weltkoordinatensystem
            #     rospy.Time.now(),  # Zeitstempel
            #     "Schulter_KF",  # Child Frame (Kamera)
            #     "world"         # Parent Frame (Weltkoordinatensystem)
            # )
            
            
            return hand_over_position
        except Exception as e:
            rospy.logwarn("HD fehlgeschlagen. Fehler: %s", e)
            return self.convert_to_pose(rb_arm_on_hum_static)

    def point_inside(self, pose):
        point = [pose.position.x, pose.position.y, pose.position.z]
        xmin, xmax = savety_koord_1[0] - 1, savety_koord_2[0] + 1
        ymin, ymax = savety_koord_1[1] - 1, savety_koord_2[1] + 1
        zmin, zmax = savety_koord_1[2] - 1, savety_koord_2[2] + 1
        return xmin < point[0] < xmax and ymin < point[1] < ymax and zmin < point[2] < zmax
    
class get_Hum_mertics:
    def __init__(self):
            self.uperarmlenght = 0
            self.forearmlenght = 0
            self.shoulderkoords = [0, 0, 0]
            self.elbowkoords = [0, 0, 0]
            self.handkoords = [0, 0, 0]
            self.calc_arm_lenght()

    def camera_listener(self):
            try:
                time = rospy.Time(0)
                listener = tf.TransformListener()
                listener.waitForTransform("world", "right_shoulder", time, rospy.Duration(1.0))
                listener.waitForTransform("world", "right_elbow", time, rospy.Duration(1.0))
                listener.waitForTransform("world", "right_hand", time, rospy.Duration(1.0))

                shoulder_trans, _ = listener.lookupTransform("world", "right_shoulder", time)
                elbow_trans, _ = listener.lookupTransform("world", "right_elbow", time)
                hand_trans, _ = listener.lookupTransform("world", "right_hand", time)
                
                

                self.shoulderkoords = [shoulder_trans[0], shoulder_trans[1], shoulder_trans[2]]
                self.elbowkoords = [elbow_trans[0], elbow_trans[1], elbow_trans[2]]
                self.handkoords = [hand_trans[0], hand_trans[1], hand_trans[2]]

            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                rospy.logwarn(f"TF Error: {e}")

    def calc_arm_lenght(self):
            self.camera_listener()
            self.uperarmlenght = self.calc_euclidean_distance(self.shoulderkoords, self.elbowkoords)
            self.forearmlenght = self.calc_euclidean_distance(self.handkoords, self.elbowkoords)

    def calc_euclidean_distance(self, point1, point2):
            distance = 0.0
            for i in range(len(point1)):
                distance += (point2[i] - point1[i]) ** 2
            return math.sqrt(distance)

robot_control = RobotControl("manipulator")

if __name__ == "__main__":

    rospy.init_node('ur5_moveit_control', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)
    while not rospy.is_shutdown():
        robot_control.calc_handover_position()
    rospy.spin()