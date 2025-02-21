#!/usr/bin/env python

import numpy as np
import csv
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from moveit_msgs.msg import Grasp, PlaceLocation
from moveit_commander.move_group import MoveGroupCommander
import smach

#================== x    y    z    w    Grp   2 Hum
Poses = np.array([[-0.28531283917512756, 0.08176575019716574,0.3565888897535509,0.021838185570339213,-0.9997536365149914,0.0006507883874787611,0.003916171666392069],
                    [-0.2689336536616916,-0.2929305205611311,0.4467808596850957,-0.8026641190873571,-0.5739865856274126,-0.07377273857864448,0.1443166466216185],
                    [-0.2883698817917215,-0.2651904843469419,0.44669702333462596,-0.8026255278246236,-0.5739945533651698,-0.0740857787125935,0.14433922607493854]])#40: Positionier the base plate 1

height_human_shoulder = 1.8

#======Robot Control======
-0.2684972317429937
def move_to_target(move_group, target_pose):
    # Setze die Zielpose
    move_group.set_pose_target(target_pose)

    # Plane und führe die Bewegung aus
    success = move_group.go(wait=True)
    if success:
        rospy.loginfo("Bewegung zum Ziel erfolgreich!")
    else:
        rospy.logwarn("Fehler bei der Bewegung zum Ziel!")


def stop_robot(move_group):
    # Stoppe die Bewegung des Roboters
    move_group.stop()
    rospy.loginfo("Roboter gestoppt!")


def reset_robot(move_group):
    # Setze den Roboter auf die Home-Position
    move_group.set_named_target("home")
    move_group.go(wait=True)
    rospy.loginfo("Roboter auf 'Home' Position zurückgesetzt!")


def moveit_control_node():

    # Initialisiere MoveIt und ROS
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('ur5_moveit_control', anonymous=True)

    # Erstelle den MoveGroupCommander für den UR5 Roboter
    group_name = "manipulator" 
    move_group = MoveGroupCommander(group_name)

    # Setze die maximale Geschwindigkeit und Beschleunigung
    move_group.set_max_velocity_scaling_factor(0.1)  # Geschwindigkeit 10% der maximalen Geschwindigkeit
    move_group.set_max_acceleration_scaling_factor(0.1)  # Beschleunigung 10% der maximalen Beschleunigung

    # Referenzrahmen
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo("Planungsrahmen: %s", planning_frame)

    # Zielrahmen für den Endeffektor
    eef_link = move_group.get_end_effector_link()
    rospy.loginfo("Endeffektor-Link: %s", eef_link)

    # Beispiel für eine Zielpose für den Endeffektor (XYZ-Position und Orientierung)
    target_pose = Pose()
    for i,j in enumerate(Poses):

        print(i)
        target_pose.position.x =    Poses[i][0]
        target_pose.position.y =    Poses[i][1]
        target_pose.position.z =    Poses[i][2]
        target_pose.orientation.x = Poses[i][3]
        target_pose.orientation.y = Poses[i][4]
        target_pose.orientation.z = Poses[i][5]
        target_pose.orientation.w = Poses[i][6]

        move_to_target(move_group, target_pose)


    # Bewegung des Roboters zur Zielpose
    rospy.loginfo("Bewege den Roboter zur Zielposition...")
    move_to_target(move_group, target_pose)

    # Stoppe den Roboter (falls erforderlich)
    stop_robot(move_group)

    # Shutdown von MoveIt und ROS-Verbindungen
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        # Starte die ROS-Node
        moveit_control_node()
    except rospy.ROSInterruptException:
        pass
