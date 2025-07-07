#!/usr/bin/env python

from datetime import datetime
import numpy as np
import csv
import rospy
import moveit_commander
import sys
from geometry_msgs.msg import Pose
from std_msgs.msg import String
from moveit_msgs.msg import Grasp, PlaceLocation
from moveit_commander.move_group import MoveGroupCommander

#================== x    y    z    w    Grp   2 Hum
Poses = np.array([[ 0.204882033909, 0.257205882746, 0.302208729810, True, False]
                  ])#40: Positionier the base plate 1

waypoints = []


height_human_shoulder = 1.8

#======Robot Control======

# def move_to_target(move_group, target_pose):
#     # Setze die Zielpose
#     move_group.set_pose_target(target_pose)

#     # Plane und führe die Bewegung aus
#     success = move_group.go(wait=True)
#     if success:
#         rospy.loginfo("Bewegung zum Ziel erfolgreich!")
#     else:
#         rospy.logwarn("Fehler bei der Bewegung zum Ziel!")


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
    move_group.set_max_velocity_scaling_factor(0.1)         # Geschwindigkeit 10% der maximalen Geschwindigkeit
    move_group.set_max_acceleration_scaling_factor(0.1)     # Beschleunigung 10% der maximalen Beschleunigung

    # Referenzrahmen
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo("Planungsrahmen: %s", planning_frame)

    # Zielrahmen für den Endeffektor
    eef_link = move_group.get_end_effector_link()
    rospy.loginfo("Endeffektor-Link: %s", eef_link)

    # Beispiel für eine Zielpose für den Endeffektor (XYZ-Position und Orientierung)


    while True:
        newuser = input('enter y/n: ')
        if newuser == "y":
            rospy.loginfo("Roboter Pose...")
            waypoints.append(move_group.get_current_pose().pose)
            print(waypoints)

            last_waypoint = waypoints[-1]    
            position = last_waypoint.position
            orientation = last_waypoint.orientation
            coordinates = f"np.array([{position.x}, {position.y}, {position.z}"
            orientation_values = f",{orientation.x}, {orientation.y}, {orientation.z}, {orientation.w}])"

            joint_goal = move_group.get_current_joint_values()
            joint_values = ", ".join([f"{val:.4f}" for val in joint_goal])  # Formatierte Ausgabe der Gelenkwerte
            
            print(coordinates + orientation_values)
            print(f"Joint Values: {joint_values}")  # Ausgabe der Gelenk-Werte
            continue
        
        elif newuser == "n":
            print("Exiting")
            print(waypoints)
            break


    # now = datetime.now()
    # dt_string = now.strftime("%d%m%Y%H%M%S")    
    # print(dt_string)    
    # with open(dt_string, 'w') as f:
    #     write = csv.writer(f)
    #     write.writerow(waypoints)

    # Bewegung des Roboters zur Zielpose
    #rospy.loginfo("Bewege den Roboter zur Zielposition...")
    #move_to_target(move_group, target_pose)

    # Stoppe den Roboter (falls erforderlich)
    # stop_robot(move_group)

    # Shutdown von MoveIt und ROS-Verbindungen
    moveit_commander.roscpp_shutdown()


if __name__ == "__main__":
    try:
        # Starte die ROS-Node
        moveit_control_node()
    except rospy.ROSInterruptException:
        pass
