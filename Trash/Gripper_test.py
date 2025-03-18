#!/usr/bin/env python3

import rospy
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

def send_gripper_command(pub, command_type):
    """Sende einen Befehl an den Gripper (öffnen, schließen, aktivieren, deaktivieren)."""
    command = outputMsg.Robotiq2FGripper_robot_output()
    
    if command_type == 'open':
        command.rPR = 255  # Öffne den Gripper
    elif command_type == 'close':
        command.rPR = 0    # Schließe den Gripper
    elif command_type == 'activate':
        command.rACT = 1   # Aktiviere den Gripper
    elif command_type == 'deactivate':
        command.rACT = 0   # Deaktiviere den Gripper
    
    # Sende den Befehl an den Gripper
    rospy.loginfo(f"Sende Befehl: {command_type}")
    pub.publish(command)

def status_callback(msg):
    """Callback-Funktion, um den Status des Grippers zu empfangen."""
    rospy.loginfo("Aktueller Gripper-Status: %s", msg)

def main():
    # Initialisierung des ROS-Knotens
    rospy.init_node('gripper_control_node')

    # Publisher für das Senden von Befehlen an den Gripper
    pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)

    # Subscriber für den Empfang von Statusinformationen des Grippers
    rospy.Subscriber('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, status_callback)

    # Steuerung des Grippers in der main Schleife
    rate = rospy.Rate(1)  # 1 Hz (einmal pro Sekunde)

    try:
        # Sende verschiedene Befehle an den Gripper
        send_gripper_command(pub, 'activate')  # Gripper aktivieren
        rospy.sleep(2)  # Warte 2 Sekunden

        send_gripper_command(pub, 'close')  # Gripper schließen
        rospy.sleep(2)  # Warte 2 Sekunden

        send_gripper_command(pub, 'open')  # Gripper öffnen
        rospy.sleep(2)  # Warte 2 Sekunden

        send_gripper_command(pub, 'deactivate')  # Gripper deaktivieren
        rospy.sleep(2)  # Warte 2 Sekunden

        # Hier könnte eine Schleife oder weitere Logik folgen, um den Gripper weiter zu steuern

    except rospy.ROSInterruptException:
        rospy.loginfo("ROS-InterruptException aufgetreten. Das Skript wird beendet.")
        
if __name__ == "__main__":
    main()
