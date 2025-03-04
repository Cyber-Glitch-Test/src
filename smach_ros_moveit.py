#!/usr/bin/env python

import numpy as np
import rospy
import moveit_commander
import sys
import smach
import smach_ros
import tf
import math
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import Grasp, PlaceLocation
from moveit_commander.move_group import MoveGroupCommander
from moveit_commander import PlanningSceneInterface
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg

#======Konstanten====== 
#Konstanten für Roboterposen (Quaternionen)
rb_arm_home             = np.array([-0.28531283917512756, 0.08176575019716574, 0.3565888897535509, 0.021838185570339213, -0.9997536365149914, 0.0006507883874787611, 0.003916171666392069])
rb_arm_over_m1          = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335, -0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m1            = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288, -0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
rb_arm_on_hum_static    = np.array([-0.2872170720236103, -0.27175826228875855, 0.38259507410129007, 0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398])
rb_arm_over_m2          = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288, -0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m2            = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335, -0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
rb_arm_over_m3          = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288, -0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m3            = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335, -0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
rb_arm_over_m4          = np.array([-0.29123786593673395, 0.08147063802929881, 0.19673868186048288, -0.022780021434265017, 0.9997249444880992, -0.005252328539882984, 0.0018759095475076684])
rb_arm_on_m4            = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335, -0.0221911382985078, 0.9997396260993958, -0.004924144052386731, 0.001996545268306617])
#Konstanten für ergonomische Berechnungen
height_hum_shoulder = 1.8
forearmlenghdin = 0.3335  # Aus DIN 33402-2 gemittelt aus Mann und Frau über alle Altersklassen
upperarmlenghtdin = 0.342  # Aus DIN 33402-2 gemittelt aus Mann und Frau über alle Altersklassen
tcp_coversion = 0.25
Hum_det = True
savety_koord_1 = np.array([0.2, 0.68, 0.8])
savety_koord_2 = np.array([-0.2, 0.5, 0.3])

#======Robot Control Class======

class RobotControl:
    def __init__(self, group_name):
        #Initialisiert die MoveIt-Gruppe und die Greifer-Node
        self.group_name = group_name
        self.move_group = MoveGroupCommander(self.group_name)
        self.gripper_controller = GripperController()
        self.scene = PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        rospy.sleep(2)  # Kurze Pause, damit die Szene initialisiert wird

        # Tischfläche in MoveIt zur Kollisionserkennung hinzufügen
        planning_frame = self.move_group.get_planning_frame()
        rospy.loginfo("Planungsrahmen: %s", planning_frame)

        p = PoseStamped()
        p.header.frame_id = planning_frame  # Setze den Planungsrahmen als Referenz
        p.pose.position.x = 0.0
        p.pose.position.y = 0.0
        p.pose.position.z = -0.09  # Etwas unter dem Boden

        self.scene.add_box("Tisch", p, size=(3, 2, 0.05))
        rospy.loginfo("Box wurde hinzugefügt.")

        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("Endeffektor-Link: %s", eef_link)

        # Setze die maximale Geschwindigkeit und Beschleunigung
        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

    def convert_to_pose(self, koords):
        #Konvertiert ein 1x7-Array in eine MoveIt-Pose
        target_pose = Pose()
        target_pose.position.x = koords[0]
        target_pose.position.y = koords[1]
        target_pose.position.z = koords[2]
        target_pose.orientation.x = koords[3]
        target_pose.orientation.y = koords[4]
        target_pose.orientation.z = koords[5]
        target_pose.orientation.w = koords[6]
        return target_pose

    def move_to_target(self, target_pose, speed):
        #Bewegt den Roboter zu einer Zielpose
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
        #Bewegt den Roboter in einer kartesischen Linie zur Zielpose
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)
        waypoints = []
        waypoints.append(self.move_group.get_current_pose().pose)
        waypoints.append(target_pose)
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.01)  # waypoints to follow, eef_step
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
        #Bewegt den Roboter zu einem definierten Gelenkwinkel
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
        #Stoppt die Bewegung des Roboters
        self.move_group.stop()
        rospy.loginfo("Roboter gestoppt!")

    def reset_robot(self):
        #Setzt den Roboter auf die Home-Position zurück
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)
        rospy.loginfo("Roboter auf 'Home' Position zurückgesetzt!")

    def calc_handover_position_schoulder(self):
        #Berechnet die ergonomische Übergabeposition basierend auf Schulterkoordinaten
        try:
            hand_over_position = Pose()
            hm = get_Hum_mertics()

            if not(all(x == 0 for x in hm.shoulderkoords)) and not(all(x == 0 for x in hm.elbowkoords)) and not(all(x == 0 for x in hm.handkoords)):
                rospy.loginfo("Schulter, Ellbogen und Hand erkannt")
                rospy.loginfo("Unterarmlänge: %s", hm.forearmlenght)
                rospy.loginfo("Oberarmlänge: %s", hm.uperarmlenght)
                hand_over_position_x = hm.shoulderkoords[0]
                hand_over_position_y = (hm.shoulderkoords[1] + (hm.forearmlenght + tcp_coversion))
                hand_over_position_z = hm.shoulderkoords[2] - hm.uperarmlenght
                hand_over_position = self.convert_to_pose(np.array([hand_over_position_x, hand_over_position_y, hand_over_position_z, 0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398]))
            elif not(all(x == 0 for x in hm.shoulderkoords)) and not(all(x == 0 for x in hm.elbowkoords)):
                rospy.loginfo("Schulter, Ellbogen erkannt")
                hand_over_position_x = hm.shoulderkoords[0]
                hand_over_position_y = (hm.shoulderkoords[1] + (forearmlenghdin + tcp_coversion))
                hand_over_position_z = hm.shoulderkoords[2] - hm.uperarmlenght
                hand_over_position = self.convert_to_pose(np.array([hand_over_position_x, hand_over_position_y, hand_over_position_z, 0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398]))
            elif not(all(x == 0 for x in hm.shoulderkoords)):
                rospy.loginfo("Schulter erkannt")
                hand_over_position_x = hm.shoulderkoords[0]
                hand_over_position_y = (hm.shoulderkoords[1] + (forearmlenghdin + tcp_coversion))
                hand_over_position_z = hm.shoulderkoords[2] - upperarmlenghtdin
                hand_over_position = self.convert_to_pose(np.array([hand_over_position_x, hand_over_position_y, hand_over_position_z, 0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398]))
            else:
                rospy.loginfo("Nichts erkannt")
                hand_over_position = self.convert_to_pose(rb_arm_on_hum_static)

            # Visualisiere die Übergabeposition in TF
            broadcaster = tf.TransformBroadcaster()
            broadcaster.sendTransform(
                (hand_over_position_x, hand_over_position_y, hand_over_position_z),  # Position der Übergabeposition
                (0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398),  # Orientierung
                rospy.Time.now(),  # Zeitstempel
                "Uebergabeposition Schulter",  # Child Frame
                "world"  # Parent Frame
            )
            return hand_over_position
        except Exception as e:
            rospy.logwarn("HD fehlgeschlagen. Fehler: %s", e)
            return self.convert_to_pose(rb_arm_on_hum_static)

    def calc_handover_position_hand(self):
        """Berechnet die ergonomische Übergabeposition basierend auf Handkoordinaten."""
        try:
            hand_over_position = Pose()
            hm = get_Hum_mertics()

            if not(all(x == 0 for x in hm.handkoords)):
                rospy.loginfo("Hand erkannt")
                hand_over_position_x = hm.handkoords[0]
                hand_over_position_y = (hm.handkoords[1] + tcp_coversion)
                hand_over_position_z = hm.handkoords[2]
                hand_over_position = self.convert_to_pose(np.array([hand_over_position_x, hand_over_position_y, hand_over_position_z, 0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398]))
            else:
                rospy.loginfo("Keine Hand erkannt")
                hand_over_position = self.convert_to_pose(rb_arm_on_hum_static)

            # Visualisiere die Übergabeposition in TF
            broadcaster = tf.TransformBroadcaster()
            broadcaster.sendTransform(
                (hand_over_position_x, hand_over_position_y, hand_over_position_z),  # Position der Übergabeposition
                (0.017952569275050657, -0.750361039466253, 0.6606544978371074, 0.01310153407614398),  # Orientierung
                rospy.Time.now(),  # Zeitstempel
                "Uebergabeposition Hand",  # Child Frame
                "world"  # Parent Frame
            )
            return hand_over_position
        except Exception as e:
            rospy.logwarn("HD fehlgeschlagen. Fehler: %s", e)
            return self.convert_to_pose(rb_arm_on_hum_static)

    def point_inside(self, pose):
        #Überprüft, ob die Übergabeposition innerhalb eines Sicherheitsrechtecks liegt
        point = [pose.position.x, pose.position.y, pose.position.z]
        xmin, xmax = savety_koord_1[0] - 1, savety_koord_2[0] + 1
        ymin, ymax = savety_koord_1[1] - 1, savety_koord_2[1] + 1
        zmin, zmax = savety_koord_1[2] - 1, savety_koord_2[2] + 1
        return xmin < point[0] < xmax and ymin < point[1] < ymax and zmin < point[2] < zmax

#======Gripper Control======

class GripperController:
    def __init__(self):
        #Initialisiert den Gripper-Controller
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        rospy.Subscriber('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.status_callback)
        self.gripper_status = inputMsg.Robotiq2FGripper_robot_input()
        self.command = outputMsg.Robotiq2FGripper_robot_output()

    def status_callback(self, msg):
        #Callback-Funktion, um den Status des Greifers zu empfangen
        self.gripper_status = msg

    def send_gripper_command(self, action_type):
        #Sendet Befehle an den Greifer
        if action_type == 'open':
            self.command.rPR = 0
        elif action_type == 'close':
            self.command.rPR = 255
        elif action_type == 'activate':
            self.command.rACT = 1
            self.command.rGTO = 1
            self.command.rSP = 255
            self.command.rFR = 150
        elif action_type == 'deactivate':
            self.command.rACT = 0
        rospy.sleep(2)
        self.pub.publish(self.command)

#======Get Hum Data======

class get_Hum_mertics:
    #innitiere tracking des Menschen
    def __init__(self):
        self.uperarmlenght = 0
        self.forearmlenght = 0
        self.shoulderkoords = [0, 0, 0]
        self.elbowkoords = [0, 0, 0]
        self.handkoords = [0, 0, 0]
        self.calc_arm_lenght()

    def camera_listener(self):
    #lese ROS nachrichten aus
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
    #bestimme ober und unterarm länge
        self.camera_listener()
        self.uperarmlenght = self.calc_euclidean_distance(self.shoulderkoords,  self.elbowkoords)
        self.forearmlenght = self.calc_euclidean_distance(self.handkoords,      self.elbowkoords)

    def calc_euclidean_distance(self, point1, point2):
    #bestimme den euclidischen Abstand zwischen zwei Punkten
        distance = 0.0
        for i in range(len(point1)):
            distance += (point2[i] - point1[i]) ** 2
        return math.sqrt(distance)

robot_control = RobotControl("manipulator")

################################ Initialisiere Smachstates ################################

class M1PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'succeeded_with_HD'])
        self.robot_control = robot_control

    def execute(self, userdata):
        #nehme Motor1 auf
        rospy.loginfo('Executing state: M1PickUp')
        return 'succeeded_with_HD'
        if not self.robot_control.move_to_target(self.robot_control.convert_to_pose(rb_arm_home), 5):
            return 'succeeded'

        self.robot_control.gripper_controller.send_gripper_command('activate')
        self.robot_control.gripper_controller.send_gripper_command('close')
        self.robot_control.gripper_controller.send_gripper_command('open')

        if not self.robot_control.move_to_target(self.robot_control.convert_to_pose(rb_arm_over_m1), 10):
            return 'succeeded'

        if not self.robot_control.move_to_target(self.robot_control.convert_to_pose(rb_arm_on_m1), 5):
            return 'succeeded'

        self.robot_control.gripper_controller.send_gripper_command('close')

        if not self.robot_control.move_to_target_carth(self.robot_control.convert_to_pose(rb_arm_over_m1), 10):
            return 'succeeded'

        if not self.robot_control.move_to_target(self.robot_control.convert_to_pose(rb_arm_on_hum_static), 10):
            return 'succeeded'

        if self.robot_control.point_inside(self.robot_control.calc_handover_position_schoulder()):
            return 'succeeded_with_HD'
        else:
            return 'succeeded'

class M1Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot_control = robot_control
    def execute(self, userdata):
        rospy.loginfo('Executing state: M1Hold')
                
        if not self.robot_control.move_to_target(self.robot_control.convert_to_pose(rb_arm_on_hum_static), 10):
            return 'succeeded'
        return 'succeeded'
class M1HoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot_control = robot_control
    def execute(self, userdata):
        rospy.loginfo('Executing state: M1Hold')
        ####

        self.robot_control.move_to_joint_goal( (3.0931, -2.3744, 1.9545, -1.0704, -0.0150, -1.6596), 20)

        if not self.robot_control.move_to_target(self.robot_control.calc_handover_position_schoulder(),5):
            return 'succeeded'  
        return 'succeeded'

class M1Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M1Positioning')
        ####
        return 'succeeded'

class M2PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_with_HD'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M2PickUp')
        ####
        return 'succeeded'

class M2Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M2Hold')
        ####
        return 'succeeded'
    
class M2HoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M2Hold')
        ####
        return 'succeeded'

class M2Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M2Positioning')
        ####
        return 'succeeded'

class M3PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_with_HD'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M3PickUp')
        ####
        return 'succeeded'

class M3Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M3Hold')
        ####
        return 'succeeded'
    
class M3HoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M3Hold')
        ####
        return 'succeeded'

class M3Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M3Positioning')
        ####
        return 'succeeded'

class M4PickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_with_HD'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M4PickUp')
        ####
        return 'succeeded'

class M4Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M4Hold')
        ####
        return 'succeeded'
    
class M4HoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M4Hold')
        ####
        return 'succeeded'

class M4Positioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M4Positioning')
        ####
        return 'succeeded'

class PCB1PickUpAndPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: PCB1PickUpAndPositioning')
        ####
        return 'succeeded'

class PCB2PickUpAndPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: PCB2PickUpAndPositioning')
        ####
        return 'succeeded'

class CopperFixing1To6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: CopperFixing1To6')
        ####
        return 'succeeded'


class BatteryPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: BatteryPositioning')
        ####
        return 'succeeded'

class BatteryFixing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: BatteryFixing')
        ####
        return 'succeeded'




if __name__ == "__main__":

    rospy.init_node('ur5_moveit_control', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    robot_control = RobotControl("manipulator")
    

    sm = smach.StateMachine(outcomes=['finished'])
    with sm:
        # Smachstates
        smach.StateMachine.add('M1PickUp', M1PickUp(),
                               transitions={'succeeded':'M1Hold',
                                            'succeeded_with_HD':'M1HoldHD'})
        smach.StateMachine.add('M1Hold', M1Hold(),
                               transitions={'succeeded':'M1Positioning'})
        smach.StateMachine.add('M1HoldHD', M1HoldHD(),
                               transitions={'succeeded':'M1Positioning'})
        smach.StateMachine.add('M1Positioning', M1Positioning(),
                               transitions={'succeeded':'M2PickUp'})
        smach.StateMachine.add('M2PickUp', M2PickUp(),
                               transitions={'succeeded':'M2Hold',
                                            'succeeded_with_HD':'M2HoldHD'})
        smach.StateMachine.add('M2Hold', M2Hold(),
                               transitions={'succeeded':'M2Positioning',})
        smach.StateMachine.add('M2HoldHD', M2HoldHD(),
                               transitions={'succeeded':'M2Positioning'})
        smach.StateMachine.add('M2Positioning', M2Positioning(),
                               transitions={'succeeded':'M3PickUp'})
        smach.StateMachine.add('M3PickUp', M3PickUp(),
                               transitions={'succeeded':'M3Hold',
                                            'succeeded_with_HD':'M3HoldHD'})
        smach.StateMachine.add('M3Hold', M3Hold(),
                               transitions={'succeeded':'M3Positioning'})
        smach.StateMachine.add('M3HoldHD', M3HoldHD(),
                               transitions={'succeeded':'M3Positioning'})
        smach.StateMachine.add('M3Positioning', M3Positioning(),
                               transitions={'succeeded':'M4PickUp'})
        smach.StateMachine.add('M4PickUp', M4PickUp(),
                               transitions={'succeeded':'M4Hold',
                                            'succeeded_with_HD':'M4HoldHD'})
        smach.StateMachine.add('M4Hold', M4Hold(),
                               transitions={'succeeded':'M4Positioning'})
        smach.StateMachine.add('M4HoldHD', M4HoldHD(),
                               transitions={'succeeded':'M4Positioning'})
        smach.StateMachine.add('M4Positioning', M4Positioning(),
                               transitions={'succeeded':'PCB1PickUpAndPositioning'})
        smach.StateMachine.add('PCB1PickUpAndPositioning', PCB1PickUpAndPositioning(),
                               transitions={'succeeded':'PCB2PickUpAndPositioning'})
        smach.StateMachine.add('PCB2PickUpAndPositioning', PCB2PickUpAndPositioning(),
                               transitions={'succeeded':'BatteryPositioning'})
        smach.StateMachine.add('BatteryPositioning', BatteryPositioning(),
                               transitions={'succeeded':'finished'})

    # Iniizialisiere den introspection server
try:
    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()
except AttributeError as e:
    rospy.logwarn(f"IntrospectionServer not found. Falling back to manual debugging. Error: {e}")

    # Führe die Statemachine aus
    outcome = sm.execute()
    rospy.spin() 

