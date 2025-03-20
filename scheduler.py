#!/usr/bin/env python

import numpy as np # type: ignore
import rospy # type: ignore
import moveit_commander # type: ignore
import sys
import smach # type: ignore
import smach_ros # type: ignore
import tf   # type: ignore
import math
import copy
import time
from tf.transformations import quaternion_from_euler  # type: ignore
from geometry_msgs.msg import Pose, PoseStamped , PointStamped # type: ignore
from moveit_msgs.msg import Grasp, PlaceLocation # type: ignore
from moveit_commander.move_group import MoveGroupCommander # type: ignore
from moveit_commander import PlanningSceneInterface # type: ignore
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg # type: ignore
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg # type: ignore

#======Konstanten====== 
#Konstanten für TCP-Ausrichtung
tcp_to_hum = [0.7151898629008024, 0.007492056498778332, -0.0038630942023753514, 0.6988793927399307]

#Konstanten für Roboterposen
rb_arm_home = np.array([-0.28531283917512756,  0.08176575019716574, 0.3565888897535509, 0.021838185570339213, -0.9997536365149914, 0.0006507883874787611, 0.003916171666392069])

rb_arm_on_m =  [np.array([0.2631105225136129,    0.11513901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.2631105225136129,    0.06813901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.2631105225136129,    0.02113901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.2631105225136129,    -0.02613901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    0.11513901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    0.06813901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    0.02113901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    -0.02613901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    0.11513901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    0.06813901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    0.02113901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    -0.02613901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5031105225136129,    0.11513901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5031105225136129,    0.06813901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5031105225136129,    0.02113901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5031105225136129,    -0.02613901314207496, 0.20474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008])]

rb_arm_on_hum_static = np.array([0.01127138298740326, -0.40789791168606154, 0.4347020900402719,0.65967278113823, 0.13322073168864898, -0.04031615244060301, 0.7385517357139446])

rb_arm_on_pcb1  =  [np.array([0.6316488317010515, -0.13953502575569454, 0.17244747378939593  ,0.7074744139374561, -0.7066996961733456, 0.007443486177193687, 0.0002959153328908581]),
                    np.array([0.6866488317010515, -0.13953502575569454, 0.17244747378939593  ,0.7074744139374561, -0.7066996961733456, 0.007443486177193687, 0.0002959153328908581]),
                    np.array([0.7416488317010515, -0.13953502575569454, 0.17244747378939593  ,0.7074744139374561, -0.7066996961733456, 0.007443486177193687, 0.0002959153328908581]),
                    np.array([0.7966488317010515, -0.13953502575569454, 0.17244747378939593  ,0.7074744139374561, -0.7066996961733456, 0.007443486177193687, 0.0002959153328908581])]

rb_arm_on_battery =[np.array([0.6011670779056063, -0.019193581297668794, 0.17018491325288876   ,0.0032537936019315095, 0.999980030621379, 0.0013019363032707083, 0.00525891124897437]),
                    np.array([0.6011670779056063,  0.103193581297668794, 0.17018491325288876   ,0.0032537936019315095, 0.999980030621379, 0.0013019363032707083, 0.00525891124897437]),
                    np.array([0.7011670779056063, -0.019193581297668794, 0.17018491325288876 ,0.0032537936019315095, 0.999980030621379, 0.0013019363032707083, 0.00525891124897437]),
                    np.array([0.7011670779056063,  0.101393581297668794, 0.17018491325288876   ,0.0032537936019315095, 0.999980030621379, 0.0013019363032707083, 0.00525891124897437])]


#Konstanten für ergonomische Berechnungen
forearmlenghdin = 0.2688  # Aus DIN 33402-2 gemittelt aus Mann und Frau über alle Altersklassen
upperarmlenghtdin = 0.342  # Aus DIN 33402-2 gemittelt aus Mann und Frau über alle Altersklassen

forearmlenghdin_max = 0.355
forearmlenghdin_min = 0.187

upperarmlenghtdin_max = 0.405
upperarmlenghtdin_min = 0.285

tcp_coversion = 0.2



savety_koord_1 = np.array([0.25, 0.0, 0.6])
savety_koord_2 = np.array([-0.3, -0.7, 0.0])

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

        Tisch = PoseStamped()
        Tisch.header.frame_id = planning_frame
        Tisch.pose.position.x = 0.0
        Tisch.pose.position.y = 0.0
        Tisch.pose.position.z = -0.09 
        
        self.scene.add_box("Tisch", Tisch, size=(3, 2, 0.05))
        rospy.loginfo("Tisch wurde Planungszene hinzugefügt.")

        Wand = PoseStamped()
        Wand.header.frame_id = planning_frame 
        Wand.pose.position.x = -0.37
        Wand.pose.position.y = 0.00
        Wand.pose.position.z = 0.00 
        
        self.scene.add_box("Wand", Wand, size=(0.05, 3, 3))
        rospy.loginfo("Wand wurde Planungszene hinzugefügt")

        Decke = PoseStamped()
        Decke.header.frame_id = planning_frame  
        Decke.pose.position.x = 0.0
        Decke.pose.position.y = 0.0
        Decke.pose.position.z = 0.92 
        
        self.scene.add_box("Decke", Decke, size=(3, 2, 0.05))
        rospy.loginfo("Decke wurde Planungszene hinzugefügt.")

        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("Endeffektor-Link: %s", eef_link)

        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

    def convert_to_pose(self, koords):
        #Konvertiert ein 1x7-Array in eine Pose
        target_pose = Pose()
        target_pose.position.x = koords[0]
        target_pose.position.y = koords[1]
        target_pose.position.z = koords[2]
        target_pose.orientation.x = koords[3]
        target_pose.orientation.y = koords[4]
        target_pose.orientation.z = koords[5]
        target_pose.orientation.w = koords[6]
        return target_pose
    
    def convert_to_koords(self, pose= Pose()):
        #Konvertiert eine Pose in ein 1x7-Array
        koords = [0.0,0.0,0.0,0.0,0.0,0.0,0.0]
        koords[0] = pose.position.x
        koords[1] = pose.position.y
        koords[2] = pose.position.z
        koords[3] = pose.orientation.x
        koords[4] = pose.orientation.y
        koords[5] = pose.orientation.z
        koords[6] = pose.orientation.w
        return koords

    def move_to_target(self, target_pose, speed):
        #Bewegt den Roboter zu einer Zielpose
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)
        self.move_group.set_pose_target(target_pose)
        rospy.loginfo("Bewege Roboter zu: x={}, y={}, z={}".format(target_pose.position.x, target_pose.position.y, target_pose.position.z))
        # rospy.loginfo("0.2662104568594572, -0.35661957908057046, 0.24265798894634866 | Orientation: 0.0050765060764118896, -0.8027125907596652, 0.5948306511336113, 0.042464363811632704")
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
        waypoints.append(target_pose)
        self.move_group.set_planning_time(10.0) 
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.05) 
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

    def move_to_taget_plan(self, waypoints, speed):
        #Fahre mit dem Roboterarm eine Reihe von Waipoints an
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)
        for i, waypoint in enumerate(waypoints):
            self.move_group.set_pose_target(waypoint)
            plan = self.move_group.plan()  
            if plan[0]:  
                rospy.loginfo(f"Führe Waypoint {i+1} aus...")
                self.move_group.execute(plan[1], wait=True) 
            else:
                rospy.logwarn(f"Konnte Waypoint {i+1} nicht erreichen!")

    def move_to_joint_goal(self, joint_goal, speed):
        #Bewegt den Roboter zu einem Gelenkwinkel
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

    def handover_to_hum(self,speed):

        #führe die Roboter bewegung zum Menschen aus
        handover_pose_end = Pose()
        handover_pose_end = self.calc_handover_position_schoulder()
        handover_pose_start = copy.deepcopy(handover_pose_end)
        handover_pose_start.position.y = handover_pose_end.position.y + 0.1
        rospy.loginfo("Bewege Roboter zu: x={}, y={}, z={}".format(handover_pose_start.position.x, handover_pose_start.position.y, handover_pose_start.position.z))


        if self.point_inside(handover_pose_end):
            if not self.move_to_joint_goal( (-4.2306, -1.3338, -2.1678, -2.7849, -1.1028, -0.0036), 20):
                return False
            if not self.move_to_target_carth(handover_pose_start,speed):
                return False
            if not self.move_to_target_carth(handover_pose_end,speed):
                return False
            return True
        else:
            return False

    def calc_handover_position_schoulder(self):
        #Berechnet die ergonomischste Übergabeposition basierend auf Schulterkoordinaten
        hm = get_Hum_mertics()
        broadcaster = tf.TransformBroadcaster()
        listener = tf.TransformListener()  
            #hand_over_position = Pose()
        for i in range(2):
            for sek in range(10):
                if not(all(x == 0 for x in hm.shoulderkoords)) and not(all(x == 0 for x in hm.elbowkoords)) and not(all(x == 0 for x in hm.handkoords)) and hm.inside_norm_upper and hm.inside_norm_fore:
                #if not(all(x == 0 for x in hm.shoulderkoords)) and not(all(x == 0 for x in hm.elbowkoords)) and not(all(x == 0 for x in hm.handkoords)):    
                    rospy.loginfo("Schulter, Ellbogen und Hand erkannt")
                    rospy.loginfo("Unterarmlänge: %s", hm.forearmlenght)
                    rospy.loginfo("Oberarmlänge: %s", hm.uperarmlenght)
                    rospy.loginfo("Schulterkoords: %s", hm.shoulderkoords)

                    translation = [ 0, (hm.forearmlenght + tcp_coversion),- hm.uperarmlenght]
                    rotation = quaternion_from_euler(((0/180)*math.pi),((0/180)*math.pi),((0/180)*math.pi))
                    break

                elif not(all(x == 0 for x in hm.shoulderkoords)) and not(all(x == 0 for x in hm.elbowkoords)) and hm.inside_norm_upper:
                    
                    rospy.loginfo("Schulter, Ellbogen erkannt")
                    rospy.loginfo("Oberarmlänge: %s", hm.uperarmlenght)
                    rospy.loginfo("Schulterkoords: %s", hm.shoulderkoords)

                    translation = [ 0,(forearmlenghdin + tcp_coversion),- hm.uperarmlenght]
                    rotation = quaternion_from_euler(((0/180)*math.pi),((0/180)*math.pi),((0/180)*math.pi))
                    break

                elif not(all(x == 0 for x in hm.shoulderkoords)):
                    rospy.loginfo("Schulter erkannt")
                    rospy.loginfo("Unterarmlänge: %s", hm.forearmlenght)
                    rospy.loginfo("Oberarmlänge: %s", hm.uperarmlenght)
                    rospy.loginfo("Schulterkoords: %s", hm.shoulderkoords)
                    translation = [ 0,(forearmlenghdin + tcp_coversion),- upperarmlenghtdin]
                    rotation = quaternion_from_euler(((0/180)*math.pi),((0/180)*math.pi),((0/180)*math.pi))
                    break

                else:
                    rospy.loginfo(f"Nichts erkannt Versuch: {sek}/10")
                    time.sleep(1)

            try:    
                    
                    handover_point = PointStamped()
                    handover_point.header.frame_id = "right_shoulder"
                    handover_point.header.stamp = rospy.Time.now() 
                    handover_point.point.x = translation[0]
                    handover_point.point.y = -translation[1]
                    handover_point.point.z = translation[2]

                    time.sleep(3)
                    listener.waitForTransform("base","right_shoulder", rospy.Time(0), rospy.Duration(1.0))
                        
                    broadcaster.sendTransform(
                        (handover_point.point.x, handover_point.point.y,handover_point.point.z),
                        (0.0, 0.0, 0.0, 1.0),  
                        rospy.Time.now(),
                        "handover_position",
                        "right_shoulder"
                    )

                    listener.waitForTransform("base","handover_position", rospy.Time(0), rospy.Duration(1.0))
                    hand_over_position_koords, _ = listener.lookupTransform("base","handover_position",  rospy.Time(0))

                    
                    
                        #hand_over_position = self.convert_to_pose(np.array([0,-0.4,0.4,tcp_to_hum[0],tcp_to_hum[1],tcp_to_hum[2],tcp_to_hum[3]]))
                    hand_over_position = self.convert_to_pose(np.array([-hand_over_position_koords[0],-hand_over_position_koords[1],hand_over_position_koords[2],tcp_to_hum[0],tcp_to_hum[1],tcp_to_hum[2],tcp_to_hum[3]]))
            except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
                    rospy.logwarn(f"Error transforming point: {e}")

            hm = None
            return hand_over_position
        
    def reset_robot(self):
        self.move_group.set_named_target("home")
        self.move_group.go(wait=True)
        rospy.loginfo("Roboter auf 'Home' Position zurückgesetzt!")

    def point_inside(self, pose):
        #Überprüft, ob die Übergabeposition innerhalb eines Sicherheitsrechtecks liegt

        point = [pose.position.x, pose.position.y, pose.position.z]
        xmin, xmax = sorted([savety_koord_1[0], savety_koord_2[0]])
        ymin, ymax = sorted([savety_koord_1[1], savety_koord_2[1]])
        zmin, zmax = sorted([savety_koord_1[2], savety_koord_2[2]])
        rospy.logwarn("Punkt:")
        rospy.logwarn(point)
        print(f"Checking point: {point}, Allowed Range: x[{xmin}, {xmax}], y[{ymin}, {ymax}], z[{zmin}, {zmax}]")
        return xmin < point[0] < xmax and ymin < point[1] < ymax and zmin < point[2] < zmax
    
    def pick_up(self,target):
        over_target = target.copy()
        target[2] =  target[2] - 0.01
        over_target[2] += 0.1  

        self.move_to_target(self.convert_to_pose(over_target), 5)
        self.gripper_controller.send_gripper_command('open')
        self.move_to_target_carth(self.convert_to_pose(target), 10)
        self.gripper_controller.send_gripper_command('close')
        self.move_to_target_carth(self.convert_to_pose(over_target), 5)

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
        self.pub.publish(self.command)
        rospy.sleep(2)

#======Get Hum Data======

class get_Hum_mertics:
    #innitiere tracking des Menschen
    def __init__(self):
        self.uperarmlenght = 0.0
        self.forearmlenght = 0.0
        self.shoulderkoords = [0.0, 0.0, 0.0]
        self.elbowkoords =    [0.0, 0.0, 0.0]
        self.handkoords =     [0.0, 0.0, 0.0]
        self.inside_norm_upper = True
        self.inside_norm_fore  = True
        self.calc_arm_lenght()

    def camera_listener(self):
    #lese tf für Schulter Elebogen und Hand aus
        try:

            time = rospy.Time(0)
            listener = tf.TransformListener()

            listener.waitForTransform("base", "right_shoulder", time, rospy.Duration(1.0))
            listener.waitForTransform("base", "right_elbow",    time, rospy.Duration(1.0))
            listener.waitForTransform("base", "right_hand",     time, rospy.Duration(1.0))

            shoulder_trans, _ = listener.lookupTransform("base", "right_shoulder",  time)
            elbow_trans,    _ = listener.lookupTransform("base", "right_elbow",     time)
            hand_trans,     _ = listener.lookupTransform("base", "right_hand",      time)

            self.shoulderkoords =   [shoulder_trans[0], shoulder_trans[1], shoulder_trans[2]]
            self.elbowkoords =      [elbow_trans[0], elbow_trans[1], elbow_trans[2]]
            self.handkoords =       [hand_trans[0], hand_trans[1], hand_trans[2]]

        except (tf.Exception, tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn(f"TF Error: {e}")

    def calc_arm_lenght(self):
    #bestimme ober und unterarm länge

        self.camera_listener()
        self.uperarmlenght = self.calc_euclidean_distance(self.shoulderkoords,  self.elbowkoords)
        self.forearmlenght = self.calc_euclidean_distance(self.handkoords,      self.elbowkoords)
        self.is_inside_norm()

    def calc_euclidean_distance(self, point1, point2):
    #bestimme den euclidischen Abstand zwischen zwei Punkten

        distance = 0.0
        for i in range(len(point1)):
            distance += (point2[i] - point1[i]) ** 2
        return math.sqrt(distance)
    
    def is_inside_norm(self):
        #Überprüft ob Ober und Unterarm innerhalb des 5. und 95 Perzentil sind

        if (self.uperarmlenght <= upperarmlenghtdin_max) and (self.uperarmlenght >= upperarmlenghtdin_min):
            self.inside_norm_upper = True
        else:
            rospy.logwarn(f"Oberarmmaße sind außerhalb 5. bis 95 Perzentil")
            self.inside_norm_upper = False

        if(self.forearmlenght <= forearmlenghdin_max) and (self.forearmlenght <= forearmlenghdin_min):
            self.inside_norm_fore = True
        else:
            rospy.logwarn(f"Unterarmmaße sind außerhalb 5. bis 95 Perzentil")
            self.inside_norm_fore = False

        

robot_control = RobotControl("manipulator")

################################ Initialisiere Smachstates ################################

class MPickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'succeeded_with_HD'])
        self.robot_control = robot_control
        self.counter = 0
        self.gripper_controller = robot_control.gripper_controller
    def execute(self, userdata):
        #nehme Motor1 auf


        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        ''''DEBUG BLOCK ZUM TESTEN'''
        while True:
            newuser = input('enter y/n: ')
            if newuser == "y":
                self.gripper_controller.send_gripper_command('activate')
                self.gripper_controller.send_gripper_command('close')
                self.gripper_controller.send_gripper_command('open')
                robot_control.pick_up(rb_arm_on_m[self.counter])
                self.counter += 1
                rospy.loginfo(f"Nehme Motor {self.counter} auf")
                return 'succeeded_with_HD'
            elif newuser == "n":
                return 'succeeded'

class MHold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
        self.robot_control = robot_control
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
                
class MHoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_end'])
        self.robot_control = robot_control
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        ####
        #self.robot_control.move_to_joint_goal( (-1.1814, -1.9903, 2.2022, -0.1776, -4.3650, 4.7049), 5)
        ''''DEBUG BLOCK ZUM TESTEN'''
        while True:
            newuser = input('enter y/n: ')
            if newuser == "y":
                rospy.loginfo("Roboter Pose...")
                #self.robot_control.move_to_joint_goal( (1.9268, -1.4306, -2.6785, 0.8303, 1.2253, 0.0456), 5)
                # if not self.robot_control.move_to_target(robot_control.convert_to_pose([0.0,-0.4,0.4,tcp_to_hum[0],tcp_to_hum[1],tcp_to_hum[2],tcp_to_hum[3]]),5):
                #     rospy.loginfo('bewegung Fehlgeschlagen')
                # continue
                if not self.robot_control.handover_to_hum(5):
                    rospy.loginfo('bewegung Fehlgeschlagen')
                continue
            elif newuser == "n":
                print("Exiting")
                return 'succeeded_end'
                break
        ''''DEBUG BLOCK ZUM TESTEN ENDE'''
        return 'succeeded'
        # if not self.robot_control.move_to_target(self.robot_control.calc_handover_position_schoulder(),5):
        #     return 'succeeded'  
        # return 'succeeded'

class MPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_to_PCB'])
        self.robot_control = robot_control
        self.counter = 0
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        self.counter += 1
        if (self.counter % 4==0):
            return 'succeeded_to_PCB'
        else:
            return 'succeeded'


class PCB1PickUpAndPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        return 'succeeded'

class PCB2PickUpAndPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        ####
        return 'succeeded'

class CopperFixing1To6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        ####
        return 'succeeded'

class BatteryPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_end'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: BatteryPositioning')
        ####
        return 'succeeded'

class BatteryFixing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_end'])
        self.counter = 0

    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        self.counter += 1
        if(self.counter <= 4):
            return 'succeeded'
        else:
            return 'succeeded_end'


if __name__ == "__main__":

    rospy.init_node('ur5_moveit_control', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    robot_control = RobotControl("manipulator")

    sm = smach.StateMachine(outcomes=['finished'])
    with sm:
        # Smachstates
        smach.StateMachine.add('MPickUp', MPickUp(),
                               transitions={'succeeded':'MHold',
                                            'succeeded_with_HD':'MHoldHD'})
        smach.StateMachine.add('MHold', MHold(),
                               transitions={'succeeded':'MPositioning'})
        smach.StateMachine.add('MHoldHD', MHoldHD(),
                               transitions={'succeeded':'MPositioning',
                                            'succeeded_end':'finished'})
        smach.StateMachine.add('MPositioning', MPositioning(),
                               transitions={'succeeded':'MPickUp',
                                            'succeeded_to_PCB':'PCB1PickUpAndPositioning'})
        smach.StateMachine.add('PCB1PickUpAndPositioning', PCB1PickUpAndPositioning(),
                               transitions={'succeeded':'PCB2PickUpAndPositioning'})
        smach.StateMachine.add('PCB2PickUpAndPositioning', PCB2PickUpAndPositioning(),
                               transitions={'succeeded':'BatteryPositioning'})
        smach.StateMachine.add('BatteryPositioning', BatteryPositioning(),
                               transitions={'succeeded_end':'finished',
                                            'succeeded':'MPickUp'})

    # Iniizialisiere den introspection server
    try:
        sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
        sis.start()
    except AttributeError as e:
        rospy.logwarn(f"IntrospectionServer not found. Falling back to manual debugging. Error: {e}")

    # Führe die Statemachine aus
    outcome = sm.execute()
    rospy.spin() 

