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
import csv
import threading
from tf.transformations import quaternion_from_euler  # type: ignore
from geometry_msgs.msg import Pose, PoseStamped , PointStamped # type: ignore
from moveit_msgs.msg import Grasp, PlaceLocation # type: ignore
from moveit_commander.move_group import MoveGroupCommander # type: ignore
from moveit_commander import PlanningSceneInterface # type: ignore
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output as outputMsg # type: ignore
from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_input as inputMsg # type: ignore


#======Konstanten====== 
#Konstanten für TCP-Ausrichtung
tcp_to_hum = [-0.0017881569928987558, -0.6960133488624333, 0.7180244453880938, 0.0017653682307088938]

#Konstanten für Roboterposen
rb_arm_home = np.array([-0.28531283917512756,  0.08176575019716574, 0.3565888897535509, 0.021838185570339213, -0.9997536365149914, 0.0006507883874787611, 0.003916171666392069])

rb_arm_on_m =  [np.array([0.2631105225136129,    0.11513901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.2631105225136129,    0.06813901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.2631105225136129,    0.02113901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.2631105225136129,    -0.02613901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    0.11513901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    0.06813901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    0.02113901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.3431105225136129,    -0.02613901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    0.11513901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    0.06813901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    0.02113901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.4231105225136129,    -0.02613901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5051105225136129,    0.11513901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5051105225136129,    0.06813901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5051105225136129,    0.02113901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008]),
                np.array([0.5051105225136129,    -0.02613901314207496, 0.19474944789272417 ,0.018266303149021744, 0.9997308933491994, -0.010420321910118447, 0.009792851666864008])]

rb_arm_on_hum_static = np.array([0.01127138298740326, -0.40789791168606154, 0.4347020900402719,0.65967278113823, 0.13322073168864898, -0.04031615244060301, 0.7385517357139446])

rb_arm_transition =             np.array([0.22048980978459626, -0.11962800779329041, 0.22232535871506093 ,-0.00519597519482744, -0.7000337195214675, 0.7140685181262056, 0.005651972731604554])
rb_arm_transition_over_m =      np.array([0.32755193192480295, 0,                    0.3552028979677898 ,-0.002982105237080432, -0.9999915258909946, 0.00274986658972347, 0.0007024445132438654])
rb_arm_transition_over_pcb1 =   np.array([0.7371109279194257, -0.12405534656551466, 0.3564804416147143 ,0.701903624069778, 0.7119699919919052, 0.017634125210474614, 0.010911949817505392])
rb_arm_transition_over_pcb2 =   np.array([0.39299783753064255, -0.25037007326362604, 0.4098002793824048  ,0.9994996999243878, 0.018132610811126923, -0.01488422170868633, 0.021213632889197198])

rb_arm_transition_over_gb0_1 =  np.array([0.43920883565114404, -0.27118297223348553, 0.21533919567733978,-0.0017450344372635439, -0.6960370290652399, 0.7180016795804389, 0.0017312263041814983])
rb_arm_transition_over_gb0_2 =  np.array([0.43920912923516957, -0.21053279915028705, 0.3005243278363052,-0.0017450344372635439, -0.6960370290652399, 0.7180016795804389, 0.0017312263041814983])

rb_arm_transition_over_gb1_1 =  np.array([0.4384443484397782, -0.33196635637380156, 0.3911058561909323,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108])
rb_arm_transition_over_gb1_2 =  np.array([0.4384443484397782, -0.46021622516842065, 0.3910704893338057  ,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108])
rb_arm_transition_over_gb1_3 =  np.array([0.4384443484397782, -0.46018886151503624, 0.35487302405715704  ,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108])

rb_arm_transition_over_gb2_1 =  np.array([0.5486854170473805, -0.3885145028949433, 0.3814376455406984 ,-0.0020243784347149197, 0.9996642271967776, 0.02193613735553668, 0.013643336576571479])
rb_arm_transition_over_gb2_2 =  np.array([0.5486854170473805, -0.4685145028949433, 0.3514376455406984 ,-0.0020243784347149197, 0.9996642271967776, 0.02193613735553668, 0.013643336576571479])
rb_arm_transition_over_gb2_3 =  np.array([0.5486854170473805, -0.27057393609447655, 0.35141967557281056 ,-0.0020243784347149197, 0.9996642271967776, 0.02193613735553668, 0.013643336576571479])

rb_arm_transition_over_gb3_1 =  np.array([0.6402808547359244, -0.26790083190492066, 0.38354439061807685 ,0.019159378644141387, 0.999622466840548, -0.005839393784188026, 0.018808069486830555])
rb_arm_transition_over_gb3_2 =  np.array([0.6402808547359244, -0.46790083190492066, 0.38354439061807685 ,0.019159378644141387, 0.999622466840548, -0.005839393784188026, 0.018808069486830555])
rb_arm_transition_over_gb3_3 =  np.array([0.6402808547359244, -0.46790083190492066, 0.35478830422197494 ,0.010290457772605947, 0.9997533010888775, -0.005696452057469483, 0.018841281131592193
])

rb_arm_on_pcb1  =  [np.array([0.6316488317010515, -0.13953502575569454, 0.16890158973568933 ,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108]),
                    np.array([0.6866488317010515, -0.13953502575569454, 0.16890158973568933  ,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108]),
                    np.array([0.7416488317010515, -0.13953502575569454, 0.16890158973568933  ,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108]),
                    np.array([0.7966488317010515, -0.13953502575569454, 0.16890158973568933 ,0.703591897260684, 0.7105805074782172, 0.0027980514341484353, 0.005094645157629108])]

rb_arm_on_pcb2  =  [np.array([0.4165396170280912, -0.2591341631409675, 0.15891605521745683   ,0.7090144359375834, 0.7047068297621026, 0.006839549337615578, 0.02529889886172804]),
                    np.array([]),
                    np.array([]),
                    np.array([])]

rb_arm_on_battery =[np.array([0.6064043378480363, -0.019193581297668794, 0.15491817631772764   ,0.9995912737424026, 0.012216905158772546, -0.012814390858456945, 0.022446025779846696]),
                    np.array([0.6064043378480363,  0.103193581297668794, 0.15491817631772764   ,0.9995912737424026, 0.012216905158772546, -0.012814390858456945, 0.022446025779846696]),
                    np.array([0.7079493583489366, -0.019193581297668794, 0.15491817631772764   ,0.9995912737424026, 0.012216905158772546, -0.012814390858456945, 0.022446025779846696]),
                    np.array([0.7079493583489366,  0.101393581297668794, 0.15491817631772764   ,0.9995912737424026, 0.012216905158772546, -0.012814390858456945, 0.022446025779846696])]


#Konstanten für ergonomische Berechnungen
forearmlenghdin = 0.2688   # Aus DIN 33402-2 gemittelt aus Mann und Frau über alle Altersklassen
upperarmlenghtdin = 0.342  # Aus DIN 33402-2 gemittelt aus Mann und Frau über alle Altersklassen

forearmlenghdin_max = 0.355 #
forearmlenghdin_min = 0.187 #

upperarmlenghtdin_max = 0.405 #
upperarmlenghtdin_min = 0.285 #

tcp_coversion = 0.2

savety_koord_1 = np.array([ 0.20,  0.0, 0.6])
savety_koord_2 = np.array([-0.24, -0.7, 0.04])

user = ""

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

        Wand_links = PoseStamped()
        Wand_links.header.frame_id = planning_frame 
        Wand_links.pose.position.x = -0.37
        Wand_links.pose.position.y = 0.00
        Wand_links.pose.position.z = 0.00 
        
        self.scene.add_box("Wand_links", Wand_links, size=(0.05, 3, 3))
        rospy.loginfo("Wand_links wurde Planungszene hinzugefügt")

        Wand_hinten = PoseStamped()
        Wand_hinten.header.frame_id = planning_frame 
        Wand_hinten.pose.position.x = 0.00
        Wand_hinten.pose.position.y = 0.34
        Wand_hinten.pose.position.z = 0.00 
        
        self.scene.add_box("Wand_hinten", Wand_hinten, size=(3, 0.05, 3))
        rospy.loginfo("Wand_hinten wurde Planungszene hinzugefügt")

        Decke = PoseStamped()
        Decke.header.frame_id = planning_frame  
        Decke.pose.position.x = 0.0
        Decke.pose.position.y = 0.0
        Decke.pose.position.z = 0.92 
        
        self.scene.add_box("Decke", Decke, size=(3, 2, 0.05))
        rospy.loginfo("Decke wurde Planungszene hinzugefügt.")

        Halter_Grundplatte = PoseStamped()
        Halter_Grundplatte.header.frame_id = planning_frame  
        Halter_Grundplatte.pose.position.x = 2*0.28
        Halter_Grundplatte.pose.position.y = -0.59
        Halter_Grundplatte.pose.position.z = -0.04
        
        self.scene.add_box("Halter_Grundplatte", Halter_Grundplatte, size=(0.60, 0.22, 0.22))
        rospy.loginfo("Halter_Grundplatte wurde Planungszene hinzugefügt.")

        eef_link = self.move_group.get_end_effector_link()
        rospy.loginfo("Endeffektor-Link: %s", eef_link)

        self.move_group.set_max_velocity_scaling_factor(0.1)
        self.move_group.set_max_acceleration_scaling_factor(0.1)

        while True:
            user = input('Gebe initialen ein: ')
            if (user == ""):
                user = "test"
            break
        

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
        
    def move_to_target_carth_plan(self, waypoints, speed):
        #Bewegt den Roboter in einer kartesischen Linie zur Zielpose
        self.move_group.set_max_velocity_scaling_factor(speed / 100.0)

        self.move_group.set_planning_time(10.0) 
        (plan, fraction) = self.move_group.compute_cartesian_path(waypoints, 0.05) 
        
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
                return False
        return True

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

         #TEST
        handover_pose_end.position.x = -0.4
        #TEST ende
        handover_pose_end =  self.point_inside(self.calc_handover_position_schoulder())



        handover_pose_start = copy.deepcopy(handover_pose_end)
        handover_pose_start.position.y = handover_pose_end.position.y + 0.1
        rospy.loginfo("Bewege Roboter zu: x={}, y={}, z={}".format(handover_pose_start.position.x, handover_pose_start.position.y, handover_pose_start.position.z))


        
        if not self.move_to_target_carth(handover_pose_start,speed):
            return False
        if not self.move_to_target_carth(handover_pose_end,speed):
            return False
        return True

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
        
        if point[0] < xmin or point[0] > xmax or point[1] < ymin or point[1]  > ymax or point[2] < zmin or point[2] >zmax:
            rospy.logwarn(f"Punkt liegt außerhalb")

        pose.position.x = max(xmin, min(point[0], xmax))
        pose.position.y = max(ymin, min(point[1], ymax))
        pose.position.z = max(zmin, min(point[2], zmax))

        return pose

    def pick_up(self,target):


        over_target = target.copy()
        over_target[2] = over_target[2] + 0.1  
        
        if not self.move_to_target(self.convert_to_pose(over_target), 5):
            return False
        
        if not self.gripper_controller.send_gripper_command('open'):
            return False

        if not self.move_to_target_carth(self.convert_to_pose(target), 10):
            return False
        
        if not self.gripper_controller.send_gripper_command('close'):
            return False

        if not self.move_to_target_carth(self.convert_to_pose(over_target), 5):
            return False
        
        return True

#======Gripper Control======

class GripperController:
    def __init__(self):
        #Initialisiert den Gripper-Controller
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        #rospy.Subscriber('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.status_callback)
        self.gripper_status = inputMsg.Robotiq2FGripper_robot_input()
        self.command = outputMsg.Robotiq2FGripper_robot_output()

    # def status_listener(self,cmd):
        
    #     rospy.loginfo(f'Gripper Status: {self.gripper_status}')
    #     if cmd == self.gripper_status:
    #         return True
    #     else:
    #         rospy.loginfo(f'Gripper Status weicht ab')
    #         return False

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
        #return self.status_listener(action_type)
        return True

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
        self.stop_event = threading.Event()
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
        with open('armlängen.csv', 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f'{user}{rospy.Time.now()} oberarmlänge:{self.uperarmlenght}'])
            writer.writerow([f'{user}{rospy.Time.now()} unterarmlänge:{self.uperarmlenght}'])

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

    def get_arm_angels(self):

        while not self.stop_event.is_set():
            self.camera_listener()

            shoulder = np.array([self.shoulderkoords[0],self.shoulderkoords[1],self.shoulderkoords[2]])
            elbow = np.array([self.elbowkoords[0],self.elbowkoords[1],self.elbowkoords[2]])
            hand = np.array([self.handkoords[0],self.handkoords[1],self.handkoords[2]])


            self.oberarmvec  = shoulder-elbow
            self.unterarmvec = elbow-hand

            elbowrad = np.arccos(np.dot(self.oberarmvec,self.unterarmvec)/ (np.sqrt((self.oberarmvec*self.oberarmvec).sum())*np.sqrt((self.unterarmvec*self.unterarmvec).sum())))
            elbowangle = elbowrad * 360 / 2 / np.pi
            print(elbowangle, end='\r') 
            with open('armlaengen.csv','a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([f'{user}{rospy.Time.now()} elbogenwinkel:{elbowangle}'])
            #return elbowangle

robot_control = RobotControl("manipulator")
    




################################ Initialisiere Smachstates ################################

class MPickUp(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded', 'succeeded_with_HD','aborted'])
        #self.robot_control = robot_control
        self.counter = 0
    def execute(self, userdata):
        #nehme Motor1 auf
        ### Kommentieren für testen
        if not robot_control.gripper_controller.send_gripper_command('activate'):
            return 'aborted'
        #return 'succeeded_with_HD'

        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        ''''DEBUG BLOCK ZUM TESTEN'''
        while True:
            newuser = input('enter y/n: ')
            if newuser == "y":

                if not robot_control.move_to_joint_goal( (-3.1557, -1.0119, -2.1765, -1.5426, 1.5686, -3.1643), 10):
                    return 'aborted'
                if not robot_control.gripper_controller.send_gripper_command('close'):
                    return 'aborted'
                if not robot_control.gripper_controller.send_gripper_command('open'):
                    return 'aborted'
                plan = []
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_m))
                if not robot_control.move_to_taget_plan(plan,10):
                    return 'aborted'
                if not robot_control.pick_up(rb_arm_on_m[15]):
                    return 'aborted'
                if not robot_control.move_to_joint_goal( (-3.8423, -1.0118, -2.3565, -2.8601, -0.7018, -3.1867), 20):
                    return 'aborted'
                self.counter += 1
                rospy.loginfo(f"Nehme Motor {self.counter} auf")
                return 'succeeded_with_HD'
            elif newuser == "n":
                rospy.loginfo('weiter')
                return 'succeeded_with_HD'

class MHold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
                
class MHoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_end','aborted'])
        self.hm = get_Hum_mertics()
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")

        newuser = input('enter y/n: ')
        if newuser == "y":
            if not robot_control.handover_to_hum(5):
                return 'aborted'
            self.hm.stop_event = threading.Event()
            thread = threading.Thread(target=self.hm.get_arm_angels)
            thread.start()
            input("Drücke Enter, um zu stoppen...\n")
            self.hm.stop_event.set()
            thread.join()

            if not robot_control.move_to_joint_goal( (-3.8423, -1.0118, -2.3565, -2.8601, -0.7018, -3.1867), 20):
                    return 'aborted'
            return 'succeeded'
        elif newuser == "n":
            rospy.loginfo('weiter')
            return 'succeeded'
        elif newuser == "a":
            rospy.loginfo('abort')
            return 'aborted'

class MPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_to_PCB','aborted'])
        self.robot_control = robot_control
        self.counter = 0
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")

        self.counter += 1
        if not (self.counter % 4==0):
            newuser = input('enter y/n: ')
            if newuser == "y":
                if not robot_control.move_to_target_carth(robot_control.convert_to_pose(rb_arm_transition_over_gb0_1),10):
                    return 'aborted'
                if not robot_control.gripper_controller.send_gripper_command('open'):
                    return'aborted'
                if not robot_control.move_to_target_carth(robot_control.convert_to_pose(rb_arm_transition_over_gb0_2),10):
                    return 'aborted'
                return 'succeeded_to_PCB'
            elif newuser == "n":
                rospy.loginfo('weiter')
                return 'succeeded_to_PCB'

        else:
            return 'succeeded_to_PCB'

class PCB1PickUpAndPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.counter = 0
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        while True:
            newuser = input('enter y/n: ')
            if newuser == "y":
                if not robot_control.move_to_joint_goal((-3.1299, -2.1996, -0.6071, -1.8830, 1.5654, -3.1786),10):
                    return 'aborted'
                if not robot_control.pick_up(rb_arm_on_pcb1[self.counter]):
                    return 'aborted'
                plan = []
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb1_1))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb1_2))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb1_3))
                if not robot_control.move_to_target_carth_plan(plan,10):
                    return 'aborted'
                if not robot_control.gripper_controller.send_gripper_command('open'):
                    return 'aborted'
                plan = []
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb1_2))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb1_1))
                if not robot_control.move_to_target_carth_plan(plan,10):
                    return 'aborted'
                self.counter +=1
                return 'succeeded'
            elif newuser == "n":
                print("weiter")
                return 'succeeded'        

class PCB2PickUpAndPositioning(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded','aborted'])
        self.counter = 0
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        while True:
            newuser = input('enter y/n: ')
            if newuser == "y":
                if not robot_control.move_to_joint_goal((-3.4437, -1.5349, -1.6576, -1.5354, 1.5145, -3.469),10):
                    return 'aborted'
                if not robot_control.pick_up(rb_arm_on_pcb2[self.counter]):
                    return 'aborted'
                plan = []
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb3_1))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb3_2))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb3_3))
                if not robot_control.move_to_target_carth_plan(plan,10):
                    return 'aborted' 
                if not robot_control.gripper_controller.send_gripper_command('open'):
                    return 'aborted'
                
                plan = []
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb3_2))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb3_1))

                if not robot_control.move_to_target_carth_plan(plan,10):
                    return 'aborted'
                
                self.counter +=1
                return 'succeeded'
            elif newuser == "n":
                print("Exiting")
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
        smach.State.__init__(self, outcomes=['succeeded','succeeded_end','aborted'])
        self.counter = 0
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        while True:
            newuser = input('enter y/n: ')
            if newuser == "y":
                if not robot_control.move_to_joint_goal((-2.8680, -1.9416, -1.1650, -1.6055, 1.5637, -1.3022),10):
                    return 'aborted' 
                if not robot_control.pick_up(rb_arm_on_battery[self.counter]):
                    return 'aborted' 
                plan = []
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb2_1))
                plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb2_2))
                #plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb2_3))
                if not robot_control.move_to_target_carth_plan(plan,10):
                    return 'aborted' 
                if not robot_control.gripper_controller.send_gripper_command('open'):
                    return 'aborted'
                # plan = []
                # plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb2_2))
                # plan.append(robot_control.convert_to_pose(rb_arm_transition_over_gb2_1))
                # if not robot_control.move_to_target_carth_plan(plan,10):
                #     return 'aborted' 
                # return 'succeeded'
            
            elif newuser == "n":
                print("Exiting")
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

class Aborted(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded_end','succeeded'])
    def execute(self, userdata):
        rospy.loginfo(f"Executing state: {self.__class__.__name__}")
        while True:
            newuser = input('neuer Versuch? y/n: ')
            if newuser == "y":
                return 'succeeded'
            elif newuser == "n":
                return 'succeeded_end'


if __name__ == "__main__":

    rospy.init_node('ur5_moveit_control', anonymous=True)
    moveit_commander.roscpp_initialize(sys.argv)

    robot_control.gripper_controller.send_gripper_command('activate')
    robot_control.gripper_controller.send_gripper_command('open')

    
    sm = smach.StateMachine(outcomes=['finished'])
    with sm:
        # Smachstates
        smach.StateMachine.add('MPickUp', MPickUp(),
                               transitions={'succeeded':'MHold',
                                            'aborted':'Aborted',
                                            'succeeded_with_HD':'MHoldHD'})
        smach.StateMachine.add('MHold', MHold(),
                               transitions={'succeeded':'MPositioning',
                                            'aborted':'Aborted'})
        smach.StateMachine.add('MHoldHD', MHoldHD(),
                               transitions={'succeeded':'MPositioning',
                                            'succeeded_end':'finished',
                                            'aborted':'Aborted'})
        smach.StateMachine.add('MPositioning', MPositioning(),
                               transitions={'succeeded':'MPickUp',
                                            'succeeded_to_PCB':'PCB1PickUpAndPositioning',
                                            'aborted':'Aborted'})
        smach.StateMachine.add('PCB1PickUpAndPositioning', PCB1PickUpAndPositioning(),
                               transitions={'succeeded':'PCB2PickUpAndPositioning',
                                            'aborted':'Aborted'})
        smach.StateMachine.add('PCB2PickUpAndPositioning', PCB2PickUpAndPositioning(),
                               transitions={'succeeded':'BatteryPositioning',
                                            'aborted':'Aborted'})
        smach.StateMachine.add('BatteryPositioning', BatteryPositioning(),
                               transitions={'succeeded_end':'finished',
                                            'aborted':'Aborted',
                                            'succeeded':'MPickUp'})
        smach.StateMachine.add('Aborted', Aborted(),
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

