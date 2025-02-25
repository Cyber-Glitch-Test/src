#!/usr/bin/env python

import numpy as np
import csv
import rospy
import moveit_commander
import sys
import smach
import smach_ros
import rtde_control
from geometry_msgs.msg import Pose
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

height_hum_shoulder = 1.8
Hum_det = True
savety_koord_1 = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335])
savety_koord_2 = np.array([-0.29132828185820775, 0.08159780929922979, 0.3055465140144335])

#======Gripper Control======

class GripperController:
    def __init__(self):

        
        # ROS Publisher für das Senden von Befehlen an den Gripper
        self.pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output, queue_size=10)
        
        # ROS Subscriber für den Empfang von Statusinformationen des Grippers
        rospy.Subscriber('Robotiq2FGripperRobotInput', inputMsg.Robotiq2FGripper_robot_input, self.status_callback)
        
        # Initialisierung des ROS-Knotens
        #rospy.init_node('gripper_control_node')
        
        # Gripper-Status und zuletzt gesendeter Befehl
        self.gripper_status = inputMsg.Robotiq2FGripper_robot_input()
        self.command = outputMsg.Robotiq2FGripper_robot_output()

    def status_callback(self, msg):
        """Callback-Funktion, um den Status des Grippers zu empfangen."""
        self.gripper_status = msg
        #rospy.loginfo("Aktueller Gripper-Status: %s", msg)

    def send_gripper_command(self, action_type):
        """Sende einen Befehl an den Gripper (öffnen, schließen, etc.)."""
        if action_type == 'open':
            self.command.rPR = 0  # Öffne den Gripper
        elif action_type == 'close':
            self.command.rPR = 255    # Schließe den Gripper
        elif action_type == 'activate':
            self.command.rACT = 1
            self.command.rGTO = 1
            self.command.rSP = 255
            self.command.rFR = 150
        elif action_type == 'deactivate':
            self.command.rACT = 0   # Deaktiviere den Gripper
        rospy.sleep(2)
        # Sende den Befehl an den Gripper
        rospy.loginfo(f"Sende Befehl: {action_type}")
        self.pub.publish(self.command)

#======Robot Control======

def Convert_to_Pose(koords):
    # Konvertiere koords in Pose
    target_pose = Pose()
    target_pose.position.x = koords[0]
    target_pose.position.y = koords[1]
    target_pose.position.z = koords[2]
    target_pose.orientation.x = koords[3]
    target_pose.orientation.y = koords[4]
    target_pose.orientation.z = koords[5]
    target_pose.orientation.w = koords[6]
    return target_pose
    

def move_to_target(move_group, target_pose,speed):
    set_speed(speed)
    move_group.set_pose_target(target_pose)
    rospy.loginfo("Bewege Roboter zu: x={}, y={}, z={}".format(target_pose.position.x, target_pose.position.y, target_pose.position.z))

    success = move_group.go(wait=True)
    
    if success:
        rospy.loginfo("Bewegung erfolgreich!")
        return True
    else:
        rospy.logwarn("Bewegung fehlgeschlagen!")
        move_group.stop()
        move_group.clear_pose_targets()
        return False


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
    move_group.set_max_velocity_scaling_factor(0.1)      # Geschwindigkeit 10% der maximalen Geschwindigkeit
    move_group.set_max_acceleration_scaling_factor(0.1)  # Beschleunigung 10% der maximalen Beschleunigung

    # Referenzrahmen
    planning_frame = move_group.get_planning_frame()
    rospy.loginfo("Planungsrahmen: %s", planning_frame)

    # **Hier: Planungsschnittstelle (Scene) erstellen**
    scene = PlanningSceneInterface()
    rospy.sleep(2)  # Kurze Pause, damit die Szene initialisiert wird

    # Box-Position definieren
    p = PoseStamped()
    p.header.frame_id = planning_frame  # Setze den Planungsrahmen als Referenz
    p.pose.position.x = 0.0
    p.pose.position.y = 0.0
    p.pose.position.z = -0.09  # Etwas unter dem Boden

    # Box zur Szene hinzufügen
    scene.add_box("Tisch", p, size=(3, 2, 0.05))

    rospy.loginfo("Box wurde hinzugefügt.")

    # Zielrahmen für den Endeffektor
    eef_link = move_group.get_end_effector_link()
    rospy.loginfo("Endeffektor-Link: %s", eef_link)

    # Shutdown von MoveIt und ROS-Verbindungen
    # moveit_commander.roscpp_shutdown()



def point_inside(point):   
    xmin, xmax = savety_koord_1[0]-1, savety_koord_2[0]+1
    yield xmin < point[0] < xmax
    ymin, ymax = savety_koord_1[1]-1, savety_koord_2[1]+1
    yield ymin < point[1] < ymax
    zmin, zmax = savety_koord_1[2]-1, savety_koord_2[2]+1
    yield zmin < point[2] < zmax
    rect = (savety_koord_1, savety_koord_2)


def set_speed(speed):
    try:
        rtde_c = rtde_control.RTDEControlInterface("192.168.0.100")
        rtde_c.sendsendSpeedSlider(speed) 
    except Exception as e:
        rospy.logwarn("RTDE-Verbindung fehlgeschlagen.Fehler: %s", e)



################################ Initialisiere Smachstates ################################

class M1PickUp(smach.State):
    def __init__(self,group_name):
        smach.State.__init__(self, outcomes=['succeeded','succeeded_with_HD'])
        self.group_name = group_name
        self.robot = moveit_commander.RobotCommander()  # Objekt für den Roboter
        self.scene = moveit_commander.PlanningSceneInterface()  # Szene
        self.group = MoveGroupCommander(self.group_name)
        self.gripper_controller = GripperController()

    def execute(self, userdata):

        rospy.loginfo('Executing state: M1PickUp')
        
        if not move_to_target(self.group, Convert_to_Pose(rb_arm_home),5):
            return 'succeeded'  # Oder 'aborted'

        rospy.loginfo("bereit für M1 aufnehmen")
        self.gripper_controller.send_gripper_command('activate')
        self.gripper_controller.send_gripper_command('close')
        self.gripper_controller.send_gripper_command('open')
        
        rospy.loginfo("Zweite Bewegung...")
        if not move_to_target(self.group, Convert_to_Pose(rb_arm_over_m1),10):
            return 'succeeded'  # Oder 'aborted'

        
        rospy.loginfo("Dritte Bewegung...")

        if not move_to_target(self.group, Convert_to_Pose(rb_arm_on_m1),5):
            return 'succeeded'  # Oder 'aborted'
        self.gripper_controller.send_gripper_command('close')
        
        
        if not move_to_target(self.group, Convert_to_Pose(rb_arm_over_m1),10):
            return 'succeeded'  # Oder 'aborted'
        return 'succeeded'
    
        if not move_to_target(self.group, Convert_to_Pose(rb_arm_on_hum_static),10):
            return 'succeeded'  # Oder 'aborted'
        return 'succeeded'

class M1Hold(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M1Hold')
        ####
        return 'succeeded'
    
class M1HoldHD(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: M1Hold')
        ####
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

class PCB3Fixing(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['succeeded'])

    def execute(self, userdata):
        rospy.loginfo('Executing state: PCB3Fixing')
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


def tracking_listener():

    topic = rospy.get_param('~shoulder', 'chatter')
    rospy.Subscriber(topic, node_example_data, callback)


if __name__ == "__main__":
    try:
        # Starte die ROS-Node
        moveit_control_node()
    except rospy.ROSInterruptException:
        pass
    
    

    sm = smach.StateMachine(outcomes=['finished'])

    moveit_commander.roscpp_initialize(sys.argv)

    # Starte die UR5 Control Node
    rospy.init_node('ur5_moveit_control', anonymous=True)

    # Starte die Robotiq Gripper Control Node



    group_name = "manipulator" 
    with sm:
        # Smachstates
        smach.StateMachine.add('M1PickUp', M1PickUp(group_name),
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
                               transitions={'succeeded':'PCB3Fixing'})
        smach.StateMachine.add('PCB3Fixing', PCB3Fixing(),
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

