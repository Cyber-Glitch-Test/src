#!/usr/bin/env python

import rospy
import roslaunch
from controller_manager_msgs.srv import ListControllers

def wait_for_controller(controller_name="scaled_pos_joint_traj_controller"):
    rospy.loginfo("Warte auf aktiven Controller: %s", controller_name)
    rospy.wait_for_service("/controller_manager/list_controllers")
    list_controllers = rospy.ServiceProxy("/controller_manager/list_controllers", ListControllers)
    while not rospy.is_shutdown():
        try:
            resp = list_controllers()
            for c in resp.controller:
                if c.name == controller_name and c.state == "running":
                    rospy.loginfo("Controller %s ist aktiv.", controller_name)
                    return
        except rospy.ServiceException:
            pass
        rospy.sleep(1.0)

def main():
    rospy.init_node("wait_and_launch_moveit", anonymous=True)
    wait_for_controller()

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)
    moveit_launch = roslaunch.parent.ROSLaunchParent(
        uuid,
        [rospy.get_param("~moveit_launch_file")]
    )
    moveit_launch.start()
    rospy.spin()

if __name__ == '__main__':
    main()
