#!/usr/bin/env python

import rospy
import yaml
from poi_manager.msg import *
from poi_manager.srv import *
from geometry_msgs.msg import Pose2D


def main():
    rospy.init_node('fake_updater')
    rospy.wait_for_service('/rbsherpa_hl_a/update_pois')
    update_pois = rospy.ServiceProxy('/rbsherpa_hl_a/update_pois', UpdatePOIs)

    pose_list = []

    pose = Pose2D(0.2, 3.7, 0.1)
    label = "lab1"
    pose2 = Pose2D(2.2, 4.5, 0.23)
    label2 = "lab2"

    pose_list.append(LabeledPose(label, pose))
    pose_list.append(LabeledPose(label2, pose2))

    print(pose_list)
    try:
      resp = update_pois(pose_list)
    except rospy.ServiceException as exc:
      print("Service did not process request: " + str(exc))
    rospy.spin()


if __name__ == "__main__":
	main()
