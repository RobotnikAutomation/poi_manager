#!/usr/bin/env python

import rospy
import yaml
import rospkg
from ros_pose_reader.msg import *
from ros_pose_reader.srv import *
from geometry_msgs.msg import Pose2D

class ReadYAML:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('ros_pose_reader') + '/test.yaml'
        self.pose_list = []
        self.test_dict = {}

    def parse_yaml(self):
        try:
            f = open(self.yaml_path, 'r')
            self.test_dict = yaml.safe_load(f)
            if self.test_dict is None:
                self.test_dict = {}
            f.close()
        except IOError as e:
        	rospy.logerr(e)
        	return -1
        except yaml.YAMLError as e:
            rospy.logerr(e)
            return -1

    def manage_data(self):
        for key, value in self.test_dict.items():
            self.pose_list.append(LabeledPose(key, Pose2D(value[0], value[1], value[2])))

def handle_labeled_pose_list(req):
    yaml_reader = ReadYAML()
    yaml_reader.parse_yaml()
    yaml_reader.manage_data()
    return LabeledPoseListResponse(yaml_reader.pose_list)

def main():
    rospy.init_node('read_yaml')
    service = rospy.Service('labeled_pose_list', LabeledPoseList, handle_labeled_pose_list)
    print "Publishing pose list"
    rospy.spin()


if __name__ == "__main__":
	main()
