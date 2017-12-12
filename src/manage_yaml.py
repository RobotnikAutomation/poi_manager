#!/usr/bin/env python

import rospy
import yaml
import rospkg
from ros_pose_reader.msg import *
from ros_pose_reader.srv import *
from geometry_msgs.msg import Pose2D

class ManageYAML:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('ros_pose_reader') + '/test2.yaml'
        self.pose_list = []
        self.pose_dict = {}

    def parse_yaml(self):
        try:
            f = open(self.yaml_path, 'r')
            self.pose_dict = yaml.safe_load(f)
            if self.pose_dict is None:
                self.pose_dict = {}
            f.close()
        except (IOError, yaml.YAMLError) as e:
        	rospy.logerr(e)
        	return 0

    def manage_read_data(self):
        for key, value in self.pose_dict.items():
            self.pose_list.append(LabeledPose(key, Pose2D(value[0], value[1], value[2])))

    def update_yaml(self, req):
        yaml_file = file(self.yaml_path, 'w')
        pose_list = req.pose_list
        for elem in pose_list:
            self.pose_dict[elem.label] = [elem.pose.x, elem.pose.y, elem.pose.theta]
        yaml.dump(self.pose_dict, yaml_file)


def handle_labeled_pose_list(req):
    yaml_manager = ManageYAML()
    yaml_manager.parse_yaml()
    yaml_manager.manage_read_data()
    return LabeledPoseListResponse(yaml_reader.pose_list)

def handle_updated_list(req):
    yaml_manager = ManageYAML()
    yaml_manager.update_yaml(req)


def main():
    rospy.init_node('manage_yaml')
    service_read_yaml = rospy.Service('manage_yaml/read_data', ReadYaml, handle_labeled_pose_list)
    service_write_data = rospy.Service('manage_yaml/update_data', UpdateYaml, handle_updated_list)
    print "(TODO: Change) Publishing pose list"
    rospy.spin()


if __name__ == "__main__":
	main()
