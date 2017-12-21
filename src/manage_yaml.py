#!/usr/bin/env python

import rospy
import yaml
import rospkg
from poi_manager.msg import *
from poi_manager.srv import *
from geometry_msgs.msg import Pose2D
from std_msgs.msg import Empty

class ManageYAML:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.yaml_path = rospack.get_path('poi_manager') + '/test.yaml'
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
    print "read_pois service done"
    return ReadPOIsResponse(yaml_manager.pose_list)

def handle_updated_list(req):
    yaml_manager = ManageYAML()
    yaml_manager.update_yaml(req)
    print "update_pois service done"
    return UpdatePOIsResponse()


def main():
    rospy.init_node('manage_yaml')
    service_read_yaml = rospy.Service('read_pois', ReadPOIs, handle_labeled_pose_list)
    service_write_data = rospy.Service('update_pois', UpdatePOIs, handle_updated_list)
    print "Node running."
    rospy.spin()


if __name__ == "__main__":
	main()
