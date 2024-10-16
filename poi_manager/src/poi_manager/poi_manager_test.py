#!/usr/bin/env python

import rospy
import yaml
import os
import sys
from std_srvs.srv import SetBool
from poi_manager_msgs.srv import AddPOI_params, DeletePOI 
from poi_manager_msgs.srv import AddPOIJoints
from poi_manager_msgs.msg import PoiJointState

class POIManagerClient:
    def __init__(self):
        # Check if a file path was passed as an argument
        if len(sys.argv) < 2:
            rospy.logerr("Usage: script.py <path_to_yaml>")
            sys.exit(1)

        # Get the YAML file path from the command-line argument
        # self.yaml_file_path = "home/.config/robot/inspection/config/navigation/poi_manager/poi.yaml"
        self.yaml_file_path = sys.argv[1]
        rospy.init_node('poi_manager_test')

        # Define the service names
        self.add_poi_service = '/robot/poi_interactive_marker/add_poi'
        self.delete_poi_service = '/robot/poi_interactive_marker/delete_poi'
        self.add_poi_and_joints_service = '/robot/poi_interactive_marker/add_poi_and_joints'
        self.delete_all_pois_service = '/robot/poi_interactive_marker/delete_all_pois'

        # Load YAML path
        # self.yaml_path = rospy.get_param('~yaml_path', '/path/to/poi_file.yaml')  # Set this parameter as needed

        # Wait for the services to become available
        rospy.wait_for_service(self.add_poi_service)
        rospy.wait_for_service(self.delete_poi_service)
        rospy.wait_for_service(self.add_poi_and_joints_service)
        rospy.wait_for_service(self.delete_all_pois_service)

        rospy.loginfo("All services are available!")

    def call_add_poi(self, poi_name, environment, frame_id, params, x, y, z, roll, pitch, yaw):
        """ Calls the service to add a POI """
        try:
            add_poi = rospy.ServiceProxy(self.add_poi_service, AddPOI_params)
            res = add_poi(poi_name, environment, frame_id, params, x, y, z, roll, pitch, yaw)
            rospy.loginfo(f"Add POI Response: {res.success}, {res.message}")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def call_delete_poi(self, poi_name, environment):
        """ Calls the service to delete a POI """
        try:
            delete_poi = rospy.ServiceProxy(self.delete_poi_service, DeletePOI)
            res = delete_poi(poi_name , environment)
            rospy.loginfo(f"Delete POI Response: {res.success}, {res.message}")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def call_add_poi_and_joints(self, poi_name, environment, frame_id, params, x, y, z, roll, pitch, yaw, joint_state):
        """ Calls the service to add a POI with joints """
        try:
            add_poi_and_joints = rospy.ServiceProxy(self.add_poi_and_joints_service, AddPOIJoints)
            res = add_poi_and_joints(poi_name, environment, frame_id, params, x, y, z, roll, pitch, yaw, joint_state)
            rospy.loginfo(f"Add POI with Joints Response: {res.success}, {res.message}")
            return res.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def call_delete_all_pois(self):
        """ Calls the service to delete all POIs """
        try:
            delete_all_pois = rospy.ServiceProxy(self.delete_all_pois_service, SetBool)
            data = True
            delete_all_pois(data)
            rospy.loginfo("All POIs deleted")
            return True
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
            return False

    def check_poi_in_yaml(self, poi_name):
        """ Checks if a POI is in the YAML file """
        if not os.path.exists(self.yaml_file_path):
            rospy.logerr(f"YAML file {self.yaml_file_path} not found!")
            return False

        with open(self.yaml_file_path, 'r') as yaml_file:
            try:
                poi_data = yaml.safe_load(yaml_file)
                if poi_data is not None:
                    for env, env_data in poi_data.get('environments', {}).items():
                        if poi_name in env_data.get('points', {}):
                            rospy.loginfo(f"POI '{poi_name}' found in environment '{env}'")
                            return True
                rospy.loginfo(f"POI '{poi_name}' not found in .yaml file")
                return False
            except yaml.YAMLError as e:
                rospy.logerr(f"Error parsing YAML file: {e}")
                return False

def main():
    client = POIManagerClient()

    poi_name_first = 'test_poi1'
    poi_name_second = 'test_poi2'
    environment = '240930_LAB_3D'

    frame_id = "robot_map"
    params = ""
    x = 0.5
    y = 0.3
    z = 0.2
    roll = 0.0
    pitch = 0.0
    yaw = 1

    poi1 = PoiJointState()
    poi1.name = "joint_1"
    poi1.position = 1.0

    poi2 = PoiJointState()
    poi2.name = "joint_2"
    poi2.position = 2.0

    poi_joint_states = []
    poi_joint_states.append(poi1)
    poi_joint_states.append(poi2)

    # Add a POI
    if client.call_add_poi(poi_name_first, environment, frame_id, params, x, y, z, roll, pitch, yaw):
        rospy.sleep(1)  # Give time for the file to update
        client.check_poi_in_yaml(poi_name_first)

    # Delete the POI
    if client.call_delete_poi(poi_name_first, environment):
        rospy.sleep(1)
        client.check_poi_in_yaml(poi_name_first)

    # # Add a POI
    if client.call_add_poi(poi_name_first, environment, frame_id, params, x, y, z, roll, pitch, yaw):
        rospy.sleep(1)  # Give time for the file to update
        client.check_poi_in_yaml(poi_name_first)

    # # Add POI with joints
    if client.call_add_poi_and_joints(poi_name_second, environment, frame_id, params, x, y, z, roll, pitch, yaw, poi_joint_states):
        rospy.sleep(1)
        client.check_poi_in_yaml(poi_name_second)

    # Delete all POIs
    if client.call_delete_all_pois():
        rospy.sleep(1)
        client.check_poi_in_yaml(poi_name_first)
        client.check_poi_in_yaml(poi_name_second)


if __name__ == "__main__":
    main()
