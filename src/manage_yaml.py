#!/usr/bin/env python

import rospy
import yaml
import rospkg
import tf
import os
from poi_manager.msg import *
from poi_manager.srv import *
from geometry_msgs.msg import Pose2D
from robotnik_msgs.msg import ptz
from std_msgs.msg import Empty
from visualization_msgs.msg import MarkerArray, Marker
from robotnik_msgs.srv import GetPOI, GetPOIResponse
from robotnik_msgs.srv import GetPTZ, GetPTZResponse

class ManageYAML:

    def __init__(self):
        rospack = rospkg.RosPack()
        self.filename = rospy.get_param('~filename', 'test')
        self.folder = rospy.get_param('~folder', os.path.join(rospack.get_path('poi_manager'), 'config'))
        self.yaml_path =  self.folder +'/'+ self.filename+'.yaml'
        self.pose_list = []
        self.pose_dict = {}
        self.service_read_yaml = rospy.Service('~read_pois', ReadPOIs, self.handle_labeled_pose_list)
        self.service_write_data = rospy.Service('~update_pois', UpdatePOIs, self.handle_updated_list)
        self.service_add_poi = rospy.Service('~add_poi', AddPOI , self.handle_add_poi)
        self.service_get_poi = rospy.Service('~get_poi', GetPOI, self.get_poi_cb)
        self.service_get_ptz = rospy.Service('~get_ptz', GetPTZ, self.get_ptz_cb)
        self.publish_markers = rospy.get_param('~publish_markers',False)
        self.publish_ptz_markers = rospy.get_param('~publish_ptz_markers',False)
        if self.publish_markers:
            self.marker_array = MarkerArray()
            self.marker_topic = rospy.get_param('~marker_topic', 'markers')
            self.frequency = rospy.get_param('~frequency', 0.5)
            self.frame_id = rospy.get_param('~frame_id', 'map')
            self.marker_array_publisher = rospy.Publisher(self.filename+'/'+self.marker_topic, MarkerArray, queue_size=10)

	rospy.loginfo('%s::_init_: config file path: %s',rospy.get_name(), self.yaml_path)

    def start(self):
        while not rospy.is_shutdown() and self.publish_markers:
            self.marker_array_publisher.publish(self.marker_array)
            rospy.sleep(1/self.frequency)
            
    def parse_yaml(self):
        try:
            f = open(self.yaml_path, 'r')
            self.pose_dict = {}
            self.pose_dict = yaml.safe_load(f)
            if self.pose_dict is None:
                self.pose_dict = {}
            f.close()
        except (IOError, yaml.YAMLError) as e:
        	rospy.logerr(e)
        	return 0

    def manage_read_data(self):
        self.pose_list = []
        for key, value in self.pose_dict.items():
            self.pose_list.append(LabeledPose(key, Pose2D(value['pose'][0], value['pose'][1], value['pose'][2]), ptz(value['ptz_pose'][0], value['ptz_pose'][1], value['ptz_pose'][2], False, "position")))

        if self.publish_markers:
            self.update_marker_array()


    def update_yaml(self, req):
        yaml_file = file(self.yaml_path, 'w')
        pose_list = req.pose_list
        self.pose_dict = {}
        for elem in pose_list:
            self.pose_dict[elem.label] = {}
            self.pose_dict[elem.label]['pose'] = [elem.pose.x, elem.pose.y, elem.pose.theta]
            self.pose_dict[elem.label]['ptz_pose'] = [elem.ptz_pose.pan, elem.ptz_pose.tilt, elem.ptz_pose.zoom]
        yaml.dump(self.pose_dict, yaml_file)

        if self.publish_markers:
            self.update_marker_array()


    def add_poi_to_yaml(self, req):
                
        new_poi = {
            req.pose.label : {
                'pose': [round(req.pose.pose.x, 4), round(req.pose.pose.y,4), round(req.pose.pose.theta,4)], 
                'ptz_pose': [round(req.pose.ptz_pose.pan,4), round(req.pose.ptz_pose.tilt,4) , req.pose.ptz_pose.zoom], 
            }
        }

        yaml_file = file(self.yaml_path, 'r')
        new_yaml_file = yaml.safe_load(yaml_file) 
        new_yaml_file.update(new_poi)

        yaml_file = file(self.yaml_path, 'w')
        yaml.safe_dump(new_yaml_file, yaml_file) 

        if self.publish_markers:
            self.parse_yaml()
            self.update_marker_array()


    def update_marker_array(self):
        marker_array = MarkerArray()
        for item in self.pose_dict.items():

            marker_arrow = self.create_marker(item, 'pose', 'arrow', 0)
            marker_text = self.create_marker(item, 'pose', 'text', 1)
            marker_array.markers.append(marker_arrow)
            marker_array.markers.append(marker_text)
            
            if self.publish_ptz_markers == True:
                marker_arrow_ptz = self.create_marker(item, 'ptz_pose', 'arrow', 2)
                marker_text_ptz = self.create_marker(item, 'ptz_pose', 'text', 3)
                marker_array.markers.append(marker_arrow_ptz)
                marker_array.markers.append(marker_text_ptz)  

        self.marker_array = marker_array
        self.marker_array_publisher.publish(self.marker_array)

    def create_marker(self, data, data_type, marker_type, marker_id):
     
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time()
        marker.ns = data[0]
        marker.action = Marker.ADD

        marker.lifetime = rospy.Duration(1/self.frequency)

        if data_type == 'pose':
            marker.pose.position.x = data[1]['pose'][0]
            marker.pose.position.y = data[1]['pose'][1]
            quaternion = tf.transformations.quaternion_from_euler(0, 0, data[1]['pose'][2])
            # TODO parameterize arrow color
            marker.color.a = 1.0 
            marker.color.g = 1.0

        if data_type == 'ptz_pose':
            marker.pose.position.x = data[1]['pose'][0]
            marker.pose.position.y = data[1]['pose'][1]
            marker.pose.position.z = 1
            quaternion = tf.transformations.quaternion_from_euler(0, data[1]['ptz_pose'][0], data[1]['ptz_pose'][1])
            # TODO parameterize arrow color
            marker.color.a = 1.0 
            marker.color.r = 1.0
            marker.color.b = 1.0

        marker.pose.orientation.x = quaternion[0]
        marker.pose.orientation.y = quaternion[1]
        marker.pose.orientation.z = quaternion[2]
        marker.pose.orientation.w = quaternion[3]


        if marker_type == 'arrow':
            marker.id = marker_id
            marker.type = Marker.ARROW
            marker.scale.x = 0.5
            marker.scale.y = 0.05
            marker.scale.z = 0.05
 
        else:
            marker.id = marker_id
            marker.type = Marker.TEXT_VIEW_FACING
            marker.scale.z = 0.2
            marker.color.r = 1.0
            marker.color.b = 1.0 
            marker.color.g = 1.0 
            if data_type == 'pose':
                # marker.text = data[0] + "\n" + "x: {:.2f} ".format(data[1]['pose'][0])  + "y: {:.2f} ".format(data[1]['pose'][1])  + "a: {:.2f} ".format(data[1]['pose'][2])
                if self.publish_ptz_markers == False:
                    marker.text = data[0]
                marker.pose.position.z = 0.5

            if data_type == 'ptz_pose':
                # marker.text = data[0] + "\n" + "pan: {:.2f} ".format(data[1]['ptz_pose'][0])  + "tilt: {:.2f} ".format(data[1]['ptz_pose'][1])  + "zoom: " + str(data[1]['pose'][2])
                marker.text = data[0]  + "\n" + "zoom: " + str(data[1]['pose'][2])
                marker.pose.position.z = 1.5
     

        return marker

    def handle_labeled_pose_list(self, req):
        self.parse_yaml()
        self.manage_read_data()
        rospy.loginfo("%s::handle_labeled_pose_list: read_pois service done", rospy.get_name())
        return ReadPOIsResponse(self.pose_list)

    def handle_updated_list(self, req):
        self.update_yaml(req)
        rospy.loginfo("%s::handle_updated_list: update_pois service done", rospy.get_name())
        return UpdatePOIsResponse()
    
    def handle_add_poi(self, req):
        self.add_poi_to_yaml(req)
        rospy.loginfo("%s::handle_add_poi: add_poi service done", rospy.get_name())
        return AddPOIResponse()

    def get_poi_cb(self, req):
        name = req.name
        response = GetPOIResponse()
        if len(self.pose_list) > 0:
            for poi in self.pose_list:
                if poi.label == name:
                    response.success = True
                    response.pose = poi.pose
                    return response
        else:
            response.success = False
        
        return response

    def get_ptz_cb(self, req):
        name = req.name
        response = GetPTZResponse()
        if len(self.pose_list) > 0:
            for ptz in self.pose_list:
                if ptz.label == name:
                    response.success = True
                    response.pan = ptz.ptz_pose.pan
                    response.tilt = ptz.ptz_pose.tilt
                    response.zoom = ptz.ptz_pose.zoom
                    # pose_list sets relative param to false by default
                    response.relative = ptz.ptz_pose.relative
                    return response
        else:
            response.success = False

        return response

def main():
    rospy.init_node('manage_yaml')
    yaml_manager = ManageYAML()
    yaml_manager.start()
    rospy.loginfo("%s::main: spin", rospy.get_name())
    rospy.spin()


if __name__ == "__main__":
	main()
