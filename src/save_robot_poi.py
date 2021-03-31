#!/usr/bin/env python

import rospy
import yaml
import rospkg
import os
from geometry_msgs.msg import Pose2D
from robotnik_msgs.msg import ptz
from poi_manager.msg import *
from poi_manager.srv import *
from robotnik_msgs.srv import SetString, SetStringResponse
import tf
from tf.transformations import euler_from_quaternion
from tf import TransformListener
import math 


class SaveRobotPOI:

    def __init__(self):

        rospack = rospkg.RosPack()
        self.listener = tf.TransformListener()
        self.filename = rospy.get_param('~filename', 'test')
        self.folder = rospy.get_param('~folder', os.path.join(rospack.get_path('poi_manager'), 'config'))
        self.yaml_path =  self.folder +'/'+ self.filename+'.yaml'
        self.global_frame = rospy.get_param('~global_frame', 'map')
        self.base_frame = rospy.get_param('~base_frame', 'base_link')
        self.ptz_frame = rospy.get_param('~ptz_frame', 'ptz_base_link')

        self.service_get_poi = rospy.Service('poi_manager/save_robot_poi', SetString , self.set_poi_cb)
        
        self.add_poi_client = rospy.ServiceProxy('poi_manager/add_poi', AddPOI)


	rospy.loginfo('%s::_init_: config file path: %s',rospy.get_name(), self.yaml_path)


    def get_robot_pose(self):

        robot_pose = Pose2D()

        self.listener.waitForTransform(self.global_frame, self.base_frame, rospy.Time(), rospy.Duration(4.0))
        (position,orientation) = self.listener.lookupTransform(self.global_frame, self.base_frame, rospy.Time(0))

        euler = euler_from_quaternion(orientation)

        robot_pose.x = position[0]
        robot_pose.y = position[1]
        robot_pose.theta = euler[2] # yaw

        # rospy.logdebug("Robot current position is: " + str(robot_pose))

        return robot_pose


    def get_ptz_pose(self):

        ptz_pose = ptz()

        self.listener.waitForTransform(self.base_frame, self.ptz_frame, rospy.Time(), rospy.Duration(4.0))
        (position,orientation) = self.listener.lookupTransform(self.base_frame, self.ptz_frame, rospy.Time(0))

        euler = euler_from_quaternion(orientation)

        ptz_pose.pan = euler[2]
        ptz_pose.tilt = euler[1]
        ptz_pose.zoom = euler[0]
        ptz_pose.relative = False

        # rospy.loginfo("PTZ current position is: " + str(ptz_pose))

        return ptz_pose

    def set_poi_cb(self, req):
      
        name = req.data
        response = SetStringResponse()
        
        robot_pose = self.get_robot_pose()
        ptz_pose = self.get_ptz_pose()

        poi = AddPOIRequest()
        poi.pose.label = req.data
        poi.pose.pose.x = robot_pose.x
        poi.pose.pose.y = robot_pose.y
        poi.pose.pose.theta = robot_pose.theta
        poi.pose.ptz_pose.pan = ptz_pose.pan
        poi.pose.ptz_pose.tilt = ptz_pose.tilt
        poi.pose.ptz_pose.zoom = ptz_pose.zoom
        poi.pose.ptz_pose.relative = ptz_pose.relative
        
        message = "Robot [X: "  + str(robot_pose.x) +  " Y: " + str(robot_pose.y) + " Theta: " + str(robot_pose.theta) + "]" + \
                  " PTZ [Pan: " + str(ptz_pose.pan) + " Tilt:" + str(ptz_pose.tilt) + " Zoom: " + str(ptz_pose.zoom) + "]"
        
        res = self.add_poi_client(poi)

        response.ret.message = message
        response.ret.success = True

        return response


def main():
    rospy.init_node('save_robot_poi')
    save_robot_poi = SaveRobotPOI()
    rospy.loginfo("%s::main: spin", rospy.get_name())
    rospy.spin()


if __name__ == "__main__":
	main()

