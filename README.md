# poi_manager

## Installation

To use this package you will need to have the following Robotnik packages installed:

- robotnik_msgs [🔗](https://www.github.com/RobotnikAutomation/robotnik_msgs)

## 1 poi_manager

A ROS node to manage the points of interest in a map. It reads a list of tagged positions from a YAML file and offers services to obtain the list and update it. 

### 1.1 Parameters

* ~**filename** (String, default: "test"): Name of the file that contains the saved POIs.
   
* ~**folder** (String, default: "/path/to/poi_manager/config"): Folder location of the file that contains the saved POIs.
   
* ~**publish_markers** (Bool, default: False): Flag to enable the publication of the POIs as [visualization_msgs/MarkerArray](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html)

* ~**marker_topic** (String, default: "markers"): Topic name where markers are published. Only used if publish_markers is true.

* ~**frequency** (Float, default: 0.5): Markers publication frequency. Only used if publish_markers is true.

* ~**frame_id** (String, default: "map"): Frame id at which the markers are published. Only used if publish_markers is true 

### 1.2 Subscribed Topics

poi_manager does not subscirbe to any topic.

### 1.3 Published Topics

* **poi_manager/markers** (visualization_msgs/MarkerArray):  POIs published as MarkerArray to visualize them in RVIZ

### 1.4 Services
* **poi_manager/read_pois** ([poi_manager/ReadPOIs](https://github.com/RobotnikAutomation/poi_manager/blob/master/srv/ReadPOIs.srv)): Loads the file where the POIs are stored.

* **poi_manager/update_pois** ([poi_manager/UpdatePOIs](https://github.com/RobotnikAutomation/poi_manager/blob/master/srv/UpdatePOIs.srv)): Saves a list of [poi_manager/LabeledPose](https://github.com/RobotnikAutomation/poi_manager/blob/master/msg/LabeledPose.msg) as POIs in the config file.

* **poi_manager/getPOI** ([robotnik_msgs/GetPOI](https://github.com/RobotnikAutomation/robotnik_msgs/blob/master/srv/GetPOI.srv)): Given a POI name, returns its pose as [geometry_msgs/Pose2D](http://docs.ros.org/en/api/geometry_msgs/html/msg/Pose2D.html).

### 1.5 Services Called

poi_manager does not call to any service server.

### 1.6 Action server

poi_manager does not implement any action server.

### 1.7 Action clients called

poi_manager does not call to any action.

### 1.8 Required tf Transforms

poi_manager does not require any tf.

### 1.9 Provided tf Transforms

poi_manager does not provide any tf.

### 1.10 Bringup

To launch the node:

```bash
roslaunch poi_manager poi_manager.launch id_robot:=robot
```
