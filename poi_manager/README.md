# POI Manager

POI Manager is a ROS package to create, edit, and persist Points Of Interest (POIs) on a map, and to interactively send navigation goals through RViz.

This document covers features, configuration, and the newly added interaction improvements and parameters.

## Installation

To use this package you will need to have the following Robotnik packages installed:

- robotnik_msgs [🔗](https://www.github.com/RobotnikAutomation/robotnik_msgs)

## 1 poi_manager

A ROS node to manage the points of interest in a map. It reads a list of tagged positions from a YAML file and offers services to obtain the list and update it.

### 1.1 Parameters

* ~**filename** (String, default: "test"): Name of the file that contains the saved POIs.
   
* ~**folder** (String, default: "/path/to/poi_manager/config"): Folder location of the file that contains the saved POIs.
   
* ~**yaml_backup_folder** (String, default: "folder/backup"): Folder location of the backups that are created every time we modify the config file.

* ~**publish_markers** (Bool, default: False): Flag to enable the publication of the POIs as [visualization_msgs/MarkerArray](http://docs.ros.org/en/api/visualization_msgs/html/msg/MarkerArray.html)


### 1.2 Subscribed Topics

poi_manager does not subscirbe to any topic.

### 1.3 Published Topics

* **poi_manager/markers** (visualization_msgs/MarkerArray):  POIs published as MarkerArray to visualize them in RVIZ

* **poi_manager/state** (robotnik_msgs/State):  The state of the component

### 1.4 Services

* **poi_manager/add_poi**: Adds a new POI to the list.

* **poi_manager/add_poi_by_params**: Adds a new POI using parameters.

* **poi_manager/add_pois**: Adds multiple POIs at once.

* **poi_manager/delete_environment**: Deletes a specified environment.

* **poi_manager/delete_poi**: Deletes a POI from the list.

* **poi_manager/get_environments**: Retrieves the list of available environments.

* **poi_manager/get_poi**: Retrieves a specific POI.

* **poi_manager/get_poi_list**: Retrieves the list of all POIs.

* **poi_manager/get_poi_params**: Gets the parameters of a specific POI.

* **poi_manager/read_pois**: Loads the file where the POIs are stored.

* **poi_manager/recover_last_backup**: Recovers the last backup of the POIs configuration file.

### 1.5 Services Called

poi_manager does not call to any service server.

### 1.6 Action server

No

### 1.7 Action clients called

No

### 1.8 Required tf Transforms

No

### 1.9 Provided tf Transforms

No 

### 1.10 Bringup

No

## 2 poi_interactive_marker

A ROS node that provides interactive markers for managing POIs in RViz. It allows users to add, move, and delete POIs interactively.

### 2.1 Parameters

* ~**frame_id** (String, default: "robot_map"): Reference frame for the interactive markers.
* ~**marker_scale_x** (Double, default: 0.5): X Scale of the interactive markers.
* ~**marker_scale_y** (Double, default: 0.15): Y Scale of the interactive markers.
* ~**marker_scale_z** (Double, default: 0.15): Z Scale of the interactive markers.
* ~**drag_pad_extra_scale** (Double, default: 1.2): Scale factor applied to a thin cylinder under the arrow used as a large drag area. Also increases the global interactive marker scale so the default RViz rotation ring is easier to grab.
* ~**use_drag_pad_helper** (Bool, default: true): Enable/disable the larger drag pad helper.
* ~**drag_pad_color** (Array[4], default: [0.2, 0.2, 0.2, 0.30]): RGBA color for the drag pad cylinder. Low alpha recommended to keep it discreet yet clickable.
* ~**rotate_handle_radius** (Double, default: 1.25): Radius of the rotation helper cylinder used to increase the rotation interaction area.
* ~**rotate_handle_thickness** (Double, default: 0.01): Thickness (height) of the rotation helper cylinder.
* ~**use_rotation_helper** (Bool, default: true): Enable/disable the rotation helper pad.
* ~**rotate_pad_color** (Array[4], default: [0.6, 0.1, 0.9, 0.35]): RGBA color for the rotation pad cylinder. Use a vivid color and moderate alpha for high contrast.
* ~**base_frame_id** (String, default: "robot_base_footprint"): Base frame of the robot.
* ~**goto_planner** (String, default: "mb_avoidance/move_base"): Name of the planner used for navigation.
* ~**use_command_manager_goto** (Bool, default: False): Whether to use the command manager for GOTO actions.
* ~**use_rms_goto** (Bool, default: False): Whether to use RMS for GOTO actions.
* ~**rms_manager_action_name** (String, default: "rms/action"): Action name for the RMS manager.
* ~**command_manager_action_name** (String, default: "command_manager/action"): Action name for the command manager.
* ~**init_pose_topic_name** (String, default: "initialpose"): Topic name for the initial pose.
* ~**load_pois_service_name** (String, default: "poi_manager/get_poi_list"): Service name to load the list of POIs.
* ~**get_poi_service_name** (String, default: "poi_manager/get_poi"): Service name to get a specific POI.
* ~**add_poi_service_name** (String, default: "poi_manager/add_poi"): Service name to add a new POI.
* ~**add_poi_params_service_name** (String, default: "poi_manager/add_poi_by_params"): Service name to add a POI by parameters.
* ~**delete_poi_service_name** (String, default: "poi_manager/delete_poi"): Service name to delete a POI.
* ~**delete_all_pois_service_name** (String, default: "poi_manager/delete_environment"): Service name to delete all POIs in an environment.
* ~**rlc_localization_status_topic_name** (String, default: "robot_local_control/LocalizationComponent/status"): Topic name for localization status.
* ~**command_manager_goto_command** (String, default: "GOTO"): Command string for GOTO actions.

### 2.2 Subscribed Topics

* **joint_states** (`sensor_msgs/JointState`)
* **poi_interactive_marker/feedback** (`visualization_msgs/InteractiveMarkerFeedback`)
* **rms/action/feedback** (`robot_simple_command_manager_msgs/RobotSimpleCommandActionFeedback`)
* **rms/action/result** (`robot_simple_command_manager_msgs/RobotSimpleCommandActionResult`)
* **rms/action/status** (`actionlib_msgs/GoalStatusArray`)
* **robot_local_control/LocalizationComponent/status** (`robot_local_control_msgs/LocalizationStatus`)
* **/tf** (`tf2_msgs/TFMessage`)
* **/tf_static** (`tf2_msgs/TFMessage`)


### 2.3 Published Topics

* **initialpose** (`geometry_msgs/PoseWithCovarianceStamped`): Publishes the robot's initial pose.
* **poi_interactive_marker/state** (`poi_manager_msgs/PoiState`): Publishes the state of the interactive marker.
* **poi_interactive_marker/update** (`visualization_msgs/InteractiveMarkerUpdate`): Publishes updates to the interactive marker.
* **poi_interactive_marker/update_full** (`visualization_msgs/InteractiveMarkerInit`): Publishes the full initialization of the interactive marker.
* **rms/action/cancel** (`actionlib_msgs/GoalID`): Publishes cancel requests for RMS actions.
* **rms/action/goal** (`robot_simple_command_manager_msgs/RobotSimpleCommandActionGoal`): Publishes goal requests for RMS actions.


### 2.4 Services

* **poi_interactive_marker/add_poi**: Adds a new POI via interactive marker.
* **poi_interactive_marker/add_poi_and_joints**: Adds a new POI along with joint information.
* **poi_interactive_marker/delete_all_pois**: Deletes all POIs.
* **poi_interactive_marker/delete_poi**: Deletes a specific POI.
* **poi_interactive_marker/get_current_pose**: Retrieves the current pose of the robot.
* **poi_interactive_marker/get_loggers**: Gets the list of available loggers.
* **poi_interactive_marker/get_poi_list**: Retrieves the list of all POIs.
* **poi_interactive_marker/get_poi_names_list**: Retrieves the list of POI names.
* **poi_interactive_marker/save_named_robot_pose**: Saves the robot's current pose with a specified name.
* **poi_interactive_marker/save_robot_pose**: Saves the robot's current pose.
* **poi_interactive_marker/set_logger_level**: Sets the logger level.
* **poi_interactive_marker/stop_goto**: Stops the current GOTO action.
* **poi_interactive_marker/update_poi_name**: Updates the name of a POI.


### 2.5 Services Called

* **poi_manager/add_poi**: To add a new POI to the manager.

* **poi_manager/delete_poi**: To remove a POI from the manager.

* **poi_manager/get_poi_list**: To retrieve the current list of POIs.

### 2.6 Action server

No

### 2.7 Action clients called

No

### 2.8 Required tf Transforms

No

### 2.9 Provided tf Transforms

No

### 2.10 Bringup

No

## Design notes

- `pose_dict` is the single source of truth for POIs. `pose_list` is reconstructed from it using `process_pose_dictionary()`.
- Validation is centralized in `_validate_poi`. Requests pass their POIs to `_save_pois_list`, which validates, updates `pose_dict`, rebuilds `pose_list`, and saves YAML.
- Concurrency: a `threading.Lock` (`self.poi_lock`) guards read/write operations to avoid races.

## Troubleshooting

- Interaction areas too small: Increase `drag_pad_extra_scale` and/or `rotate_handle_radius`.
- Pads too visible or distracting: Reduce alpha in `drag_pad_color` / `rotate_pad_color`.
- RViz ring too small: The global interactive marker `scale` is increased proportionally to `drag_pad_extra_scale`.