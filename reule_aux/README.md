# Reule_aux
Auxiliary package for reuleaux's original packages.
(so Reule_AUX, you know, like reuleaux + AUXiliary, get it? *:)* )

It contains the definition of new messages, services and nodes useful in the packages (mainly base_placement_plugin)

## MOVE TO BASE PLACMENT RESULTS
### New messages and topics
- **reule_aux/bp_results** is the new topic used in the BPP (see base_placement_plugin>src>place_base.cpp) to publish messages (of the new type *BP_Res* defined here - see msgs) containing the poses obtained with the plugin
- **reule_aux/BP_Res.msg** is a custom message type (for the topic above) which allows to transfer all the info of the results given by the BPP, it contains: 
    - bool *best_pose* -> true only fot the best pose
    - string *robot_root_frame* -> name of the robot root frame,  obtained in placeBase using the robot_model (for tiago: base_footprint)
    - string *arm_root_frame* -> name of the "root"/parent frame of the manipulator/arm,  obtained in placeBase using the robot_model (for tiago, if using *arm* planning group : torso_lift_link)
    - float32 *score* -> reachability score of the pose (used to order the poses, making sure the ones with the highest score are considered first, in case the best pose fails)
    - geometry_msgs/Pose *pose* -> (robot or arm) base pose obtained with the BPP 
### move_to_bp_result node
**move_to_bp_result** is a new node that is used to subscribe to the bp_results topic, order and store the poses obtained and provide services to actually uses the results.  
The poses once obtained are sorted by score.  
By checking the *robot_root_frame* and *arm_root_frame* variables, the node understands if the poses obtained are of the manipulator base (with methods PCS,IKS,GRS) or of the robot base (with methods VRM, UI), in the first case an additional step is required: the arm base poses have to be transformed into poses of the root of the robot.  
This transformation then allows to move the robot using either a static tf of the navigation stack (depending on how the robot is being simulated). 

To do this there are two services available: 
- *move_to_bp_result/static_tf* : publishes the tf between the fixed_frame and the robot_root_frame - in this case the robot can be moved to a "floating" position (use when simulating the robot only in Rviz or if no navigation system is provided) - useful for simple manipulators
-  *move_to_bp_result/nav* : sends a goal to the move_base action server to move the robot using the provided navigation system. Since sometimes it happens that some goals result not "navigable", this service, if the best pose fails, attempts with the next of those received and sorted until one is reached (use when simulating in gazibo with a working navigation system) - useful for mobile robots

#### move_to_bp_result_TIAGO
This node is a more robot-specific version of the previous. While the previous should work generally for most robots (especially simpler ones), the TIAGo robot has some particular limitations that required to have some additional computations, while still using and providing the same services.

Feel free to use the previous node as a baseline and customize it to the necessities of the robot used. (MAKE SURE TO CHANGE THE NODE NAME IN THE BASE_PLACEMENT.LAUNCH FILE)

The additional computation is used to compute the value to "give" to the torso joint to make sure the arm is positioned correctly. To do this the transform between torso_lift_link and torso_footprint (dummy frame fixed to the torso - corresponds to the base_footprint frame when the torso joint is at its minimum value) is applied to the arm poses received, from the z position obtained the torso value is set (making sure it is within its limits) and the poses sent to the navigation will have z=0, to make sure the goal is feasible.  
The goal is sent to the navigation stack and the value for the torso is given to the robot using the Torso_controller.  
!! If the TIAGo is simulated only in RViz (e.g. using the demo of the moveit_config pkg) the torso_controller is not available (usually), so the position broadcasted with the tf will make the robot float (use the robot_model display to visualize the robot floating - the moveit one does not show it defying gravity). 

### Launch
To use the reuleaux base placement plugin (and move_to_bp_reusult): 
```
roslaunch base_placement_plugin base_placement.launch bp_results:=true
```
Move the robot in RViz using static tf: 
```
rosservice call move_to_bp_reusult/static_tf
```
Move the robot in simulation using the navigation stack goal:
```
rosservice call move_to_bp_reusult/nav
```

## OCTOMAP MAPPING
[Octomaps](https://octomap.github.io/) are 3D occupancy grid maps, based on the octree structure.  
Libraries are available that provide the tools to generate, visualize and use these maps.  
I highly suggest to follow the official istructions for installation and usage, but here is my mini "guide".  

### Installation
Make sure that the octomap, octovis and octomap-mapping packages are installed (use ```sudo apt search``` to check).

### Usage
The octomap_server_node works by taking in the information of a PointCloud2 sensor and using it to create the octomap.  Its functioning is very similar to the *map_server* used for 2d navigation:
It allows to create new maps from zero or from a static map loaded at start, to save the map created and get it (via topic or service).

To run correctly we need to make sure all the parameters are set and the topic names remapped. There are quite a few parameters needed, make sure to check out the [rosWiki page](http://wiki.ros.org/octomap_server) to see all and understand better the functioning.

The octomap_server pkg provides different nodes, but the most important (in my opinion, and for this work) are: **octomap_server_node** which performs the mapping and advertises all the topics and services to use the map and **octomap_saver** which once the map is created (with the node) saves them (in format *.bt* for BINARY MAPS - free/occupied - or *.ot* for FULL PROBABILITY MAPS - probability of occupancy). Additionaly an other useful node is the **octomap_server_static** node which, given a static map, advertises the services to get it - not the topics.


In this package a launch file (*octomap_server.launch*) is provided that allows to set all the needed parameters and run the **octomap_server_node** correctly for mapping (from zero or with a static map as "seed"). (see below)   

To make sure that the mapping is working and to visualize the map you need to run rviz and add a **MarkerArray** (and set the topic to */occupied_cells_vis_array*)

#### Launch options:
- Mapping from zero: *latch=false* , set the parameters for the ground plane filtering as you prefer them  
Launch the server and visualize it in rviz, once the map is completed to your satisfaction, save it (make sure to do this in the folder where you want to save your map):
    ```
    rosrun octomap_server octomap_saver -f <name of your map>.<bt/ot>
    ```
    *.bt* is used for BINARY MAPS  - *.ot* is used for FULL PROBABILITY MAPS
- Mapping with a static map loaded: *latch=true*, set the *static_octomap_name* param to the name of the map you want to load **including** ".bt/.ot" 
- OCTOMAP VISUALIZATION: in the launch use the arg *rviz* to open a new rviz window with the visualization of the octomap
- GROUND FILTERING: in many applications - such as this - the ground needs to not be considered as an obstacle. Setting the param *filter_ground* to true allows to do that (see the official wikis for a better explanation on how this is done), other parameter are used to adjust the ground detection method. 

### Load octomap to planning scene
Build the octomap in the planning scene takes time and requires the robot to move around a lot: move_group provides a service to load a (binary) prebuilt octomap to the planning scene:
```
rosservice call /move_group/load_map "filename: '/full/absolute/path/to/the/map/<map name>.bt'"
```
