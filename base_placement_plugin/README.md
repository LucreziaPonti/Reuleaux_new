# Base Placement plugin

## LAUNCH THE PLUGIN
There are a few different options to use the plugin:
1. Only the rviz panel (with all the required displays) and the code (useful with big simulation already running):
  ```
  roslaunch base_placement_plugin base_placement.launch
  ```
  - optional argument **bp_results** : allows to run an additional node (for this work is reuleaux_bp_to_nav) that subscribes to the topic *reule_aux/bp_results* to receive the results and elaborate them as needed - default = *false* 
  - optional argument **adjust_frame** : when true a new tf is published to "connect" the generalized fixed frame of the plugin (*bpp_fixed_frame*) to the fixed frame of the robot (which needs to be set with the next arg) - default = *false*
  - optional argument **robot_fixed_frame** : allows to set the fixed frame of the robot - default : *base_link* (may change the default depending on the robot used)

2. The plugin + the robot (with Moveit! demo):
  ```
  roslaunch base_placement_plugin robot_base_placement.launch
  ```
  - **ATTENTION**: change in the launch file the name of the  *robot*_moveit_config pkg to use the correct one
  - optional argument **bp_results** : allows to run an additional node (for this work is reuleaux_bp_to_nav) that subscribes to the topic *reule_aux/bp_results* to receive the results and elaborate them as needed - default = *false* 
  - optional argument **adjust_frame** : when true a new tf is published to "connect" the generalized fixed frame of the plugin (*bpp_fixed_frame*) to the fixed frame of the robot (which needs to be set with the next arg) - default = *false*
  - optional argument **robot_fixed_frame** : allows to set the fixed frame of the robot - default : *base_link* (may change the default depending on the robot used) 

**ATTENTION** when choosing the fixed frame that you don't have other parent frames to that frame (e.g. when launching with the moveit demo it usually has a "world" frame automatically put as a parent to the base link of the robot - in that case the "frame adjusting" is not needed)

3. Manually setup the Rviz window:
  ```
  rosrun rviz rviz
  ```
  - From the panel window select base placement planner
  - Add a reachability map display and set the topic to /reachability_map
  - Add an Interactive Marker display and set the topic to /base_placement_plugin/update
  - Add a markerArray display. Set the topic to /visualization_marker_array



## WORK THE PLUGIN
==== Please refer to [ros wiki] (http://wiki.ros.org/reuleaux) for more detailed instruction

#1. RViz and RQT User Interface:

There are two types of interactive markers:
  - The red arrow acts as a pointer which the user can move around the RViz enviroment. Fruthermore by clicking on the arrow another magenta arrow is added to the RViz enviroment. This arrow acts as task poses for base placement planner.
  - The magenta arrow is the task poses for the base placement planner. The orientation of the arrow can be changed by holding the CTRL key and moving it with the mouse.
  - Each arrow has a menu where the user can either delete the selected arrow or it can change its position and orientation by using the 6DOF marker control.
  - The RQT UI communicates simultaniously with the RViz enviroment and the User can change the state of a marker either through RViz or the RQT UI 
  - TreeView displays all the added waypoints. The user can manipulate them directly in the TreeView and see their position and orientation of each waypoint.
  - The user can add new point or delete it through the RQT UI.
  - New tool component has been added for adding Arrows by using a mouse click

After deciding the task poses load an inverse reachability map previously created. 

After loading the inverse reachability map set the desired parameters. There are two parameters. Number of desired base locations and number of high scoring spheres from where the poses will be collected.

Set the desired output visualization method.
When everything is set up, press the Find Base button. It will show the base locations.

If you want to see the union map, press the show union map button.

## ATTENTION
For how the computation is done not all visualization methods can be used with all the resolution methods and viceversa:

- UserIntuition and VerticalRobotModel can **NOT** use the MANIPULATOR visualization (because they compute the *robot base* pose)
- RobotModel can NOT be used for the methods PrincipalComponentsArray, GraspReachabilityScore, IKSolutionScore (because they compute the *arm base* pose)

