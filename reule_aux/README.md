# Reule_aux
Auxiliary package for reuleaux's original packages.
(so Reule_AUX, you know, like reuleaux + AUXiliary, get it? *:)* )

It contains the definition of new messages, services and nodes useful in the packages (mainly base_placement_plugin)

## New messages and topics
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

## Launch
- To use the reuleaux base placement plugin (and move_to_bp_reusult): 
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

