# Base Placement plugin

## LAUNCH THE PLUGIN
There are a few different options to use the plugin:
1. Only the rviz panel (with all the required displays) and the code (useful with big simulation already running):
  ```
  roslaunch base_placement_plugin base_placement.launch
  ```
  ARGUMENTS: 
  - **bp_results** : allows to run an additional node move_to_bp_result (for the work with the TIAGo robot is actually move_to_bp_result_TIAGO) that subscribes to the topic *reule_aux/bp_results* to receive the results and elaborate them as needed - default = *false* 
  - **fixed_frame** : set the fixed frame for the plugin, it should be the parent of the virtual joint of the robot or the root frame of the robot  - default : *base_link* (may change the default depending on the robot used)
  - *Filtering parameters*: to choose and setup the options for the filtering of the Union Map (see related section of the Readme)
    - **TIAGO_torso_filtering** : bool to have the additional filtering for the TIAGo robot constraints - default = *true*  //// will be set to default false later in the work
    - **OGM2d_filtering** : bool to have the filtering using a Occupancy Grid Map (2-dimentional) - default = *false*
      - **OGM2d_topic** : name of the topic to retreive the OGM (make sure that it is a topic of type *nav_msgs/OccupancyGrid*) - default=*"/move_base/global_costmap/costmap"*
      - **OGM2d_cost_threshold** : value of the cost of the grid over which a point has to be filtered - default = *70*
    - **OCTOMAP_PS_filtering** : bool to have the filtering using a OCTOMAP from the Planning Scene(3-dimentional) - default = *false*
    - **OCTOMAP_SRV_filtering** : bool to have the filtering using a OCTOMAP from the octomap_server (3-dimentional) (alterinative method to the planning scene one) - default = *false*
    


2. The plugin + the robot (with Moveit! demo):
  ```
  roslaunch base_placement_plugin robot_base_placement.launch
  ```
  - **ATTENTION**: change in the launch file the name of the  *robot*_moveit_config pkg to use the correct one
  - SAME OPTIONAL ARGUMENTS AS base_placement.launch  


3. Manually setup the Rviz window - not suggested since the plugin requires multiple parameters, set at launch (in particular for the filtering):
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

# FILTERING OF THE UNION MAP
The creation of the union map is one of the first steps of the computation of the optimal base placement.  
It is done by combining "instances" of the IRM positioned in each of the grasp/task poses.  
However not all base poses might be actually feasible, so it is necessary to perform a filtering, and it is best to be done in the first steps of the creation of the UM so to reduce right away the number of poses considered (thus reducing the time required for the computation).  
The step of the "combination of the IRMs" is done in the function associatePose of discretization.cpp in **Map_generator**, this is where the filtering has been added.

Multiple "types" of filtering have to (and can) be performed, depending on the resolution method chosen and of the resources available:

- Robot base poses filtering: when using the VerticalRobotModel method the poses that are requested are of the robot base, generally speaking that means that those poses have to be on the ground and can only have an orientation with the Z axis perpendicular to the ground (again generally - if the base frame of your robot is oriented differently make sure to correct this part). The function associatePose receives as argument *arm_pose* (Boolean) that determines if this filtering has to be done or not. The poses are filtered with some allowance, so to not be too restrictive (and risk filtering out all the poses).

- Collision filtering: poses of both the arm base and the robot base must not be in collision with the objects of the scene. This filtering can be done using different resources (also depending on the robot used): 
  - **Occupancy Grid Map** (2d: *nav_msgs/OccupancyGrid*): occupancy grid maps contain information on the environment obstacles by assigning a value (or cost) to points/grids, and can be easily retreived by topic. Usually with the navigation stack multiple CostMaps are provided: *static map* (loaded at the beginning to seed the global cosmap, not updated during simulation), *local costmap* (contains partial information given by lidar/other sensors, used to update the global costmap, and for local motion planning) and *global costmap* (contains all the information obtained with the local map -and the static map- and is updated during simulation).  
  Using the **global costmap**, that can be retreived by topic (*/move_base/global_costmap/costmap*), the poses of the UM can be filtered to avoid poses in collision considering the navigation limitations of the robot. In the launch file a cost threshold has to be set to define over what cost a pose is considered not acceptable (sinche the costmap contains probabilistic info and not binary occupancy info)
    - **!!!** some occupancy grid maps may also contain 3D information but in this case the implementation considers 2D OGMs, which also means that for complex robots, such as TIAGo, -where the arm base frame and the robot base frame don't coincide- an additional step of transformation is necessary to ensure the navigation goal feasibilty in all cases.  
    - **!!** If you don't have/want a working navigation stack but have a static map of your environment, you can load it using the map_server (```rosrun map_server map_server mymap.yaml``` - need the relative path or run it from the map folder) and use the */map* topic to retreive it (change the OGM2d_topic arg in the BPP launch file)  

  - **Octomap**: octomaps contain information on the obstacles of the environment in a 3d format. The use of these maps is not always "automatically" integrated in a simple system (since it requires 3D sensors that publish PointCloud2 information and it is not used for navigation). To use this easily I added a launch file in the pkg reule_aux, as well as a very simple "guide" in the README file. In more complete systems, the octomap use can be already implemented, check that the octomap_server_node is running to make sure that the information is available. 
  To retreive the octomap of the environment there are two main options:
    - **Octomap_server** : both the octomap_server_node and the octomap_server_static provide the service */octomap_binary* to retreive an octomap (the first a octomap that is being built at the moment, maybe with static map as seed, and the second a static octomap)  
      **!!!** The octomap does NOT have an inflation layer, as the costmap does, so some poses considered acceptable might not be actually feasible due to the physical dimentions of the robot.
    - **Planning Scene** octomap : thanks to the [PERCEPTION PIPELINE](https://moveit.github.io/moveit_tutorials/doc/perception_pipeline/perception_pipeline_tutorial.html) of MoveIt (check your robot moveit_config pkg to make sure it is set if you want to use it) the Planning Scene can contain information on the environment in form of an octomap (created using either PointCloud or Depth image sensors). The octomap of the planning scene is used for collision checks during motion planning, to recognize objects and create collision objects (for grasping tasks) and more. TO retreive the octomap from the planning scene the service **/get_planning_scene** can be used.
    If you have not setup the perception pipeline but have a prebuilt octomap of your enviroment use the service */move_group/load_map* to load a static octomap to your planning scene.   
    

- ROBOT SPECIFIC FILTERING: some robots - as is the case for the TIAGo - have some more constraints that need to be taken into account (and that are not taken into account in the RM/IRM) in this case some additional steps are added to the function (MAKE SURE TO CHECK THEM OUT AND REMOVE/CHANGE THEM FOR YOUR ROBOT).  
For the TIAGo robot the additional constraints have to do with the torso: the torso joint is not taken into account in the manipulator (due to IKFast limitations) therefore the arm base poses have to respect its limits both in terms of height (max min values) and orientation (Z axis vertical) - !! All of this considering that the robot has to stay on the ground - if you want to simulate in a non physically realistic environment where your robot can float, then there is no issue :).