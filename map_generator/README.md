# MAP_GENERATOR
This pkg is a combination of the map_generator pkg (from the original reuleaux repo) and the map_generation pkg (from a PR to the original repo). 
It is made to combine the resources provided by map_generator and the use of moveit for the creatorion of the Reachability Map of map_generation.

## USAGE
### Reachability map generation
Use the launch file generate_RM.launch - specify the args



## CODE EXPLENATIONS
### Libraries
- hdf5_dataset : create the .h5 files with the maps from the workspace created in the code (and viceversa)
### Nodes
- create_reachability_map_node : node that uses the libraries to create the reachability map  - This node exploits MoveIt! to query the IK solver WITH collision check, therefore is MUCH slower    
    USAGE:  launch it using the generate_RM.launch file  - SEE ABOVE

- OG_create_RM : original node to create the RM, which uses the IKFast cpp solver for the IK WITHOUT self-collision check - is much faster BUT less accurate
    SETUP: create your robot IKfast cpp solver (and the plugin for moveit) - add the .cpp to the *include* folder - change the "*#include <ROBOT_IKFAST>_solver.cpp*" line in **kinematics.h** with yours   
    USAGE: ``` rosrun map_generator OG_create_RM <resolution> <file_name>```    
        The *file_name* arg is optional - if omitted the node will create a default name with the robot name, group and the resolution  
        The smaller the resolution value, the longer it will take to create (and then use) the RM, a save minimum value is 0.05 - if not provided the default value is 0.08

- create_inverse_reachability : node that creates the inverse RM from a given RM    
    USAGE: ``` rosrun map_generator load_reachability_map <hdf5_file_RM> <file_name_IRM>``` 
        The *hdf5_file_RM* arg is necessary and has to contain the relative path to the map (SUGGEST instead running from the folder where you have the map)    
        The *hdf5_file_RM* arg optional - if omitted the node will create a default name by adding "_INVERSE" to the name of the RM

- load_reachability_map : used to publish the topic to visualize the map in rviz    
    USAGE:  start Rviz with the robot model loaded (*suggestion*: launch the moveit_config demo.launch) and add the displa **reachability_map** with the topic */reachability_map*, then run: ```rosrun map_generator load_reachability_map <hdf5_file> <reference_frame_id>  ```   
    !!! run the load node either from the folder where you have your map, or you need to add the full path to your map



    
