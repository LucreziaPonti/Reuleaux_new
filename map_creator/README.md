### Map Creator
====
***All the Inverse Kinematics solutions for Reuleaux is generated by ikfast. The generated solutions from ikfast for the desired robot should be linked with the header files of Reuleaux. Please refer to the [http://wiki.ros.org/reuleaux] (http://wiki.ros.org/reuleaux)  page for the process of generating ikfast solution for your robot and linking the ikfast solution to Reuleaux. The default robot provided in the package is motorman_mh5. You can easily import your robot by the instructions.*** 

Reuleaux map_creator pacakge creates three types of maps. (You don’t have to create all the maps, it is per your needs):
## 1. Reachability map
The Reachability map describes the reachability of a given robot model by discretizing its environment, creating poses in the environment and calculating valid IK solutions for the poses. The poses which are reachable by robot are associated with discretized spheres. The reachability of each sphere in the environment are parameterized, by a Reachability index. The output is saved as an hdf5 file (link for hdf5 file) which has details about all the reachable poses and discretized spheres. There are mainly two user arguments:

a) Resolution parameter: The first step of the map generation process is discretization of the environment by voxelization. The resolution determines how much small the boxes would be. The smaller voxels, the higher the number of poses per spheres. (**Believe me, it grows exponentially. Please do not try to go too low with the resolution. The safe threshold is 0.05. Less than that, can take hours, or your system may stop responding**). If the user does not provide any resolution, the default resolution is 0.08

b) Map filename: The second argument decides the output filename. If the user does not provide an ouput filename, it will automatically decide an ugly map name with the robot name and provided resolution.
To create a reachability map, run:

rosrun map_creator create_reachability_map

with arguments:

rosrun map_creator create_reachability_map 0.05 funny_robot.h5

When the process finishes, the output reachability map will be stored in map_creator/maps folder. If you do not have the existing maps folder, do not worry. It will create a map folder in the map_creator package and store the output there. 

## 2. Capability Map (optional)
Capability map is an extension of reachability map (I guess you have already done that, otherwise please create a reachability map first), where the outer spheres of the reachability map, is set as cones. So the reachability limit of the robot is well visualized. All the outer spheres are decided for a principal axes and iterates over different values for opening angles for cones. The suitable opening angle that correctly accumulates all the poses on that sphere, is picked up
(** Until we found out some very useful algorithm for capability map, the process may take several hours based on resolution)
The process is same as creating reachability map:

rosrun map_creator create_capability_map

The ouput map file will also be stored in map_creator/maps folder.


## 3. Inverse Reachability Map
The purpose of  Inverse Reachability map is to find suitable base positions for a robot with given task poses. To know how to find suitable bases, please refer to (base_placement plugin page)
The inverse reachability map is a general inverse transformation of all the reachable poses of the reachability map of the robot. The user have to provide the reachability map as an argument. The desired name of the ouput file can also be provided. If no output file name is provided, the system will automatically generate a map file with the robot name and resolution provided in the reachability map. To create an inverse reachability map:

rosrun map_creator create_inverse_reachability_map motoman_mh5_r0.08_reachability.h5

(provided reachability map is of motoman_mh5 robot and resolution is 0.08).  The outout inverse reachability map will be stored in map_creator/Inv_map folder unless specified otherwise.

 (Congratulations. You have now also created the inverse reachability map which is the key element for finding bases. Now it is the time to see the result of all the hard work. Let’s move to the [visualization] (https://github.com/ros-industrial-consortium/reuleaux/tree/master/workspace_visualization) page) 

## 4. Visualise an Existing Reachability Map
To visualize in RViz the maps created use the display "reachability map" with the topic /reachability_map
```bash
roscd map_creator/maps/

rosrun map_creator load_reachability_map *map_file_name*.h5 *optional arg*
```
The optional arg allows to change the reference frame in which the map will be loaded:
- no optional arg: frame= "arm_tool_link" - use for IRM
- 'c' : frame= "torso_lift_link"  - for RM created with map_creator and map_generation AFTER CENTERING 
- 'g' : frame= "base_footprint" - for RM with map_generation NOT CENTERED (!! maps have to be centered before creating IRM)
- any other value: frame= "base_link" - default frame originally used by reuleaux
!! These frames are ok for TIAGo