# reuleaux_moveit
---

## Overview
This version of Reauleaux creates reachability maps using the [MoveIt!](https://moveit.ros.org/) interface.
Allowing Reuleaux to create a map of redundant manipulators with self collision checking.
![col](https://user-images.githubusercontent.com/3790876/27742268-da5dfc86-5d6c-11e7-91bc-aad20f5a4048.jpg)

### License
`TBD`
This source code is released under the ... license.

---

## Installation
1. Follow the [installation instructions](https://ros-planning.github.io/moveit_tutorials/doc/getting_started/getting_started.html#create-a-catkin-workspace-and-download-moveit-source) for MoveIt! for ROS Noetic.

2. You will need an updated fork of the original [Reuleaux package](http://wiki.ros.org/reuleaux) to visualise maps, however you won't need to install OpenRAVE to build this.

Working in you catkin_ws/src directory...
```bash
git clone --branch noetic-devel https://github.com/MShields1986/reuleaux.git
```

3. Clone this package, again working in you catkin_ws/src directory...
```bash
git clone https://github.com/MShields1986/reuleaux_moveit.git
```

4. Update and install any dependencies...
```bash
sudo apt update
rosdep update
rosdep install -r -y --from-paths . --ignore-src
```

5. Build the project...
```bash
cd ..
catkin build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

---

## Usage
A launch file is included that can be used with [the Panda arm demo](https://ros-planning.github.io/moveit_tutorials/doc/quickstart_in_rviz/quickstart_in_rviz_tutorial.html) that is bundled with MoveIt!

### Create a Reachability Map
Check the settings in [the launch file](https://github.com/MShields1986/reuleaux_moveit/blob/noetic-devel/map_generation/launch/generate_map.launch) and then run...
```bash
roslaunch map_generation generate_map.launch
```

Note that MoveIt! will throw a `Found empty JointState message` error for every pose solution (which is a lot), this is a [known issue](https://github.com/ros-planning/moveit/issues/659) and doesn't effect the generation of reachability maps.

!!!! as of now the centering function within the map_generation_node has issues and does not work properly: 
set the "do_centering" arg to false and use the fix_RM_reference_frame node in map_creator to "manually" fix the reference frame of the RM **BEFORE** creating the IRM - make sure to adjust the transformation that needs to be done within the node  !!!!

### Inverse Reachability Maps
Inverse reachability maps can be created using the map_creator package commands, as such:
```bash
roscd map_generation/maps/

rosrun map_creator create_inverse_reachability_map *RM_file_name*.h5 *optional_desired_IRM_filename*.h5
```

!! the maps are by default saved in map_creator/maps folder - I moved them manually to map_generation/maps for better organization

### Visualise an Existing Reachability Map
Use the map_creator package commands, as such:
```bash
roscd map_generation/maps/

rosrun map_creator load_reachability_map *map_file_name*.h5 *optional arg*
```
The optional arg allows to change the reference frame in which the map will be loaded:
- no optional arg: frame= "arm_tool_link" - use for IRM
- 'c' : frame= "torso_lift_link"  - for RM created with map_creator and map_generation AFTER CENTERING 
- 'g' : frame= "base_footprint" - for RM with map_generation NOT CENTERED (!! maps have to be centered before creating IRM)
- any other value: frame= "base_link" - default frame originally used by reuleaux
!! These frames are ok for TIAGo

