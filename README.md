# Reuleaux_new
This repository contains the results obtained on the Reuleaux package (ORIGINAL REPO: https://wiki.ros.org/reuleaux) done for the thesis:
**idk how it will actually be called**

This work is a further developement from the previous work on the thesis **Reuleaux Optimization of Base Placement for Mobile Robots in a Kitchen Environment**, and takes its results (contained in the repo https://github.com/LucreziaPonti/tesi_reuleaux_tiago) as a starting point.

The previous repo contains along with the Reuleaux package the other packages used to fully develop the complete simulation. 
This repository only contains the reuleaux package. 
The other packages used in this thesis work are:

*** TIAGO

*** PANDA

*** grasp (?)


## Differences from the original REPO
*da aggiungere una volta concluso il lavoro - vedi note per idee*

## Setup
- ROS Noetic INSTALLATION: https://wiki.ros.org/noetic/Installation/Ubuntu
- Install OTHER IMPORTANT PKGs : 
    ```
    sudo apt-get install git wget ipython3 python3-catkin-tools python-is-python3 ros-noetic-rviz-visual-tools
    ```
- Install MOVEIT from pre-built binaries :  
    ```
    sudo apt install ros-noetic-moveit
    ```
- Install "additional" PKGs for BP_TO_NAV **!!!!! check se lo metto effettivamente**:
    ```
    sudo apt install ros-noetic-navigation
    ```
- Create a workspace and clone the repository in the src folder 
- ROBOT INFORMATION: for this package to work you will need a robot description packages, most important package is the *robot*_moveit_config package in order to have the robot description and the kinematic solver available through MoveIt! 
- ** *at the moment* ** ROBOT SETUP: in map_creator>include>map_creator add the *robot*_ikfast_solver.cpp and add "#include *robot*_ikfast_solver.cpp" in the kinematics.h file 



## Use
!!! For all packages refer to their own README pages. This is just an overview
- **reuleaux**: is a metapackage useful for building;
- **workspace_visualization**: contains the "setup" for the visualization of all the maps (RM, IRM, UM), the icons and the plugin descriton of the custom display
- *map_creator*: contains the tools to create, load and visualize the RM and IRM; (as is it uses directly IKFast for the computation - no self-collision check)
- *map_generation*: contains a node that allows to create the REACHABILITY MAP exploiting MoveIt!, to perform self-collision checks; It relies of map_creator for the computation of the IRM and the visualization of all maps;
- **base_placement_plugin**: is the main package of the repository, it contains the plugin for Rviz that allows to create the task and compute the base placement of the robot. 
- **reule_aux**: new auxiliary package which contains the definition of new msgs (like reule_aux/bp_results used in the BPP) and nodes useful for a better functioning of the repo








