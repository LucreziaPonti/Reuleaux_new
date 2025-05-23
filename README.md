# Reuleaux_new
This repository contains the results obtained with the work done on the Reuleaux package (ORIGINAL REPO: https://wiki.ros.org/reuleaux) for the thesis:
**idk how it will actually be called**

This work is a further developement from the previous work on the thesis **Reuleaux Optimization of Base Placement for Mobile Robots in a Kitchen Environment**, and takes its results (contained in the repo https://github.com/LucreziaPonti/tesi_reuleaux_tiago) as a starting point.

The previous repo contains along with the Reuleaux package the other packages used to fully develop the complete simulation. 
This repository only contains the reuleaux package. 
The other packages used in this thesis work are:

- TIAGO robot: https://github.com/LucreziaPonti/TiagoForReuleaux.git - my repository containing all the pkgs for the tiago robot + some minor changes (!! make sure to look into the README file for additional installation steps, after having installed Reuleaux)

- RML63-B : https://github.com/LucreziaPonti/RM63BforReuleaux - my repository containing the robot description and the moveit_config package (created with the Moveit Setup Assistant)

- Full simulation : https://github.com/LucreziaPonti/FullSimForReuleaux - contains multiple packages that allow to create the full simulation with the robot TIAGo in a kitchen environment
    - sim_gazebo : contains all the launch, map and config files for the kitchen simulation
    - grasp : contains multiple packages, cloned from the repositories of Jennifer Buehler that provide different plugins for gazebo and RViz: to simulate a fake object recognition, load collisionobjects in the planning scene and execute grasps easily ( check out the original repos and all their wiki pages for more info: https://github.com/JenniferBuehler)


## Differences from the original REPO
*da aggiungere una volta concluso il lavoro - vedi note per idee*
- map generator !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

- updated function **createMarker::updateRobotState** to update the full robot and not only the manipulator group (additional arg *arm_only* allows to choose to only update the manipulator insted of all the robot)
    - added to the functions *CreateMarker::getDefaultMarkers* (used by *add_robot_base* - the UserIntuition method - to create the robot interactive button), *PlaceBase::transformToRobotbase* (used to create the robot_base IRM for VerticalRobotModel metod) and *PlaceBase::transformFromRobotbaseToArmBase* 
    - **ATTENTION** For this function to work properly make sure that in the SRDF file of your robot, if there is one, the virtual joint is called "virtual_joint" 
- generalized the setup of the fixed frame in the base placement plugin: now it can be set at launch with the dedicated arg that loads a param, that is then used in all the pieces of the plugin that require it.


## Setup
- ROS Noetic INSTALLATION: https://wiki.ros.org/noetic/Installation/Ubuntu
- Install OTHER IMPORTANT PKGs : 
    ```
    sudo apt-get install git wget ipython3 python3-catkin-tools python-is-python3 ros-noetic-rviz-visual-tools ros-noetic-navigation ros-noetic-octomap
    ```
- Install MOVEIT from pre-built binaries :  
    ```
    sudo apt install ros-noetic-moveit
    ```
- Create a workspace and clone the repository in the src folder 

- ROBOT INFORMATION: for this package to work you will need a robot description packages, most important package is the *robot*_moveit_config package (make sure it is properly setup - see reule_aux and base_placement_plugin README for + info on how it will be used)   
    !!!! make sure to have a virtual_joint called "virtual_joint" that is either floating or planar, so that you will be able to move the robot around the planning scene
- additional ROBOT SETUP - if you want to use IKFast directly: in map_generator>include>map_generator add the *robot*_ikfast_solver.cpp and add "#include *robot*_ikfast_solver.cpp" in the kinematics.h file 
- correct the Launch files with the right information for the robot you want to use with the base placement plugin.


## Use
!!! For all packages refer to their own README pages. This is just an overview:
- **reuleaux**: is a metapackage useful for building;
- **workspace_visualization**: contains the "setup" for the visualization of all the maps (RM, IRM, UM), the icons and the plugin descriton of the custom display
- **map_generator**: the nodes to create and visualize the Reachability Map and Inverse Reachability Map - the RM computation can be done either using IKFast directly or through MoveIt! (to have the self-collision checks);  
- **base_placement_plugin**: is the main package of the repository, it contains the plugin for Rviz that allows to create the task and compute the base placement of the robot. 
- **reule_aux**: new auxiliary package which contains the definition of new msgs (like reule_aux/bp_results used in the BPP) and nodes useful for a better use of the repo in a complete simulation

## **ATTENTION**
This work has been done for a Bachelor degree thesis in Automation Engineering by a candidate that is self-taught in ROS: it may contain errors and not fully correct information.  
The README are written as I wished they were while learning - they contain a lot of explainations that most ROS users don't need, and might not be the most efficient/correct way to use some tools/resources.   








