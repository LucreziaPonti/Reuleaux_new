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
- Useful packages: 
    ```
    sudo apt-get install git wget ipython3 python3-catkin-tools python-is-python3 
    ```
- INSTALL MOVEIT from pre-built binaries :  
    ```
    sudo apt install ros-noetic-moveit
    ```
- ROBOT INFORMATION: for this package to work you will need (*atleast*) the <<robot>>_moveit_config package in order to have the robot description and the kinematic solver available through MoveIt! - for a better use is of course recommended to have the full packages to simulate the robot. 





