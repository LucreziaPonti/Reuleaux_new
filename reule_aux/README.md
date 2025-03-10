# Reule_aux
Auxiliary package for reuleaux's original packages.
(so Reule_AUX, you know, like reuleaux + AUXiliary, get it? *:)* )

It contains the definition of new messages, services useful in the packages (mainly base_placement_plugin)

## New messages and topics
- **reule_aux/bp_results** is the new topic used in the BPP (see base_placement_plugin>src>place_base.cpp>findbase()) to publish messages (of the new type *BP_Res* defined here - see msgs) containing the poses obtained with the plugin

## Launch
- To use the reuleaux base placement plugin (and reuleaux_bp_to_nav): 
    ```
    roslaunch base_placement_plugin rviz_bp.launch
    ```
    To move to the obtained results (for PCA,IKS,GRS methods) : 
    ```
    rosservice call reuleaux_bp_to_nav/move_to_bp_arm
    ```
    To move to the obtained results (for VRM,UI methods) : 
    ```
    rosservice call reuleaux_bp_to_nav/move_to_bp_robot
    ```

