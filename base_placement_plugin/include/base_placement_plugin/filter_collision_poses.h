
#ifndef FILTER_COLLISION_POSES_H
#define FILTER_COLLISION_POSES_H

#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningScene.h>


class FilterCollisionPoses {
public:
    FilterCollisionPoses(); //constructor
    bool check_collision_objects( ros::NodeHandle& node, const float x, const float y, const float z, bool checkZ); 

private:
    ros::ServiceClient planning_scene_client;
    float co_x,co_y,co_z,dX,dY,dZ;

};


#endif