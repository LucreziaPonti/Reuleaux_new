#include <ros/ros.h>
#include <geometry_msgs/Pose.h> 
#include <reule_aux/BP_Res.h>

//#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>

//services
#include "std_srvs/Trigger.h"
//for TF srvs
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
//for NAVIGATION srvs
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>


std::vector<reule_aux::BP_Res> received_bp_;
std::vector<geometry_msgs::Pose> base_poses_;
reule_aux::BP_Res best_pose_;
bool arm_;
std::string fixed_frame_;
std::string robot_root_;
std::string arm_root_;

void clear_all(){
    received_bp_.clear();
    base_poses_.clear();
    reule_aux::BP_Res ep; //empty pose
    best_pose_=ep;
    arm_root_.clear();
    robot_root_.clear();
    arm_=true;
}
/*
    can receive and store poses while base_poses_ is empty - it gets filled once i receive the best pose
    once filled the base_poses_ vector i can use the service to move the robot 
*/
void store_data(std::vector<reule_aux::BP_Res> poses, reule_aux::BP_Res best){
    arm_root_ = best.arm_root_frame;
    robot_root_ = best.robot_root_frame;
    arm_ = (arm_root_!=robot_root_);

    // sort the poses by score
    for (int i = 1; i < poses.size(); i++){
        for(int j=i;j>0;j--){
            if (poses[j].score>=poses[j-1].score){
                std::swap(poses[j],poses[j-1]);
            }else{
                break;
            }
        }   
    }

    //store the poses in a new vector making sure the best pose is on top (if multiple poses with highest score it might not be)
    std::vector<geometry_msgs::Pose> sorted_poses;
    sorted_poses.reserve(poses.size());
    for (int i = 0; i < poses.size(); i++){ 
        if(poses[i].pose.position.x==best.pose.position.x && poses[i].pose.position.y==best.pose.position.y && poses[i].pose.position.z==best.pose.position.z && poses[i].pose.orientation.x==best.pose.orientation.x && poses[i].pose.orientation.y==best.pose.orientation.y && poses[i].pose.orientation.z==best.pose.orientation.z && poses[i].pose.orientation.w==best.pose.orientation.w){
            sorted_poses.insert(sorted_poses.begin(),best.pose);
        }else{
            sorted_poses.push_back(poses[i].pose);
        }
    }

    if(arm_){ // if the pose received is of the base of the manipulator (arm) i need to make sure to have the poses of the root of the robot, in order to move it
        // get the transform arm_root to robot_root
        tf2_ros::Buffer tf_buffer;
        tf2_ros::TransformListener tf_listener(tf_buffer);
        geometry_msgs::TransformStamped arm_to_robot;
        try{
            arm_to_robot = tf_buffer.lookupTransform(arm_root_, robot_root_, ros::Time(0), ros::Duration(3.0));
        }catch (tf2::TransformException &ex){
            ROS_WARN("%s", ex.what());
        }
        // debug if ok
        ROS_DEBUG("move_to_bp_result - STORE_DATA - check armToRobot TF: %f %f %f %f %f %f %f",arm_to_robot.transform.translation.x,arm_to_robot.transform.translation.y,arm_to_robot.transform.translation.z,arm_to_robot.transform.rotation.x,arm_to_robot.transform.rotation.y,arm_to_robot.transform.rotation.z,arm_to_robot.transform.rotation.w);
        
        for (int i = 0; i < sorted_poses.size(); i++){ // apply the transform to all the poses
            geometry_msgs::PoseStamped arm_pose;
            arm_pose.pose = sorted_poses[i];
            geometry_msgs::PoseStamped robot_pose;
            tf2::doTransform(arm_pose,robot_pose,arm_to_robot);
            sorted_poses[i] = robot_pose.pose;
        }
    }

    base_poses_ = sorted_poses;
}


void bp_sub_cb(const reule_aux::BP_Res::ConstPtr& msg){ 
//receive all the results from the BPP until the best pose is received 
//once received the best, call the store_data to order them and make srvs available 
//until the node is reset (by srvs reset or because robot has been moved) it can not receive new poses
    if(base_poses_.empty()){
        reule_aux::BP_Res appo;
        appo.best_pose=msg->best_pose;
        appo.pose=msg->pose;
        appo.score=msg->score;
        appo.robot_root_frame=msg->robot_root_frame;
        appo.arm_root_frame=msg->arm_root_frame;
        if(appo.best_pose){
            ROS_INFO("move_to_bp_result : best pose received - storing all data");
            best_pose_ = appo;
            store_data(received_bp_,best_pose_);
            ROS_INFO("move_to_bp_result : all data stored - services are now available"); /////
        }else{
            received_bp_.push_back(appo);
        }
    }else{
        ROS_ERROR("Can't receive new base placement to go to - use the available services to move the robot (move_to_bp_result/move_tf_static) or reset the node (move_to_bp_result/reset)");
    }
}

bool move_tf_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if(!base_poses_.empty()){
        ROS_INFO("move_to_bp_result - TF: creating the tf to broadcast with the best pose");
        //generally there should not be issues with the positioning with the tf, so we can do it only for the best pose (=base_poses_[0])
        static tf2_ros::StaticTransformBroadcaster tf_br;
        geometry_msgs::TransformStamped tf_stampd;

        tf_stampd.header.stamp = ros::Time::now();
        tf_stampd.header.frame_id = fixed_frame_;
        tf_stampd.child_frame_id = robot_root_;
        tf_stampd.transform.translation.x = base_poses_[0].position.x;
        tf_stampd.transform.translation.y = base_poses_[0].position.y;
        tf_stampd.transform.translation.z = base_poses_[0].position.z;
        tf_stampd.transform.rotation.x = base_poses_[0].orientation.x;
        tf_stampd.transform.rotation.y = base_poses_[0].orientation.y;
        tf_stampd.transform.rotation.z = base_poses_[0].orientation.z;
        tf_stampd.transform.rotation.w = base_poses_[0].orientation.w;

        tf_br.sendTransform(tf_stampd);
        ROS_INFO("move_to_bp_result - TF: static tf sent");
//********************** */
        res.success=true;
        std::string appo= "Moved to pose ("+std::to_string(base_poses_[0].position.x)+","+std::to_string(base_poses_[0].position.y)+","+std::to_string(base_poses_[0].position.z)+") ("+std::to_string(base_poses_[0].orientation.x)+","+std::to_string(base_poses_[0].orientation.y)+","+std::to_string(base_poses_[0].orientation.z)+","+std::to_string(base_poses_[0].orientation.w)+")";
        res.message= appo;
        clear_all();
    }else{
        ROS_ERROR("Cannot execute the service, NO POSES STORED");
        res.success=false;
        res.message="Service not available - no poses stored";
    }
    return true;
}

geometry_msgs::Pose fixValidOrientation(geometry_msgs::Pose pose){
    ROS_WARN("move_to_bp_result : fixing the orientation to send to NAV goal so that z axis is vertical");
    tf2::Quaternion quat(pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w);
    //convert quat to RPY to remove x,y rotations
    double roll, pitch, yaw;
    tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    tf2::Quaternion q_new;
    q_new.setRPY(0, 0, yaw);
    q_new.normalize();
    tf2::convert(q_new, pose.orientation);
    return pose;
}


bool move_nav_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if(!base_poses_.empty()){
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        for(int i=0;i<base_poses_.size();i++){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = fixed_frame_; /////// this ok or maybe better to use map?
            goal.target_pose.header.stamp = ros::Time::now();
            
            goal.target_pose.pose= fixValidOrientation(base_poses_[i]); // the pose is always a pose of the root of the robot 

            ROS_INFO("Sending goal (pose n.%d)",i);
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_DEBUG("Moved base to pose %d : (%f,%f,%f) (%f,%f,%f,%f)",i,goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
    //********************** */
                res.success=true;
                std::string appo= "Reached pose "+std::to_string(i+1)+" ("+std::to_string(goal.target_pose.pose.position.x)+","+std::to_string(goal.target_pose.pose.position.y)+","+std::to_string(goal.target_pose.pose.position.z)+") ("+std::to_string(goal.target_pose.pose.orientation.x)+","+std::to_string(goal.target_pose.pose.orientation.y)+","+std::to_string(goal.target_pose.pose.orientation.z)+","+std::to_string(goal.target_pose.pose.orientation.w)+")";
                res.message= appo;
                break;
            }else{
                ROS_DEBUG("Can't move to base pose %d : (%f,%f,%f) (%f,%f,%f,%f) - attempting next pose",i,goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
                ROS_INFO("move_to_bp_result - NAV: pose %d failed - attemping next pose",i+1);
                if(i==base_poses_.size()-1){
                    ROS_ERROR("move_to_bp_result - NAV:  Couldn't reach any of the poses");
                    res.success=false;
                    res.message="Couldn't reach any of the poses";
                }
            }
        }
        ROS_INFO("move_to_bp_result - NAV:  Resetting the poses array and the node to be used again");
        clear_all();
    }else{
        ROS_ERROR("Cannot execute the service, NO POSES STORED");
        res.success=false;
        res.message="Service not available - no poses stored";
    }
    return true;
}

bool reset_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    ROS_DEBUG("Resetting move_to_bp_result:");
    clear_all();
    res.success=true;
    res.message="all data was reset";
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "move_to_bp_result");
    ros::NodeHandle n; 
    clear_all();

    n.getParam("BPP_fixed_frame", fixed_frame_);

    ros::Subscriber bp_sub = n.subscribe("reule_aux/bp_results", 1000, bp_sub_cb);
    
    ros::ServiceServer move_tf_srv = n.advertiseService("move_to_bp_result/tf_static",move_tf_cb);
    ros::ServiceServer move_nav_srv = n.advertiseService("move_to_bp_result/nav",move_nav_cb);
    ros::ServiceServer reset_srv = n.advertiseService("move_to_bp_result/reset",reset_cb);
    ROS_INFO("move_to_bp_result: READY TO RECIEVE BASE POSES");
    ros::spin();

    return 0;
}