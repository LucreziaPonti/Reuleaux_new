/*
    IGNORE FOR NOW -- IT NEEDS TO BE FIXED

 waits for navigation to be available (wait for action move_base i think)

 receives the optimal bp (via service) from reuleaux bpp or from terminal 
 when it receives an empty goal it means that the goal recived RIGHT BEFORE is the BEST bp from reuleaux 
 so it stores it at the beginning of the array (index 0 - left empty from the start) and removes it from the end of the array
 (adds the orientation so that tiago is facing the table - not yet implemented)
 the service reuleaux_bp_to_nav/move_to_bp becomes available and no more bp can be received
 when the service is called it sends the goal to the navigation (starting with the first, if it fails it attempts the next in the array)
 */

#include "ros/ros.h"
#include "geometry_msgs/Pose.h" 
#include <tf2/LinearMath/Matrix3x3.h> 
#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include "std_srvs/Trigger.h"


bool receive;
bool service_avail;
std::vector<geometry_msgs::Pose> poses;
geometry_msgs::Pose empty_pose;
float torso_value;

void bp_sub_cb(const geometry_msgs::Pose::ConstPtr& msg){ 
    geometry_msgs::Pose appo;
    appo.position=msg->position;
    appo.orientation=msg->orientation;
    if(receive){
        ROS_DEBUG("pose received: [%f,%f,%f] [%f,%f,%f,%f]", appo.position.x, appo.position.y, appo.position.z, appo.orientation.x,appo.orientation.y, appo.orientation.z,appo.orientation.w);
        if(appo==empty_pose){
            poses[0]=poses[poses.size()-1];
            poses.pop_back();
            ROS_INFO("Saved best pose - storing all poses");
            receive=false;
            service_avail=true;
            ROS_DEBUG("poses stored");
            
            for(int i=0;i<poses.size();i++){
                ROS_DEBUG("pose %d : (%f,%f,%f) (%f,%f,%f,%f)",i,poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x,poses[i].orientation.y, poses[i].orientation.z,poses[i].orientation.w);
            }
        }else{
            poses.push_back(appo);
        }
    }else{
        ROS_ERROR("Can't receive new base placement to go to - call the /reuleaux_bp_to_nav/move_to_bp service to attempt move or reset_bp_to_nav to reset the node");
    }
}

geometry_msgs::Pose fix_base_orientation(geometry_msgs::Pose pose){
    if((pose.orientation.x==0)&&(pose.orientation.y==0)&&(pose.orientation.z==0)){
        //start with empty RPY angles
        double yaw=0.0;
        //dep on where the position is change yaw
        if((pose.position.y>0)||(pose.position.y<=-1.75)){ //is at the place table or at side of tables farther from start
            yaw=1.565;//rad = 90 deg
        }else{ //at pick table
            if ((pose.position.x>0.35)||(pose.position.y>=-1.15)){ // - is at side of table closer to start where no chairs
                yaw=-1.58;//rad = -90 deg
            }
        }
        // convert back to quaternions and overwrite
        tf2::Quaternion q_new;
        q_new.setRPY(0, 0, yaw);
        q_new.normalize(); //just to be safe
        pose.orientation.x = q_new.x();
        pose.orientation.y = q_new.y();
        pose.orientation.z = q_new.z();
        pose.orientation.w = q_new.w();
    }
    return pose;
}

bool send_robot_goal_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if(service_avail){
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        for(int i=0;i<poses.size();i++){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose=fix_base_orientation(poses[i]);

            ROS_INFO("reuleaux_bp_to_nav:  Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_DEBUG("Moved base to pose %d : (%f,%f,%f) (%f,%f,%f,%f)",i,poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x,poses[i].orientation.y, poses[i].orientation.z,poses[i].orientation.w);
                res.success=true;
                std::string appo= "Reached pose "+std::to_string(i)+" ("+std::to_string(poses[i].position.x)+","+std::to_string(poses[i].position.y)+","+std::to_string(poses[i].position.z)+") ("+std::to_string(poses[i].orientation.x)+","+std::to_string(poses[i].orientation.y)+","+std::to_string(poses[i].orientation.z)+","+std::to_string(poses[i].orientation.w)+")";
                res.message= appo;
                break;
            }else{
                if(i==poses.size()-1){
                    ROS_ERROR("reuleaux_bp_to_nav: Couldn't reach any of the poses");
                    res.success=false;
                    res.message="Couldn't reach any of the poses";
                }
                ROS_DEBUG("Can't move to base pose %d : (%f,%f,%f) (%f,%f,%f,%f) - attempting next pose",i,poses[i].position.x, poses[i].position.y, poses[i].position.z, poses[i].orientation.x,poses[i].orientation.y, poses[i].orientation.z,poses[i].orientation.w);
                ROS_INFO("reuleaux_bp_to_nav: pose %d failed - attemping next pose",i+1);
            }
        }
        ROS_INFO("reuleaux_bp_to_nav: Resetting the poses array and the node to be used again");
        poses.clear();
        poses.push_back(empty_pose);

        receive=true;
        service_avail=false;
    }else{
        ROS_ERROR("reuleaux_bp_to_nav: Service not available at the moment - no poses stored");
        res.success=false;
        res.message="Service not available at the moment - no poses stored";
    }
    return true;
}

geometry_msgs::Pose arm_to_base(geometry_msgs::Pose arm_pose){ 
    //// assuming the pose given by reuleaux for the arm base is a torso_lift_link pose
    //// fix orientation so that it is doable (only rotate about z)
    // Convert to transform
    tf2::Vector3 pos_vec(arm_pose.position.x,arm_pose.position.y,arm_pose.position.z);
    tf2::Quaternion q_arm(arm_pose.orientation.x,arm_pose.orientation.y,arm_pose.orientation.z,arm_pose.orientation.w);
    //convert quat to RPY to remove x,y rotations
    double roll, pitch, yaw;
    tf2::Matrix3x3(q_arm).getRPY(roll, pitch, yaw);
    tf2::Quaternion q_new;
    q_new.setRPY(0, 0, yaw);
    q_new.normalize();
    //store in transform
    tf2::Transform arm_trns;
    arm_trns.setOrigin(pos_vec);
    arm_trns.setRotation(q_new);
    //create torso_lift_link to torso_footprint transform
    tf2::Vector3 vec(0.062, 0, -0.888);
    tf2::Quaternion quat(0,0,0,1);
    tf2::Transform frame_transf;
    frame_transf.setOrigin(vec);
    frame_transf.setRotation(quat);
    //apply the transform
    tf2::Transform base_trns;
    base_trns = arm_trns*frame_transf;
    tf2::Vector3 base_vec;
    base_vec = base_trns.getOrigin();
    tf2::Quaternion base_quat;
    base_quat= base_trns.getRotation();
    base_quat.normalize();

    geometry_msgs::Pose base_pose;
    base_pose.position.x=base_vec[0];
    base_pose.position.y=base_vec[1];
    base_pose.position.z=base_vec[2];
    base_pose.orientation.x=base_quat[0];
    base_pose.orientation.y=base_quat[1];
    base_pose.orientation.z=base_quat[2];
    base_pose.orientation.w=base_quat[3];
    //// the z value is how much we want the torso to move up - !! stay within joint ranges
    torso_value=base_pose.position.z;
    if (torso_value < 0.03) {
        torso_value = 0.03;
    } else if (torso_value >= 0.34) {
        torso_value = 0.34;
    }
    //// move the position on the ground (into base_footprint)
    base_pose.position.z = 0.0;
    ////return the pose
    return base_pose;
}

bool move_torso(){
    // Client for action to control the torso joint
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> ac("/torso_controller/follow_joint_trajectory", true);
    if (!ac.waitForServer(ros::Duration(5.0))) {
        ROS_ERROR("Server torso_controller/follow_joint_trajectory not available.");
        return false;
    }

    // create action goal
    control_msgs::FollowJointTrajectoryGoal goal;
    goal.trajectory.joint_names.push_back("torso_lift_joint");
    trajectory_msgs::JointTrajectoryPoint point;
    point.positions.push_back(torso_value);
    point.time_from_start = ros::Duration(2.0);
    goal.trajectory.points.push_back(point);

    // Send goal and wait for result
    ac.sendGoal(goal);
    bool finished_before_timeout = ac.waitForResult(ros::Duration(5.0));

    // make sure action finished correctly
    if (finished_before_timeout) {
        actionlib::SimpleClientGoalState state = ac.getState();
        if (state == actionlib::SimpleClientGoalState::SUCCEEDED) {
            return true;
        } else {
            ROS_WARN("Failed torso movement: %s", state.toString().c_str());
            return false;
        }
    } else {
        ROS_ERROR("Timeout exceeded.");
        return false;
    }
}

bool send_arm_goal_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    if(service_avail){
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
        //wait for the action server to come up
        while(!ac.waitForServer(ros::Duration(5.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        for(int i=0;i<poses.size();i++){
            move_base_msgs::MoveBaseGoal goal;
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();

            goal.target_pose.pose=arm_to_base(poses[i]); //change the arm pose into a base pose to send to the navigation

            ROS_INFO("Sending goal");
            ac.sendGoal(goal);

            ac.waitForResult();

            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                ROS_DEBUG("Moved base to pose %d : (%f,%f,%f) (%f,%f,%f,%f)",i,goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
                res.success=true;
                std::string appo= "Reached pose "+std::to_string(i+1)+" (torso value= "+std::to_string(torso_value)+") ("+std::to_string(goal.target_pose.pose.position.x)+","+std::to_string(goal.target_pose.pose.position.y)+","+std::to_string(goal.target_pose.pose.position.z)+") ("+std::to_string(goal.target_pose.pose.orientation.x)+","+std::to_string(goal.target_pose.pose.orientation.y)+","+std::to_string(goal.target_pose.pose.orientation.z)+","+std::to_string(goal.target_pose.pose.orientation.w)+")";
                res.message= appo;
                if (move_torso()){
                    ROS_INFO("reuleaux_bp_to_nav: torso position adjusted to meet arm base pose");
                }else{
                    ROS_ERROR("failed to position the torso, retry manually");
                }
                
                break;
            }else{
                if(i==poses.size()-1){
                    ROS_ERROR("reuleaux_bp_to_nav:  Couldn't reach any of the poses");
                    res.success=false;
                    res.message="Couldn't reach any of the poses";
                }
                ROS_DEBUG("Can't move to base pose %d : (%f,%f,%f) (%f,%f,%f,%f) - attempting next pose",i,goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, goal.target_pose.pose.orientation.x,goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z,goal.target_pose.pose.orientation.w);
                ROS_INFO("reuleaux_bp_to_nav: pose %d failed - attemping next pose",i+1);
            }
        }
        ROS_INFO("reuleaux_bp_to_nav:  Resetting the poses array and the node to be used again");
        poses.clear();
        poses.push_back(empty_pose);

        receive=true;
        service_avail=false;
    }else{
        ROS_ERROR("reuleaux_bp_to_nav:  Service not available at the moment - no poses stored");
        res.success=false;
        res.message="Service not available at the moment - no poses stored";
    }
    return true;
}

bool reset_cb(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res){
    ROS_DEBUG("Resetting the node reuleaux_bp_to_nav:");
    service_avail=false;
    ROS_DEBUG("/reuleaux_bp_to_nav/move_to_bp service no longer available");
    poses.clear();
    poses.push_back(empty_pose);
    receive=true;
    ROS_DEBUG("Base poses array emptied - ready to receive new poses");
    return true;
}

int main(int argc, char **argv){
    ros::init(argc, argv, "reuleaux_bp_to_nav");
    ros::NodeHandle n; 
    
    poses.push_back(empty_pose); //store the first element as empty, it will be filled with the best pose later
    ros::Subscriber bp_sub = n.subscribe("reuleaux_bp_to_nav/bp_poses", 1000, bp_sub_cb);
    receive=true;
    service_avail=false;
    ros::ServiceServer send_goal_srv = n.advertiseService("reuleaux_bp_to_nav/move_to_bp_robot",send_robot_goal_cb);
    ros::ServiceServer send_arm_goal_srv = n.advertiseService("reuleaux_bp_to_nav/move_to_bp_arm",send_arm_goal_cb);
    ros::ServiceServer reset_srv = n.advertiseService("reuleaux_bp_to_nav/reset_bp_to_nav",reset_cb);
    ROS_INFO("reuleaux_bp_to_nav: READY TO RECIEVE BASE POSES");
    ros::spin();

    return 0;
}