// This file implements a ROS node to test the Inverse Kinematics functionality with MoveIt.


#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/Pose.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>


#include<moveit/robot_model_loader/robot_model_loader.h>
#include<moveit/robot_model/robot_model.h>
#include <moveit/robot_state/conversions.h>

#include<tf2/LinearMath/Transform.h>
#include<tf2/LinearMath/Quaternion.h>


#include <cmath>
class MoveItIKTest
{
public:
    MoveItIKTest(std::string group_name, bool check_collision) : group_name_(group_name), check_collision_(check_collision) 
    {
        // Initialize the ROS node
        ros::NodeHandle nh;
        group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(group_name_);
        planning_frame_ =  group_->getPlanningFrame();

        client_ = nh.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
    }


    int getValidIKCount(const geometry_msgs::Pose base_pose,/*const std::string& root_link, */moveit::core::RobotStatePtr robot_state, const geometry_msgs::Pose grasp_pose/*, bool arm_base*/)
    {
        ////ROS_INFO("///////////////ENTERED getValidIKCount//////////////////////");
        // Aggiorna lo stato del robot con la posizione della base
        Eigen::Isometry3d base_pose_eigen;
        tf::poseMsgToEigen(base_pose, base_pose_eigen);
//        robot_state->updateStateWithLinkAt(root_link, base_pose_eigen, arm_base); // forse per farlo da arm_base serve backward=true

        std::vector<double> vj_pos;
        if(robot_state->getJointModel("virtual_joint")->getTypeName() == "Planar"){
            vj_pos.resize(3);
            vj_pos[0] = base_pose.position.x;
            vj_pos[1] = base_pose.position.y;
            tf2::Quaternion q(base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            vj_pos[2] = yaw; // theta angle
        }else if(robot_state->getJointModel("virtual_joint")->getTypeName() == "Floating"){
            vj_pos.resize(7);
            vj_pos[0] = base_pose.position.x;
            vj_pos[1] = base_pose.position.y;
            vj_pos[2] = base_pose.position.z;
            vj_pos[3] = base_pose.orientation.x;
            vj_pos[4] = base_pose.orientation.y;
            vj_pos[5] = base_pose.orientation.z;
            vj_pos[6] = base_pose.orientation.w;
            
        }
        //robot_state->setToDefaultValues();///////////////////////
        robot_state->setJointPositions("virtual_joint", vj_pos);
        robot_state->update();
        ////ROS_INFO("robot state updated with base pose");
        robot_state->printStateInfo();

        std::vector<std::string> joint_names = group_->getJointNames();
        ROS_INFO("JOINT NAMES DEBUG");
        for(int i = 0; i < joint_names.size(); ++i)
        {
            ROS_INFO("joint name %d : %s",i, joint_names[i].c_str());
        }


        if(joint_names[0]=="arm_1_joint"){// sto considerando solo l'arm
            ROS_INFO("fix torso val");
            const double* torso_pos = robot_state->getJointPositions("torso_lift_joint");
            //double torso_val=torso_pos;
            ROS_INFO("og torso val: %f",torso_pos[0]);
            
            std::vector<double> torso_val_vec;
            torso_val_vec.push_back(double(torso_pos[0]+base_pose.position.z));
            ROS_INFO("final torso val: %f",torso_val_vec[0]);
            robot_state->setJointPositions("torso_lift_joint", torso_val_vec);
            robot_state->update();
            robot_state->printStateInfo();
            
        }

        ////ROS_INFO("CREATING REQUEST");
        //CREA LA REQUEST
        moveit_msgs::PositionIKRequest req;
        geometry_msgs::PoseStamped pose_st;
        pose_st.header.frame_id = planning_frame_;
        pose_st.pose = grasp_pose;
        req.pose_stamped = pose_st;
        req.group_name = group_name_;
        req.avoid_collisions = check_collision_;
        //req.attempts = 10;
        req.timeout.fromSec(0.1);
        
        
        ////ROS_INFO("REQUEST CREATED - start iteration of call ik srvs");


        std::vector<std::vector<double>> all_joint_values;
        const moveit::core::JointModelGroup* joint_model_group_ = robot_state->getJointModelGroup(group_name_);
        for (int i = 0; i < 1; ++i) // Itera su un numero arbitrario di configurazioni
        {   
            robot_state->setToRandomPositions(joint_model_group_);///////////////////////
            //robot_state->setJointPositions("virtual_joint", vj_pos);
            robot_state->update();
            robot_state->printStateInfo();
            moveit::core::robotStateToRobotStateMsg(*robot_state, req.robot_state);
            moveit_msgs::GetPositionIK srv;
            srv.request.ik_request = req;
            ////ROS_INFO("CALL %d", i);
            if(client_.call(srv)){
                ////ROS_INFO("error code: %d", srv.response.error_code.val);
                if(srv.response.error_code.val == 1){ // IK success
                    // get valori dei joint
                    ////ROS_INFO("IK SUCCESS - get joint values");
                    
                    std::vector<std::string> full_names = srv.response.solution.joint_state.name;
                    ROS_INFO("FULL NAME DEBUG");
                    for(int i=0;i<full_names.size();++i)
                    {
                        
                        ROS_INFO("joint name: %s - joint value: %f", full_names[i].c_str(), srv.response.solution.joint_state.position[i]);
                    }
                    
                    std::vector<double> joint_solution;
                    for(int i=0;i<joint_names.size();++i)
                    {
                        int position = std::find(full_names.begin(), full_names.end(), joint_names[i]) - full_names.begin();
                        
                        //approximate value to the 5 decimal point
                        ////ROS_INFO("joint name: %s - joint value: %f", joint_names[i].c_str(), srv.response.solution.joint_state.position[position]);
                        double joint_val = std::trunc(srv.response.solution.joint_state.position[position] * 1000) / 1000;
                        joint_solution.push_back(joint_val);
                    }
                    // se non coincide con un altra joint solution che ho in all_joint_value allora la aggiungo a all_joint_values
                    bool is_new_solution = true;
                    for (const auto& existing_solution : all_joint_values) {
                        if (joint_solution == existing_solution) {
                            is_new_solution = false;
                            ////ROS_INFO("Duplicate solution found, not adding to the list.");
                            break;
                        }
                    }
                    if (is_new_solution) {
                        //ROS_INFO("New solution found, adding to the list.");
                        //ROS_INFO("joint solution: %f %f %f %f %f %f %F", joint_solution[0], joint_solution[1], joint_solution[2], joint_solution[3], joint_solution[4], joint_solution[5], joint_solution[6]);
                        all_joint_values.push_back(joint_solution);
                    }
                    
                }else{
                    ROS_INFO("IK FAIL - error code: %d", srv.response.error_code.val);
                }
            }else{
                ROS_ERROR("Failed to call IK service");
                ros::Duration(5).sleep();
                return 0;
            }
        }
        return all_joint_values.size();
    }

private:
    std::string group_name_;
    bool check_collision_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> group_;
    ros::ServiceClient client_;
    std::string planning_frame_;
    
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "moveit_ik_test");
    MoveItIKTest ik_test("arm_torso",true);

    // Define a target pose (example values)
    geometry_msgs::Pose target_pose;
    target_pose.position.x = 0.588; //0.8
    target_pose.position.y = -1.3; //-1.3
    target_pose.position.z = 1.03;
    target_pose.orientation.x = 0; 
    target_pose.orientation.y = 0; 
    target_pose.orientation.z = -std::sqrt(0.5);
    target_pose.orientation.w = std::sqrt(0.5); 
    
    geometry_msgs::Pose base_pose;
    base_pose.position.x = 0.8; //0.8
    base_pose.position.y = -0.7; // -0.062
    base_pose.position.z = 0.11; // 0.889
    base_pose.orientation.x = 0; 
    base_pose.orientation.y = 0; 
    base_pose.orientation.z = -std::sqrt(0.5); 
    base_pose.orientation.w = std::sqrt(0.5); 

    std::string base_frame = "base_footprint"; // torso_lift_link base_footprint 
    
    moveit::core::RobotModelConstPtr robot_model_;
    moveit::core::RobotStatePtr robot_state_;

    robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
    if(robot_model_loader.getModel() ==NULL){
        ROS_ERROR("failed to load robot model");
        return 0;
    }
    robot_model_ = robot_model_loader.getModel();
    robot_state_ = std::make_shared<moveit::core::RobotState>(robot_model_);

    robot_state_->setToDefaultValues();
    std::vector<double> torso_pos;
    torso_pos.push_back(0.150);
    robot_state_->setJointPositions("torso_lift_joint", torso_pos);
    robot_state_->update();
    robot_state_->printStateInfo();


    
    // Test the IK
    int res= ik_test.getValidIKCount(base_pose, /*base_frame ,*/ robot_state_, target_pose/*, true*/);
    
    ROS_INFO("Number of valid IK solutions: %d", res);
    ros::spin();
    return 0;
}