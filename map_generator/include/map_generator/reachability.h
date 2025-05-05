#ifndef REACHABILITY_H
#define REACHABILITY_H
#include <ros/ros.h>
#include <map_generator/WorkSpace.h>
#include <map_generator/utility.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/PositionIKRequest.h>
#include <moveit/move_group_interface/move_group_interface.h>

#include<moveit/robot_model/joint_model_group.h>
#include<moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <geometry_msgs/Pose.h>
#include<tf2/LinearMath/Quaternion.h>
#include<tf2/LinearMath/Transform.h>
#include <cmath>

namespace reachability
{
class ReachAbility
{
public:
  ReachAbility(ros::NodeHandle& node, std::string group_name, bool check_collision);
  void setInitialWorkspace(const map_generator::WorkSpace& initial_ws);
  void getFinalWorkspace(map_generator::WorkSpace& final_ws);
  bool getIKSolution(const geometry_msgs::Pose& pose, std::string ref_frame, moveit_msgs::RobotState& robot_state);  
  bool createReachableWorkspace();

  int getValidIKCount(const geometry_msgs::Pose base_pose, moveit::core::RobotStatePtr robot_state_ptr, const geometry_msgs::Pose grasp_pose);
  std::vector<double> getValidIKSol(const geometry_msgs::Pose base_pose, moveit::core::RobotStatePtr robot_state_ptr, const geometry_msgs::Pose grasp_pose);


private:
  std::string group_name_;
  bool check_collision_;
  moveit_msgs::PositionIKRequest makeServiceRequest(const geometry_msgs::Pose &pose_in, std::string ref_frame);
  bool ik(const moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState& robot_state);

  bool createReachability(const map_generator::WorkSpace& ws);


  boost::scoped_ptr<moveit::planning_interface::MoveGroupInterface> group_;
  std::string planning_frame_;
  moveit_msgs::GetPositionIK srv_;

  ros::NodeHandle nh_;
  ros::ServiceClient client_;
  map_generator::WorkSpace init_ws_;
  map_generator::WorkSpace final_ws_;
  int pose_size_;
  int sphere_size_;


};

}//end namespace 

#endif // REACHABILITY_H
