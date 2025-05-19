#include<map_generator/reachability.h>

namespace reachability
{
ReachAbility::ReachAbility(ros::NodeHandle& node, std::string group_name, bool check_collision)
  :group_name_(group_name), check_collision_(check_collision)
{
  nh_ = node;
  client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("/compute_ik");
  group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
  planning_frame_ = group_->getPlanningFrame();
  final_ws_.WsSpheres.clear();
  init_ws_.WsSpheres.clear();

}

 moveit_msgs::PositionIKRequest ReachAbility::makeServiceRequest(const geometry_msgs::Pose &pose_in, std::string ref_frame)
 {
   moveit_msgs::PositionIKRequest req;
   geometry_msgs::PoseStamped pose_st;
   pose_st.header.frame_id = ref_frame;
   pose_st.pose = pose_in;
   req.pose_stamped = pose_st;
   req.group_name = group_name_;
   req.avoid_collisions = check_collision_;
   // req.attempts = 10;
   req.timeout.fromSec(0.1);
   return req;
 }

 void ReachAbility::setInitialWorkspace(const map_generator::WorkSpace &initial_ws)
 {
   init_ws_ = initial_ws;
   utility::getPoseAndSphereSize(init_ws_, sphere_size_, pose_size_);
 }

 void ReachAbility::getFinalWorkspace(map_generator::WorkSpace &final_ws)
 {
   final_ws = final_ws_;
 }

 
 bool ReachAbility::ik(const moveit_msgs::PositionIKRequest& req, moveit_msgs::RobotState &robot_state)
 {
   srv_.request.ik_request = req;
   if(client_.call(srv_))
   {
     if(srv_.response.error_code.val == 1)
     {
       robot_state = srv_.response.solution;
       return true;
     }
     else
       return false;
    }
   else
   {
     ROS_ERROR("Failed to call IK service");
           return 1;
   }
 }

 bool ReachAbility::getIKSolution(const geometry_msgs::Pose &pose, std::string ref_frame, moveit_msgs::RobotState &robot_state) // this is the one used for the RM generation
 {
   //ROS_DEBUG("===============================");
   //ROS_DEBUG("Requesting IK solution for...");
   //ROS_DEBUG("Position: x: %f y: %f z: %f", pose.position.x, pose.position.y, pose.position.z);
   //ROS_DEBUG("Orientation: qx: %f qy: %f qz: %f qw: %f", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
   moveit_msgs::PositionIKRequest req = makeServiceRequest(pose,ref_frame);
   if(ik(req, robot_state))
     return true;
   else
     return false;
 }

 
 bool ReachAbility::createReachableWorkspace()
 {
   if(createReachability(init_ws_))
     return true;
   else
     return false;
 }

 bool ReachAbility::createReachability(const map_generator::WorkSpace& ws)
 {
  utility::MultiMap ws_map;
   utility::MapVecDouble sp_map;
   int sp_size = sphere_size_;
   for(int i=0;i<sp_size;++i){
     ROS_INFO("Processing sphere: %d / %d", i+1,sp_size);
     std::vector<double> sp_vec;
     utility::pointToVector(ws.WsSpheres[i].point, sp_vec);
     for(int j=0;j<ws.WsSpheres[i].poses.size();++j){
       moveit_msgs::RobotState state;
       geometry_msgs::Pose reach_pose= ws.WsSpheres[i].poses[j];
       bool is_reachable = getIKSolution(reach_pose, planning_frame_, state);
       if(is_reachable){
         ////ROS_DEBUG("SUCCESS: Pose was reached!");
         std::vector<double> sp_pose;
         utility::poseToVector(reach_pose, sp_pose);
         ws_map.insert(std::make_pair(sp_vec, sp_pose));
       } else {
         ////ROS_DEBUG("FAIL: Pose was not reached");
       }
       /////ROS_DEBUG("===============================");
     }
   }
   for(utility::MultiMap::iterator it=ws_map.begin(); it!=ws_map.end();++it)
   {
     std::vector<double> sp_coord = it->first;
     float d = float(ws_map.count(sp_coord)) / (pose_size_ /sphere_size_) * 100;
     sp_map.insert(std::make_pair(it->first, double(d)));
   }

   for (utility::MapVecDouble::iterator it = sp_map.begin(); it != sp_map.end(); ++it)
   {
     map_generator::WsSphere wss;
     wss.point.x = (it->first)[0];
     wss.point.y = (it->first)[1];
     wss.point.z = (it->first)[2];
     wss.ri = it->second;
     for (utility::MultiMap::iterator it1 = ws_map.lower_bound(it->first); it1 != ws_map.upper_bound(it->first); ++it1)
     {
       geometry_msgs::Pose pp;
       pp.position.x = (it1->second)[0];
       pp.position.y = (it1->second)[1];
       pp.position.z = (it1->second)[2];
       pp.orientation.x = (it1->second)[3];
       pp.orientation.y = (it1->second)[4];
       pp.orientation.z = (it1->second)[5];
       pp.orientation.w = (it1->second)[6];
       wss.poses.push_back(pp);
     }
   final_ws_.WsSpheres.push_back(wss);
   }
   final_ws_.resolution = init_ws_.resolution;

   // Added as there was no return from this function, not sure if there ought to be a false case...
   return true;
 }


 int ReachAbility::getValidIKCount(const geometry_msgs::Pose base_pose, moveit::core::RobotStatePtr robot_state_ptr, const geometry_msgs::Pose grasp_pose)
  {
    ////ROS_DEBUG("getValidIKCount - start ");/////
    moveit::core::RobotState robot_state = *robot_state_ptr;
    //ROS_INFO("getValidIKCount - robot state received:"); ////
    ////robot_state.printStateInfo();

    std::vector<std::string> joint_names = group_->getJointNames();
    const moveit::core::JointModelGroup* joint_model_group_ = robot_state.getJointModelGroup(group_name_);

    // The robot state received needs to be a VALID state
    // Create the new state for the virtual_joint - to be used in the robot state for the ik
    Eigen::Isometry3d base_pose_eigen;
    tf::poseMsgToEigen(base_pose, base_pose_eigen);
    ////ROS_DEBUG("getValidIKCount - base pose = %f %f %f %f %f %f %f", base_pose.position.x, base_pose.position.y, base_pose.position.z, base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w); //////
    ////ROS_DEBUG("getValidIKCount - grasp pose = %f %f %f %f %f %f %f", grasp_pose.position.x, grasp_pose.position.y, grasp_pose.position.z, grasp_pose.orientation.x, grasp_pose.orientation.y, grasp_pose.orientation.z, grasp_pose.orientation.w); //////
    std::vector<double> vj_pos;
    if(robot_state.getJointModel("virtual_joint")->getTypeName() == "Planar"){
        vj_pos.resize(3);
        vj_pos[0] = base_pose.position.x;
        vj_pos[1] = base_pose.position.y;
        tf2::Quaternion q(base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        vj_pos[2] = yaw; // theta angle
        ////ROS_DEBUG("getValidIKCount - vj_pos PLANAR = %f %f %f ", vj_pos[0], vj_pos[1], vj_pos[2]);/////
    }else if(robot_state.getJointModel("virtual_joint")->getTypeName() == "Floating"){
        vj_pos.resize(7);
        vj_pos[0] = base_pose.position.x;
        vj_pos[1] = base_pose.position.y;
        vj_pos[2] = base_pose.position.z;
        vj_pos[3] = base_pose.orientation.x;
        vj_pos[4] = base_pose.orientation.y;
        vj_pos[5] = base_pose.orientation.z;
        vj_pos[6] = base_pose.orientation.w;
        ////ROS_DEBUG("getValidIKCount - vj_pos FLOATING = %f %f %f %f %f %f %f", vj_pos[0], vj_pos[1], vj_pos[2],vj_pos[3], vj_pos[4], vj_pos[5], vj_pos[6]);/////
    }
    robot_state.setJointPositions("virtual_joint", vj_pos);

    if(joint_names[0]=="arm_1_joint"){ // //// MANINO PER FAR FUNZIONARE BENE CON ARM
      const double* torso_val = robot_state.getJointPositions("torso_lift_joint");
      std::vector<double> torso_val_vec;
      torso_val_vec.push_back(double(torso_val[0]+base_pose.position.z));
      robot_state.setJointPositions("torso_lift_joint", torso_val_vec);
    }

    robot_state.update();

    //Create and start the setup for the request
    moveit_msgs::PositionIKRequest req;
    geometry_msgs::PoseStamped pose_st;
    pose_st.header.frame_id = planning_frame_;
    pose_st.pose = grasp_pose;
    req.pose_stamped = pose_st;
    req.group_name = group_name_;
    req.avoid_collisions = check_collision_;
    req.timeout.fromSec(0.1);
    //ROS_INFO("getValidIKCount - request: frame: %s pose %f %f %f %f %f %f %f",req.pose_stamped.header.frame_id.c_str(), req.pose_stamped.pose.position.x, req.pose_stamped.pose.position.y, req.pose_stamped.pose.position.z, req.pose_stamped.pose.orientation.x, req.pose_stamped.pose.orientation.y, req.pose_stamped.pose.orientation.z, req.pose_stamped.pose.orientation.w); //////
    //ROS_INFO("getValidIKCount - request: group name %s", req.group_name.c_str()); //////
    //ROS_INFO("getValidIKCount - request: avoid collisions %d", req.avoid_collisions); //////
    std::vector<std::vector<double>> all_joint_values;
    
    for (int i = 0; i < 50; ++i) // iterate an arbitrary number of times
    {   
      // Update the robot state with random values for the manipulator joints and the base pose for the virtual joint
      robot_state.setToRandomPositions(joint_model_group_);
      robot_state.update();
      ////robot_state.printStateInfo();
      moveit::core::robotStateToRobotStateMsg(robot_state, req.robot_state); // insert the robot state in the request
      
      moveit_msgs::GetPositionIK srv;
      srv.request.ik_request = req; 
      if(client_.call(srv)){
          
        if(srv.response.error_code.val == 1){ // IK success
          // save the obtained joint values for the maniuplator
          std::vector<std::string> full_names = srv.response.solution.joint_state.name;
          std::vector<double> joint_solution;
          for(int i=0;i<joint_names.size();++i)
          {
              int position = std::find(full_names.begin(), full_names.end(), joint_names[i]) - full_names.begin();
              //approximate value to the 3 decimal point - to avoid having the same solution with different values
              double joint_val = std::trunc(srv.response.solution.joint_state.position[position] * 1000) / 1000;
              joint_solution.push_back(joint_val);
          }
          // check that the solution obtained now is NOT one we already have ////////////////////////////// 
          bool is_new_solution = true;
          for (const auto& existing_solution : all_joint_values) {
              if (joint_solution == existing_solution) {
                  is_new_solution = false;
                  ////ROS_INFO("Duplicate solution found, not adding to the list.");
                  break;
              }
          }
          if (is_new_solution) {
            //save the solution found
            /////ROS_INFO("joint solution: %f %f %f %f %f %f %F", joint_solution[0], joint_solution[1], joint_solution[2], joint_solution[3], joint_solution[4], joint_solution[5], joint_solution[6]);
            all_joint_values.push_back(joint_solution);
          }
          
        }else{
            ////ROS_DEBUG("IK FAIL - error code: %d", srv.response.error_code.val);
        }
      }else{
        ROS_ERROR("Failed to call IK service");
        return -999999;
      }
    }
    ////ROS_DEBUG("getValidIKCount - return: %d", all_joint_values.size());/////
    return all_joint_values.size();
  }

std::vector<double> ReachAbility::getValidIKSol(const geometry_msgs::Pose base_pose, moveit::core::RobotStatePtr robot_state_ptr, const geometry_msgs::Pose grasp_pose)
  {
    moveit::core::RobotState robot_state = *robot_state_ptr;

    std::vector<std::string> joint_names = group_->getJointNames();
    const moveit::core::JointModelGroup* joint_model_group_ = robot_state.getJointModelGroup(group_name_);

    // The robot state received needs to be a VALID state
    // Create the new state for the virtual_joint - to be used in the robot state for the ik
    ////ROS_INFO("getValidIKSol - base pose: %f %f %f %f %f %f %f ",base_pose.position.x,base_pose.position.y,base_pose.position.z,base_pose.orientation.x,base_pose.orientation.y,base_pose.orientation.z,base_pose.orientation.w);////
    /////rmv///Eigen::Isometry3d base_pose_eigen;
    /////rmv///tf::poseMsgToEigen(base_pose, base_pose_eigen);
    std::vector<double> vj_pos;
    if(robot_state.getJointModel("virtual_joint")->getTypeName() == "Planar"){
        vj_pos.resize(3);
        vj_pos[0] = base_pose.position.x;
        vj_pos[1] = base_pose.position.y;
        tf2::Quaternion q(base_pose.orientation.x, base_pose.orientation.y, base_pose.orientation.z, base_pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        vj_pos[2] = yaw; // theta angle
        ////ROS_INFO("getValidIKSol - VJ= %f %f %f", vj_pos[0],vj_pos[1],vj_pos[2]); ///////
    }else if(robot_state.getJointModel("virtual_joint")->getTypeName() == "Floating"){
        vj_pos.resize(7);
        vj_pos[0] = base_pose.position.x;
        vj_pos[1] = base_pose.position.y;
        vj_pos[2] = base_pose.position.z;
        vj_pos[3] = base_pose.orientation.x;
        vj_pos[4] = base_pose.orientation.y;
        vj_pos[5] = base_pose.orientation.z;
        vj_pos[6] = base_pose.orientation.w;
    }
    robot_state.setJointPositions("virtual_joint", vj_pos);

    if(joint_names[0]=="arm_1_joint"){ // //// MANINO PER FAR FUNZIONARE BENE CON ARM
      const double* torso_val = robot_state.getJointPositions("torso_lift_joint");
      std::vector<double> torso_val_vec;
      torso_val_vec.push_back(double(torso_val[0]+base_pose.position.z));
      if(torso_val_vec[0]>0.34){
        torso_val_vec[0]=0.34;
      }
      robot_state.setJointPositions("torso_lift_joint", torso_val_vec);
    }

    robot_state.update();


    //Create and start the setup for the request
    moveit_msgs::PositionIKRequest req;
    geometry_msgs::PoseStamped pose_st;
    pose_st.header.frame_id = planning_frame_;
    pose_st.pose = grasp_pose;
    req.pose_stamped = pose_st;
    req.group_name = group_name_;
    req.avoid_collisions = check_collision_;
    req.timeout.fromSec(0.1);

    std::vector<double> joint_solution;
    
    for (int i = 0; i < 4; ++i) // iterate an arbitrary number of times
    {   
      // Update the robot state with random values for the manipulator joints and the base pose for the virtual joint
      robot_state.setToRandomPositions(joint_model_group_);
      robot_state.update();
      ////robot_state.printStateInfo();
      moveit::core::robotStateToRobotStateMsg(robot_state, req.robot_state); // insert the robot state in the request
      
      moveit_msgs::GetPositionIK srv;
      srv.request.ik_request = req; 
      if(client_.call(srv)){
          
        if(srv.response.error_code.val == 1){ // IK success
          // save the obtained joint values for the maniuplator
          std::vector<std::string> full_names = srv.response.solution.joint_state.name;
          for(int i=0;i<joint_names.size();++i)
          {
              int position = std::find(full_names.begin(), full_names.end(), joint_names[i]) - full_names.begin();
              //approximate value to the 3 decimal point - to avoid having the same solution with different values
              double joint_val = std::trunc(srv.response.solution.joint_state.position[position] * 1000) / 1000;
              joint_solution.push_back(joint_val);
          }
          // return the first valid solution that is found
          return joint_solution;

        }else{
            ////ROS_DEBUG("IK FAIL - error code: %d", srv.response.error_code.val);
        }
      }else{
        ROS_ERROR("Failed to call IK service");
        joint_solution.clear(); //just to make sure but SHOULD NOT be needed
        joint_solution.push_back(-999999); // to be interpreted as a error message when this function is used
        return joint_solution;
      }
    }
    joint_solution.clear(); //just to make sure but SHOULD NOT be needed
    return joint_solution; // no solution found
  }

}
