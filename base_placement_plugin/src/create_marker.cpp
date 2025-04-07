#include <base_placement_plugin/create_marker.h>
#include <ctime>
#include<typeinfo>
#include <tf2_ros/transform_listener.h> 

double unifRand()
{
    return rand() / double(RAND_MAX);
}

CreateMarker::CreateMarker(std::string group_name) : spinner(1), group_name_(group_name)
{
  spinner.start();
  group_.reset(new MoveGroupInterface(group_name_));
  ////ROS_INFO_STREAM("Selected planning group: "<< group_->getName());
  robot_model_ = group_->getRobotModel();

/* ///// VERS 1 of updateRobotState - not in use

  // Crea un buffer TF2 e un listener
  auto tf_buffer = std::make_shared<tf2_ros::Buffer>();
  tf2_ros::TransformListener tf_listener(*tf_buffer);
  planning_scene_monitor_.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description", tf_buffer));
  if(!planning_scene_monitor_->getPlanningScene())
  {
    ROS_ERROR("Planning scene not configured");
    return;
  }
    ROS_INFO("Planning scene configured");
    planning_scene_monitor_->startSceneMonitor();
    planning_scene_monitor_->startWorldGeometryMonitor();
    planning_scene_monitor_->startStateMonitor();
*/
  
  if (nh.getParam("BPP_fixed_frame", fixed_frame_)) {
      ROS_DEBUG("CM - Received fixed frame for plugin: %s", fixed_frame_.c_str());
  } else {
      ROS_WARN("CM - Failed to get param 'param_name' - setting to defualt: 'world'");
      fixed_frame_ = "world"; ////////////////
  }
}

bool CreateMarker::checkEndEffector()
{
  if(!(group_->getEndEffector()).empty())
    return true;
  else
    return false;
}

void CreateMarker::discardUnreachableModels(BasePoseJoint& baseJoints)
{
  for(BasePoseJoint::iterator it= baseJoints.begin(); it !=baseJoints.end();)
  {
    std::vector<double> joint_soln = it->first;
    if(std::equal(joint_soln.begin()+1, joint_soln.end(), joint_soln.begin()))
    {
      BasePoseJoint::iterator save = it;
      ++save;
      baseJoints.erase(it);
      it = save;
    }
    else
      ++it;
  }
}

bool checkForJointSoln(const std::vector<double>& soln)
{
  if(std::equal(soln.begin()+1, soln.end(), soln.begin()))
    return true;
}

void CreateMarker::jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_state){
  joint_state_update_ = *joint_state;
  ROS_INFO_STREAM("Joint state received");
}

void CreateMarker::updateRobotState(const std::vector<double>& joint_soln, moveit::core::RobotStatePtr robot_state, bool arm_only)
{
  if(!arm_only){

  /* ////VERS 1 for robot state update - using planning scene monitor - gave issues
    if(!planning_scene_monitor_){
      ROS_ERROR("Planning scene not configured");
      return;
    }

    planning_scene_monitor_->getPlanningScene()->getCurrentStateNonConst().update();
    planning_scene_monitor_->updateFrameTransforms();
    const moveit::core::RobotState& current_state = planning_scene_monitor_->getPlanningScene()->getCurrentState();
    current_state.printStateInfo(); 
    
    // Get all joint names
    const std::vector<std::string>& joint_names = current_state.getVariableNames();
    int start=0; // default> no virtual joints or fixed vj 
    
    // check for virtual joints variables that need to be handled differently
    if(joint_names[0] == "virtual_joint/trans_x"){ // floating joint - variables: trans_x, trans_y, trans_z, rot_x, rot_y, rot_z, rot_w 
      start=7;      
    }else if (joint_names[0] == "virtual_joint/x"){ // planar joint - variables: x, y, theta
      start=3;
    }

    for (int i = 0; i < start; i++){  // set the virtual joint variables
      double vj_pos = current_state.getVariablePosition(joint_names[i]);
      robot_state->setVariablePosition(joint_names[i], vj_pos);
      ROS_INFO(" VJ VAR %s - %f", joint_names[i].c_str(), vj_pos);
    }
    if(start!=0){
      std::vector<double> vj_pos;
      vj_pos.resize(start);
      for (int i = 0; i < start; i++){  // set the virtual joint variables
        vj_pos[i] = current_state.getVariablePosition(joint_names[i]);
        ROS_INFO(" VJ VAR %s - %f", joint_names[i].c_str(), vj_pos);//////////////
      }
      robot_state->setJointPositions("virtual_joint", vj_pos);
    }
    for(int i=start;i<joint_names.size();i++){ // set the rest of the joint variables
      const double* joint_pos = current_state.getJointPositions(joint_names[i]);
      robot_state->setJointPositions(joint_names[i], joint_pos);
      ROS_INFO(" var %d %s - %f", i, joint_names[i].c_str(), joint_pos[0]);
    }
  */
    
    //// VERS 2 using joint state
    // GET THE VALUES OF THE JOINTS FROM THE JOINT STATE PUBLISHER
    sensor_msgs::JointState empty_state;
    joint_state_update_ = empty_state;
    joint_state_sub_ = nh.subscribe("/joint_states", 1, &CreateMarker::jointStateCallback, this);
    while (joint_state_update_.name.size() == 0){
      ROS_DEBUG("Waiting for joint state update"); /////////////////////
      ros::Duration(0.1).sleep();
    } 
    joint_state_sub_.shutdown();
    //check joint state received:
    for(int i=0;i<joint_state_update_.name.size();i++){
      //ROS_INFO_STREAM("Joint name: "<<joint_state_update_.name[i]);/////////////////////
      //ROS_INFO_STREAM("Joint position: "<<joint_state_update_.position[i]);/////////////////////
      robot_state->setJointPositions(joint_state_update_.name[i], &(joint_state_update_.position[i]));
    }
    // !!!! the joint state does NOT contain the virtual joint !!!! 
    //check if the model has a virtual joint - if so get the value and update it in the robot state
    if(robot_model_->hasJointModel("virtual_joint")){
      tf2_ros::Buffer tf_buffer;
      tf2_ros::TransformListener tf_listener(tf_buffer);
      geometry_msgs::TransformStamped transform_stamped;
      geometry_msgs::Pose root_link_pose;
      try{
        transform_stamped = tf_buffer.lookupTransform(fixed_frame_, robot_model_->getRootLinkName(), ros::Time(0), ros::Duration(3.0));
      }catch (tf2::TransformException &ex){
        ROS_WARN("%s", ex.what());
      }
      if(robot_model_->getJointModel("virtual_joint")->getTypeName() == "Planar"){
        std::vector<double> vj_pos(3);
        vj_pos[0] = transform_stamped.transform.translation.x;
        vj_pos[1] = transform_stamped.transform.translation.y;
        tf2::Quaternion q(transform_stamped.transform.rotation.x, transform_stamped.transform.rotation.y, transform_stamped.transform.rotation.z, transform_stamped.transform.rotation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        vj_pos[2] = yaw; // theta angle
        //ROS_INFO("/////// vj planar %f %f %f ", vj_pos[0],vj_pos[1],vj_pos[2]);/////////////////////
        robot_state->setJointPositions("virtual_joint", vj_pos);
      }else if(robot_model_->getJointModel("virtual_joint")->getTypeName() == "Floating"){
        std::vector<double> vj_pos(7);
        vj_pos[0] = transform_stamped.transform.translation.x;
        vj_pos[1] = transform_stamped.transform.translation.y;
        vj_pos[2] = transform_stamped.transform.translation.z;
        vj_pos[3] = transform_stamped.transform.rotation.x;
        vj_pos[4] = transform_stamped.transform.rotation.y;
        vj_pos[5] = transform_stamped.transform.rotation.z;
        vj_pos[6] = transform_stamped.transform.rotation.w;
        //ROS_INFO("/////// vj floating %f %f %f %f %f %f %f", vj_pos[0],vj_pos[1],vj_pos[2],vj_pos[3],vj_pos[4],vj_pos[5],vj_pos[6]);/////////////////////
        robot_state->setJointPositions("virtual_joint", vj_pos);
      } // if fixed there is no need to set the joint values of the virtual joint
    } 
  }

  if(joint_soln.size() != 0){ //is not done if i don't have the joint solutions - in placeBase::TransformToRobotbase, placeBase::TransformFromRobotToArmBase and createMarker::getDefaultMarkers
    std::string robot_name = group_->getName();
    const moveit::core::JointModelGroup* robot_jmp = robot_model_->getJointModelGroup(robot_name);
    std::vector<std::string> joint_names = robot_jmp->getActiveJointModelNames();
    for (int i=0;i<joint_soln.size();i++)
    {
      robot_state->setJointPositions(joint_names[i], &(joint_soln[i]));
    }
  }
  robot_state->update();
}

void CreateMarker::getArmLinks(std::vector<std::string>& arm_links)
{
  std::string arm_name = group_->getName();
  const moveit::core::JointModelGroup* arm_jmp = robot_model_->getJointModelGroup(arm_name);
  arm_links = arm_jmp->getLinkModelNames();
}

void CreateMarker::getEELinks(std::vector<std::string>& ee_links)
{
  std::string ee_name = group_->getEndEffector();
  if(robot_model_->hasJointModelGroup(ee_name))
  {
    const moveit::core::JointModelGroup* ee_jmp = robot_model_->getJointModelGroup(ee_name);
    ee_links = ee_jmp->getLinkModelNames();
  }
}

void CreateMarker::getFullLinkNames(std::vector<std::string>& full_link_names, bool arm_only)
{
  std::vector<std::string> full_links = robot_model_->getLinkModelNames();
  if(arm_only)
  {
    std::vector<std::string> arm_links, ee_links;
    getArmLinks(arm_links);
    getEELinks(ee_links);
    int position = std::find(full_links.begin(), full_links.end(), arm_links[0]) - full_links.begin();
    parent_link = full_links[position -1];
    for(int i=0;i<arm_links.size();++i)
      full_link_names.push_back(arm_links[i]);
    for(int i=0;i<ee_links.size();++i)
      full_link_names.push_back(ee_links[i]);
  }
  else
  {
    full_link_names = full_links;
    parent_link = full_link_names[0];
  }
}

void CreateMarker::updateMarkers(const geometry_msgs::Pose& base_pose, bool is_reachable, Eigen::Affine3d tf_first_link_to_root, visualization_msgs::MarkerArray& markers)
{
  Eigen::Affine3d base_tf;
  tf::poseMsgToEigen(base_pose, base_tf);
  double r = unifRand();
  double g = unifRand();
  double b = unifRand();
  for(std::size_t j=0;j<markers.markers.size();j++)
  {
    ////markers.markers[j].header.frame_id = "odom";
    markers.markers[j].header.frame_id = fixed_frame_;
    markers.markers[j].type = markers.markers[j].type;
    if(markers.markers[j].type == visualization_msgs::Marker::MESH_RESOURCE){
      markers.markers[j].mesh_use_embedded_materials = true;
    }
    
    if (markers.markers[j].type == visualization_msgs::Marker::POINTS || 
      markers.markers[j].type == visualization_msgs::Marker::LINE_STRIP || 
      markers.markers[j].type == visualization_msgs::Marker::LINE_LIST) {

      markers.markers[j].points = markers.markers[j].points;
      markers.markers[j].colors = markers.markers[j].colors;
    }
    
    markers.markers[j].id = j*5;
    markers.markers[j].header.stamp = ros::Time::now();
    markers.markers[j].ns = "robot_links";
    markers.markers[j].lifetime = ros::Duration(40.0);
    if(!is_reachable)
    {
      markers.markers[j].color.r = 1.0;
      markers.markers[j].color.g = 0.0;
      markers.markers[j].color.b = 0.0;
      markers.markers[j].color.a= 1.0;
    }
    else
    {
      markers.markers[j].color.r = r;
      markers.markers[j].color.g = g;
      markers.markers[j].color.b = b;
      markers.markers[j].color.a= 0.7;
    }
    Eigen::Affine3d link_marker;
    tf::poseMsgToEigen(markers.markers[j].pose, link_marker);
    Eigen::Affine3d tf_link_in_root = tf_first_link_to_root * link_marker;
    geometry_msgs::Pose new_marker_pose;
    tf::poseEigenToMsg(base_tf * tf_link_in_root, new_marker_pose);
    markers.markers[j].pose = new_marker_pose;
  
  }
}


void CreateMarker::makeIntMarkerControl(const geometry_msgs::Pose& base_pose, const std::vector<double>& joint_soln,bool arm_only, bool is_reachable, visualization_msgs::InteractiveMarkerControl& robotModelControl)
{
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model_));
  updateRobotState(joint_soln, robot_state, arm_only);
  std::vector<std::string> full_link_names;
  getFullLinkNames(full_link_names, arm_only);
  visualization_msgs::MarkerArray full_link_markers;
  robot_state->getRobotMarkers(full_link_markers, full_link_names);

  Eigen::Affine3d tf_root_to_first_link = robot_state->getGlobalLinkTransform(parent_link);
  Eigen::Affine3d tf_first_link_to_root = tf_root_to_first_link.inverse();

  updateMarkers(base_pose, is_reachable, tf_first_link_to_root, full_link_markers);
  for(int i=0;i<full_link_markers.markers.size();++i)
    robotModelControl.markers.push_back(full_link_markers.markers[i]);
  robotModelControl.interaction_mode = visualization_msgs::InteractiveMarkerControl::NONE;
  robotModelControl.always_visible = true;
}


void CreateMarker::createInteractiveMarker(const geometry_msgs::Pose& base_pose, const std::vector<double>& joint_soln,
                                           const int& num, bool arm_only, bool is_reachable,visualization_msgs::InteractiveMarker& iMarker)
{
  ////iMarker.header.frame_id = "odom";
  iMarker.header.frame_id = fixed_frame_;
  iMarker.pose = base_pose;
  iMarker.scale = 0.3;
  std::string name = "robot_model";
  std::string description = "robot_descr"; //perchè non è scritto robot_description?? (robot_desc)
  iMarker.name = name + boost::lexical_cast<std::string>(num);
  iMarker.description = description+boost::lexical_cast<std::string>(num);
  visualization_msgs::InteractiveMarkerControl robotModelControl;
  makeIntMarkerControl(base_pose, joint_soln, arm_only, is_reachable,robotModelControl);
  iMarker.controls.push_back(robotModelControl);
}

void CreateMarker::makeIntMarkers(BasePoseJoint &basePJoints, bool arm_only, std::vector<visualization_msgs::InteractiveMarker> &iMarkers)
{
  iMarkers.clear();
  for(BasePoseJoint::iterator it = basePJoints.begin(); it !=basePJoints.end();++it)
  {
    int i = std::distance(basePJoints.begin(), it);
    bool is_reachable = true;
    geometry_msgs::Pose base_pose = it->second;
    std::vector<double> joint_soln = it->first;
    if(checkForJointSoln(joint_soln))
      is_reachable = false;
    visualization_msgs::InteractiveMarker iMarker;
    createInteractiveMarker(base_pose, joint_soln, i, arm_only, is_reachable, iMarker );
    iMarkers.push_back(iMarker);
  }
}


bool CreateMarker::makeRobotMarker(BasePoseJoint baseJoints, std::vector<visualization_msgs::InteractiveMarker> &iMarkers, bool show_unreachable_models)
{
  if(!show_unreachable_models)
     discardUnreachableModels(baseJoints);
  makeIntMarkers(baseJoints, false, iMarkers);
}


bool CreateMarker::makeArmMarker(BasePoseJoint baseJoints, std::vector<visualization_msgs::InteractiveMarker> &iMarkers, bool show_unreachable_models)
{
  if(!show_unreachable_models)
      discardUnreachableModels(baseJoints);
  makeIntMarkers(baseJoints, true, iMarkers);
}


visualization_msgs::MarkerArray CreateMarker::getDefaultMarkers()
{
  moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model_));
  std::vector<double> joint_soln_empty;
  updateRobotState(joint_soln_empty, robot_state, false);
  std::vector<std::string> full_link_names;
  getFullLinkNames(full_link_names, false);
  visualization_msgs::MarkerArray full_link_markers;
  robot_state->getRobotMarkers(full_link_markers, full_link_names);
  return full_link_markers;
}



