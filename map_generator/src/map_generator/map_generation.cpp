#include <map_generator/map_generation.h>
#include <sys/types.h>
#include <sys/stat.h>


namespace map_generation
{
mapGeneration::mapGeneration(ros::NodeHandle& node, const std::string &group_name, const std::string &path,
                             const std::string &filename, const std::string& pkg_name, const double &resolution, const double &radius, bool check_collision, bool do_centering)
{
  nh_ = node;
  group_name_ = group_name;
  path_ = path;
  filename_ = filename;
  pkg_name_=pkg_name;
  resolution_ = resolution;
  radius_ = radius;
  check_collision_= check_collision;
  do_centering_= do_centering;
  group_.reset(new moveit::planning_interface::MoveGroupInterface(group_name_));
  init_ws_.WsSpheres.clear();
  filtered_ws_.WsSpheres.clear();
  centered_ws_.WsSpheres.clear();
}

void mapGeneration::discretizeWorkspace(geometry_msgs::Pose& pose)
{
  ROS_INFO("Discretizing workspace with resolution %f and radius %f", resolution_, radius_);
  discretization::Discretization* disc(new discretization::Discretization(pose, resolution_, radius_));
  disc->discretize();
  disc->getInitialWorkspace(init_ws_);
  utility::getPoseAndSphereSize(init_ws_, init_sp_size_, init_pose_size_);
  ROS_INFO("Initial workspace has %d spheres and %d poses", init_sp_size_, init_pose_size_);
  delete disc;
}

void mapGeneration::filterWorkspace()
{
  reachability::ReachAbility* reach(new reachability::ReachAbility(nh_,group_name_, check_collision_));
  reach->setInitialWorkspace(init_ws_);
  reach->createReachableWorkspace();
  reach->getFinalWorkspace(filtered_ws_);
  utility::getPoseAndSphereSize(filtered_ws_, final_sp_size_, final_pose_size_);
  delete reach;
}

void mapGeneration::centerWorkspace(){
  centering::Centering* centr(new centering::Centering(nh_,arm_pose_));
  centr->setInitialWorkspace(filtered_ws_);
  centr->createCenteredWorkspace();
  centr->getFinalWorkspace(centered_ws_);
  utility::getPoseAndSphereSize(centered_ws_, final_sp_size_, final_pose_size_);///////rmv
  delete centr;
}

void mapGeneration::saveWorkspace()
{
  std::string name;
  std::string filename;
  if(filename_ == "default")
  {
    filename = utility::createName(pkg_name_, group_name_, resolution_);
  } else{
    filename = filename_;
  }
  name = path_+filename;

  hdf5_dataset::Hdf5Dataset* h5(new hdf5_dataset::Hdf5Dataset(name));
  if (do_centering_){
    h5->save(centered_ws_);
  }else{
    h5->save(filtered_ws_);
  }
  ROS_INFO("%s saved to %s", filename.c_str(), path_.c_str());
}

void mapGeneration::getArmPose(geometry_msgs::Pose& arm_pose)
{
  std::string planning_frame = group_->getPlanningFrame();
  ROS_INFO("Planning frame is %s", planning_frame.c_str()); ////////
  
  moveit::core::RobotModelConstPtr robot_model = group_->getRobotModel();
  std::vector<std::string> full_link_names = robot_model->getLinkModelNames();
  std::vector<std::string> arm_link_names = group_->getLinkNames();
  int position = std::find(full_link_names.begin(), full_link_names.end(), arm_link_names[0]) -full_link_names.begin() ;
  std::string arm_parent_link = full_link_names[position-1];
  ROS_INFO("Arm parent link is %s", arm_parent_link.c_str()); ////

  if (arm_parent_link == planning_frame){
    geometry_msgs::Pose empty_pose;
    empty_pose.orientation.w = 1; // make it a valid "empty" pose
    arm_pose = empty_pose;
    do_centering_ = false;
  }else{
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    robot_state->setToDefaultValues();
    robot_state->update();
    Eigen::Affine3d tf_to_arm_parent = robot_state->getGlobalLinkTransform(arm_parent_link); 
    // I ASSUME THAT THE PLANNING FRAME COINCIDES WITH THE GLOBAL FIXED REFERENCE FRAME (= its transform would be (0,0,0)(0,0,0,1))
    tf::poseEigenToMsg(tf_to_arm_parent, arm_pose);
    ROS_INFO("Arm base pose : x:%f, y:%f, z:%f - quat:(%f,%f,%f,%f)", arm_pose.position.x, arm_pose.position.y, arm_pose.position.z,
           arm_pose.orientation.x, arm_pose.orientation.y, arm_pose.orientation.z, arm_pose.orientation.w);
  }

}
  
void mapGeneration::generate()
{
  ros::Time startit = ros::Time::now();
  getArmPose(arm_pose_);
  ROS_INFO("Center of workspace   x:%f, y:%f, z:%f - quat:(%f,%f,%f,%f)", arm_pose_.position.x, arm_pose_.position.y, arm_pose_.position.z,
           arm_pose_.orientation.x, arm_pose_.orientation.y, arm_pose_.orientation.z, arm_pose_.orientation.w);
  discretizeWorkspace(arm_pose_);
  double dif2 = ros::Duration( ros::Time::now() - startit).toSec();
  ROS_INFO("Time for discretizing workspace %.2lf seconds.", dif2);
  ROS_INFO("Initial workspace has %d spheres and %d poses", init_sp_size_, init_pose_size_);
  filterWorkspace();
  double dif3 = ros::Duration( ros::Time::now() - startit).toSec();
  ROS_INFO("Time for creating reachable workspace is %.2lf seconds.", dif3);
  if(do_centering_){
    centerWorkspace(); //added by me
    double dif4 = ros::Duration( ros::Time::now() - startit).toSec();
    ROS_INFO("Time for creating reachable workspace is %.2lf seconds.", dif4);
  }  
  saveWorkspace();
  ROS_INFO("Center of workspace   x:%f, y:%f, z:%f", arm_pose_.position.x, arm_pose_.position.y, arm_pose_.position.z);
  ROS_INFO("Time for discretizing workspace %.2lf seconds.", dif2);
  ROS_INFO("Time for creating reachable workspace is %.2lf seconds.", dif3);
  ROS_INFO("Final workspace has %d spheres and %d poses", final_sp_size_, final_pose_size_);
  ROS_INFO("Completed");
}

}
