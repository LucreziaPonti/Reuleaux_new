#include <boost/function.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/assert.hpp>
#include <math.h>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <base_placement_plugin/place_base.h>
//////#include <base_placement_plugin/filter_collision_poses.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <visualization_msgs/MarkerArray.h>
#include <ros/ros.h>

#include <map_creator/WorkSpace.h>
#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>


#include <visualization_msgs/InteractiveMarker.h>
#include <interactive_markers/interactive_marker_server.h>

#include <reule_aux/BP_Res.h>

PlaceBase::PlaceBase(QObject *parent)
{
  init();
}

PlaceBase::~PlaceBase()
{
}

void PlaceBase::init()
{
  //show_ureach_models_ = false;
}

void PlaceBase::BasePlacementHandler(std::vector< geometry_msgs::Pose > waypoints)
{
  /*! The function for base placement has been placed in a separate thread.
      This prevents the RViz and the Plugin to lock.
  */
  ROS_INFO("Starting concurrent process for Base Placement");
  QFuture< void > future = QtConcurrent::run(this, &PlaceBase::findbase, waypoints);
}

void PlaceBase::initRvizDone()
{
  /*!
      Once the initialization of the RViz is has finished, this function sends the pose of the initial marker. It also
     sends the method list and the visualization method lists.
  */
  ROS_INFO("RViz is done now we need to emit the signal");

  tf::Vector3 vec(0, 0, 0);
  tf::Quaternion quat(0, 0, 0, 1);
  quat.normalize();
  tf::Transform trns;
  trns.setOrigin(vec);
  trns.setRotation(quat);

  // Q_EMIT getRobotModelFrame_signal(trns);
  Q_EMIT getinitialmarkerFrame_signal(trns);

  ROS_INFO("Sending the method list");
  method_names_.push_back("PrincipalComponentAnalysis");
  method_names_.push_back("GraspReachabilityScore");
  method_names_.push_back("IKSolutionScore");
  method_names_.push_back("VerticalRobotModel");
  method_names_.push_back("UserIntuition");
  Q_EMIT sendBasePlaceMethods_signal(method_names_);

  ROS_INFO("Sending the output visualization method list");
  output_type_.push_back("Arrows");
  output_type_.push_back("Manipulator");
  output_type_.push_back("RobotModel");
  Q_EMIT sendOuputType_signal(output_type_);

}

void PlaceBase::getShowUreachModels(bool show_u_models)
{
  show_ureach_models_ = show_u_models;
  ROS_INFO("Show unreachable model changing");
}

void PlaceBase::getSelectedRobotGroup(int group_index)
{
  /* The selected group signal has been received
  */

  selected_group_ = group_names_[group_index];
  ROS_INFO_STREAM("Selected Group : "<<group_names_[group_index]);
  mark_ = new CreateMarker(selected_group_);
  // if(!mark_->checkEndEffector())
  // {
  //   ROS_ERROR_STREAM("Is your selected group is a manipulator?? ");
  //   delete mark_;
  // }
  ////questo nella pr e` scommentato
  Q_EMIT sendSelectedGroup_signal(selected_group_);
}

void PlaceBase::getSelectedMethod(int index)
{
  /* The selected method signal has been received
  */
  selected_method_ = index;
  ROS_INFO_STREAM("selected_method: " << method_names_[index]);
  if(index == 3 || index==4)
  {
    if(!checkforRobotModel())
    {
      ROS_ERROR_STREAM("The base placement method you selected needs robot model to be loaded.");
    }
  }
}

void PlaceBase::getSelectedOpType(int op_index)
{
  /* The selected output type signal has been received
  */
  selected_op_type_ = op_index;
  ROS_INFO_STREAM("selected visualization method: " << output_type_[op_index]);
  if(selected_op_type_==1 || selected_op_type_==2) // arm or robot_model
  {
    if(!checkforRobotModel())
    {
      ROS_ERROR_STREAM("The visualization method you selected needs robot model to be loaded.");
    }
  }
}


bool PlaceBase::checkforRobotModel()
{
  if(!robot_model_)
  {
    if(loadRobotModel())
    {
      group_names_.clear();
      group_names_ = robot_model_->getJointModelGroupNames();
      //ROS_INFO("Sending the robot groups list");
      Q_EMIT sendGroupType_signal(group_names_);
      ROS_INFO("Please select your manipulator group. ");
      return true;
    } 
   }
  else
    return true;
}

bool PlaceBase::loadRobotModel()
{
  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  if(robot_model_loader.getModel() ==NULL)
    return false;
  else{
    robot_model_ = robot_model_loader.getModel();
    return true;
  }
}

void PlaceBase::getBasePoses(std::vector<geometry_msgs::Pose> base_poses)
{
  final_base_poses_user=base_poses;
}

void PlaceBase::setBasePlaceParams(int base_loc_size, int high_score_sp)
{
  /*! Set the necessary parameters for the Base Placement process.
      These parameters correspond to the ones that the user has entered or the default ones before pressing the find
     base button.
      The base loc size is the number of ouput the user desires
      The highscoresp is the number of high scoring spheres we are collecting all the valid poses
  */
  ROS_INFO_STREAM("Base Placement parameters from UI:\n Number of base locations:"
                  << base_loc_size << "\n Number of HighScore Spheres:" << high_score_sp);

  BASE_LOC_SIZE_ = base_loc_size;
  HIGH_SCORE_SP_ = high_score_sp;
  if (BASE_LOC_SIZE_ <= 0)
  {
    ROS_ERROR_STREAM("Please provide a valid number of how many base locations do you need.");
  }

  if (HIGH_SCORE_SP_ <= 0)
  {
    ROS_ERROR_STREAM("Please provide a valid number of how many spheres do you need to create valid base poses");
  }
}

//Creating <spheres, ri> and <highscoringsp> multimap from the <sphere, poses> the bool determines wherther to make it 2d/3d
void PlaceBase::createSpheres(std::multimap< std::vector< double >, std::vector< double > > basePoses, /*baseTrnsCol*/
                   std::map< std::vector< double >, double >& spColor, std::vector< std::vector< double > >& highScoredSp, bool reduce_D)
{
  ros::NodeHandle nn;
  kinematics::Kinematics k;
  std::vector<int> poseCount;
  ROS_INFO("begin creating UNION MAP -- may take some time because of collision checking");
  poseCount.reserve(basePoses.size());
  for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = basePoses.begin(); it != basePoses.end();++it)
  {
    int num = basePoses.count(it->first);
    poseCount.push_back(num);
  }
  std::vector<int>::const_iterator it;
  it = max_element(poseCount.begin(), poseCount.end());
  int max_number = *it;
  it = min_element(poseCount.begin(), poseCount.end());
  int min_number = *it;
  if(reduce_D) // reduce_D=true -- "cuts" the sphere distribution of the IRM in the plane (from 3D to 2D) for VerticalRobotModel
  {
    for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = basePoses.begin(); it != basePoses.end();++it)
    {
      if((it->first)[2] < 0.06 && (it->first)[2] > -0.06) // poses on the ground 
      {
        geometry_msgs::Pose prob_base_pose;
        prob_base_pose.position.x = (it->first)[0];
        prob_base_pose.position.y = (it->first)[1];
        prob_base_pose.position.z = (it->first)[2];
        prob_base_pose.orientation.w = 1.0;
        
        ////////FILTERING POSES TO AVOID COLLISION WITH OBJECTS IN THE SCENE *****REMOVE****
        //////FilterCollisionPoses filter;
        ///////if(filter.check_collision_objects(nn, prob_base_pose.position.x, prob_base_pose.position.y, prob_base_pose.position.z, false)){ 
          ///////true = NOT IN COLLISION with any collision object of the planning scene -- acceptable base pos
          geometry_msgs::Pose base_pose_at_arm;
          transformFromRobotbaseToArmBase(prob_base_pose, base_pose_at_arm);
          int num_of_solns = 0;
          for(int j=0;j<GRASP_POSES_.size();j++)
          {
            std::vector<double> joint_soln;
            int nsolns = 0;
            k.isIkSuccesswithTransformedBase(base_pose_at_arm, GRASP_POSES_[j], joint_soln, nsolns);
            num_of_solns +=nsolns;
          }
          //recomputes the reachability score of the spheres
          float d = (float(num_of_solns)/float(GRASP_POSES_.size()*8))*100;
          spColor.insert(std::pair< std::vector< double >, double >(it->first, double(d)));
          //// stores it in the union map associating score and pose
        ///////}
      }
     }
  }
  else{ ////use the whole 3D distribution
  //Determining color of spheres of 3d map
    for(std::multimap<std::vector<double>, std::vector<double> >::iterator it = basePoses.begin(); it!=basePoses.end();++it)
    {
      geometry_msgs::Pose prob_base_pose;
      prob_base_pose.position.x = (it->first)[0];
      prob_base_pose.position.y = (it->first)[1];
      prob_base_pose.position.z = (it->first)[2];
      ////////FILTERING POSES TO AVOID COLLISION WITH OBJECTS IN THE SCENE AND SATISFY ROBOT CONSTRAINTS (for tiago=torso's height)
      ////////FilterCollisionPoses filter;
      /////////if(filter.check_collision_objects(nn, prob_base_pose.position.x, prob_base_pose.position.y, prob_base_pose.position.z, true)){ 
        //true = NOT IN COLLISION with any collision object of the planning scene -- acceptable base pos
        float d = ((float(basePoses.count(it->first))- min_number)/ (max_number - min_number)) * 100; //PLACE_BASE index
        if(d>1){
          spColor.insert(std::pair< std::vector< double >, double >(it->first, double(d)));
        }
      ///////}
    }
  }

 std::multiset<std::pair<double, std::vector<double> > > scoreWithSp; // ordered structure containing scored and sphere poses ORDERED BY SCORE (low to high)
 for(std::map<std::vector<double>, double>::iterator it = spColor.begin(); it !=spColor.end();++it )
 {
   scoreWithSp.insert(std::pair<double, std::vector<double> >(it->second, it->first));
 }
 for (std::multiset< std::pair< double, std::vector< double > > >::reverse_iterator it = scoreWithSp.rbegin();it != scoreWithSp.rend(); ++it)
 {
   highScoredSp.push_back(it->second); // ordered structure containing only the sphere poses ORDERED BY SCORE (high to low)
 }
  ROS_INFO("finished creating UNION MAP");
}


double PlaceBase::calculateScoreForRobotBase(std::vector<geometry_msgs::Pose> &grasp_poses, std::vector<geometry_msgs::Pose> &base_poses)
{
  kinematics::Kinematics k;
  float total_score = 0;
  float max_score = 0;
  
  ros::NodeHandle n;
  ros::Publisher bp_pub = n.advertise<reule_aux::BP_Res>("reule_aux/bp_results", 1000);
  reule_aux::BP_Res bp_res;

  ROS_DEBUG(" start calc score with %ld poses",base_poses.size());
  geometry_msgs::Pose best_pose;
  for(int i=0;i<base_poses.size();i++){
    geometry_msgs::Pose base_pose_at_arm;
    transformFromRobotbaseToArmBase(base_poses[i], base_pose_at_arm);
    ROS_DEBUG("trasform base pose %d to arm pose",i);
    int num_of_solns = 0;
    for(int j=0;j<grasp_poses.size();j++){
      std::vector<double> joint_soln;
      int nsolns = 0;
      k.isIkSuccesswithTransformedBase(base_pose_at_arm, grasp_poses[j], joint_soln, nsolns);
      num_of_solns +=nsolns;
    }
    float d = (float(num_of_solns)/float(grasp_poses.size()*8))*100; //// review questo calcolo
    ROS_DEBUG("score of pose %d : %f",i,d);
    if(d>max_score){
      max_score = d;
      best_pose = base_poses[i];
      ROS_DEBUG("base pose %d is the best for now",i);
    }
    total_score +=d;

    // publishing (publish all the poses as not the best - also the one that will be considered the best)
    bp_res.best_pose=false;
    bp_res.pose = base_poses[i];
    bp_res.score = d;
    bp_pub.publish(bp_res);
    // printing the results in the terminal for check 
    tf2::Quaternion quat(base_poses[i].orientation.x, base_poses[i].orientation.y, base_poses[i].orientation.z, base_poses[i].orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Optimal base pose[%d]: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f) - Score: %.2f", i + 1, base_poses[i].position.x, base_poses[i].position.y, base_poses[i].position.z, roll,pitch, yaw,d);
  }
  ROS_DEBUG("total score= %f",total_score);
  best_pose_ = best_pose;

  // publishing of the best pose (remember that this pose has also been published before - create the node that receives them so that it is accounted for)
  bp_res.best_pose=true;
  bp_res.pose = best_pose;
  bp_res.score = max_score;
  bp_pub.publish(bp_res);
  // printing the results in the terminal for check 
  tf2::Quaternion quat(best_pose_.orientation.x, best_pose_.orientation.y, best_pose_.orientation.z, best_pose_.orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("Best pose for this solution: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)", best_pose_.position.x, best_pose_.position.y, best_pose_.position.z, roll,pitch, yaw);
  
    double score = double(total_score/float(base_poses.size()));
  ROS_DEBUG("Average score= %f",score);
  return score;
}


double PlaceBase::calculateScoreForArmBase(std::vector<geometry_msgs::Pose> &grasp_poses, std::vector<geometry_msgs::Pose> &base_poses)
{
  kinematics::Kinematics k;
  float total_score = 0;
  float max_score = 0;

  ros::NodeHandle n;
  ros::Publisher bp_pub = n.advertise<reule_aux::BP_Res>("reule_aux/bp_results", 1000);
  reule_aux::BP_Res bp_res;

  geometry_msgs::Pose best_pose;
  for(int i=0;i<base_poses.size();i++)
  {
    int num_of_solns = 0;
    for(int j=0;j<grasp_poses.size();j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      k.isIkSuccesswithTransformedBase(base_poses[i], grasp_poses[j], joint_soln, nsolns);
      num_of_solns +=nsolns;
    }
    float d = (float(num_of_solns)/float(grasp_poses.size()*8))*100;
    
    // publishing (publish all the poses as not the best - also the one that will be considered the best)
    bp_res.best_pose=false;
    bp_res.pose = base_poses[i];
    bp_res.score = d;
    bp_pub.publish(bp_res);
    // printing the results in the terminal for check 
    tf2::Quaternion quat(base_poses[i].orientation.x, base_poses[i].orientation.y, base_poses[i].orientation.z, base_poses[i].orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Optimal base pose[%d]: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f) - Score: %.2f", i + 1, base_poses[i].position.x, base_poses[i].position.y, base_poses[i].position.z, roll,pitch, yaw,d);
    if(d>max_score){
      max_score = d;
      best_pose = base_poses[i];
    }
    total_score +=d;
  }
  best_pose_ = best_pose;

  // publishing of the best pose (remember that this pose has also been published before - create the node that receives them so that it is accounted for)
  bp_res.best_pose=true;
  bp_res.pose = best_pose;
  bp_res.score = max_score;
  bp_pub.publish(bp_res);
  // printing the results in the terminal for check 
  tf2::Quaternion quat(best_pose_.orientation.x, best_pose_.orientation.y, best_pose_.orientation.z, best_pose_.orientation.w);
  tf2::Matrix3x3 m(quat);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  ROS_INFO("Best pose for this solution: (%.2f, %.2f, %.2f) (%.2f, %.2f, %.2f)", best_pose_.position.x, best_pose_.position.y, best_pose_.position.z, roll,pitch, yaw);
  
  double score = double(total_score/float(base_poses.size()));
  ROS_INFO("Score average = %.2f",score);
  return score;
}

void PlaceBase::transformFromRobotbaseToArmBase(const geometry_msgs::Pose& base_pose, geometry_msgs::Pose &arm_base_pose)
{
  ////Get a joint group from this model (by name) - get the joints of the "arm" group
  const moveit::core::JointModelGroup* arm_jmp = robot_model_->getJointModelGroup(selected_group_);
  ////Get the names of the links that are part of this joint group. (arm)
  const std::vector<std::string>& arm_link_names = arm_jmp->getLinkModelNames();
  ////get the configuration the model is in, to then use to compute the transforms
  moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(robot_model_));
  std::vector<double> joint_soln_empty;
  mark_->updateRobotState(joint_soln_empty, robot_state_, false);
  ////Get the link names (of all links)
  std::vector<std::string> full_link_names = robot_model_->getLinkModelNames();
  ////find the index of the first link of the arm inside the entire list of link names (for tiago - arm_1_link)
  int position = std::find(full_link_names.begin(), full_link_names.end(), arm_link_names[0]) -full_link_names.begin() ;
  ////get the "Global transform" to the parent link of the arm (for tiago - torso_lift_link)
  //// !!!!!! need the robot root frame to be coinciding to the global fixed frame
  const Eigen::Affine3d trans_to_arm_parent = robot_state_->getGlobalLinkTransform(full_link_names[position-1]);
  Eigen::Affine3d base_pose_tf; //Affine3d is a Pose type message(contains a Vector 3d and Quaterniond/RotationMatrix). 
  ////change the pose info of the base pose computed into a tf
  tf::poseMsgToEigen(base_pose, base_pose_tf);
  ////finds the "new"position of the first link of the arm if the global frame=base frame corresponds to the base pose
  tf::poseEigenToMsg( base_pose_tf * trans_to_arm_parent  , arm_base_pose);
  //return the pose of the first link of the arm (for the first three solving methods)
}

//function transforms the inverse reachability data to the base of the whole robot model
void PlaceBase::transformToRobotbase(std::multimap< std::vector< double >, std::vector< double > > armBasePoses,
                          std::multimap< std::vector< double >, std::vector< double > >& robotBasePoses)
{
  ////get the joints of the "arm" group
  const moveit::core::JointModelGroup* robot_jmp = robot_model_->getJointModelGroup(selected_group_);
  ////get the names of the links of the arm
  const std::vector<std::string>& arm_link_names = robot_jmp->getLinkModelNames();
  ////robot state/configuration(?)
  moveit::core::RobotStatePtr robot_state_(new moveit::core::RobotState(robot_model_));
  std::vector<double> joint_soln_empty;
  mark_->updateRobotState(joint_soln_empty, robot_state_, false);
  ////robot_state_->printStateInfo(); 
  ////transform from global frame to "root link" of the entire robot (for tiago - base_footprint)
  const Eigen::Affine3d trans_to_root = robot_state_->getGlobalLinkTransform(robot_model_->getRootLinkName()); ////not used ever  //world T root
  ////tranform from global frame to first link of arm (for tiago - arm_1_link)
  const Eigen::Affine3d trans_to_arm = robot_state_->getGlobalLinkTransform(arm_link_names[0]); // world T arm

  geometry_msgs::Pose arm_pose;
  tf::poseEigenToMsg(trans_to_arm, arm_pose);
  ROS_INFO("TRANSFORMTOROBOTBASE - global transform arm_1_link: %f %f %f %f %f %f %f",arm_pose.position.x,arm_pose.position.y,arm_pose.position.z,arm_pose.orientation.x,arm_pose.orientation.y,arm_pose.orientation.z,arm_pose.orientation.w);
  geometry_msgs::Pose root_pose;
  tf::poseEigenToMsg(trans_to_root, root_pose);
  ROS_INFO("TRANSFORMTOROBOTBASE - global transform root: %f %f %f %f %f %f %f",root_pose.position.x,root_pose.position.y,root_pose.position.z,root_pose.orientation.x,root_pose.orientation.y,root_pose.orientation.z,root_pose.orientation.w);

  ////inverse - !!!! we need that root and global frames correspond
  const Eigen::Affine3d arm_to_root = trans_to_arm.inverse()*trans_to_root; // arm T root = arm T world * world T root
  ////maybe here we should use trans_to_root if we don't want this correspondence 
  const Eigen::Vector3d v_arm_inv = arm_to_root.translation();
  ROS_INFO("TRANSFORMTOROBOTBASE - inverse transform arm_1_link: %f %f %f",v_arm_inv[0],v_arm_inv[1],v_arm_inv[2]);

  sphere_discretization::SphereDiscretization sd;
  ////for all the possible armbaseposes computed  (globally??)
  for (std::multimap< std::vector< double >, std::vector< double > >::iterator it = armBasePoses.begin(); it != armBasePoses.end();++it)
  {
    geometry_msgs::Pose arm_base_pose;
    ////save the pose of this iteration into arm_base_pose
    sd.convertVectorToPose(it->second, arm_base_pose);
    Eigen::Affine3d arm_base_tf;
    ////convert into an Affine3d tf
    tf::poseMsgToEigen(arm_base_pose, arm_base_tf);
    geometry_msgs::Pose robot_base_pose;
    ////compute position of the robot base given the position of the first link of the arm
    tf::poseEigenToMsg(arm_to_root*arm_base_tf , robot_base_pose);
    ////convert to have the correct data format of the global tranform to computed robot base
    static const int arr[] = {1,1,1};
    std::vector<double> base_vec (arr, arr + sizeof(arr) / sizeof(arr[0]) );
    std::vector<double> base_pose;
    sd.convertPoseToVector(robot_base_pose, base_pose);
    robotBasePoses.insert(std::pair< std::vector< double >, std::vector< double > >(base_vec,base_pose));

  }
}



bool PlaceBase::findbase(std::vector< geometry_msgs::Pose > grasp_poses)
{
  /*! The main function that finds the base. 
   * If the method is UserIntuition it does not need to create the union map, but only evaluates the poses given by the user to find the best
   * First it transforms all the poses from the loaded inverse reachability file with the grasp poses selected by the user -> PoseColFilter
   * Then (the function associatePose) by nearest neighbor searching it associates all the poses to the corresponding spheres -> baseTrnsCol
   * In the function createSpheres:
   *      By normalization it decides the color of the spheres. -> sphereColor contains elements <sphere, score (=color)> (used only for VISUALIZATION from here on)
   *      Then from the highest scoring spheres it calls the desired methods for finding the final base locations. -> highScoreSp contains sphere poses ordered by score
    SO baseTrnsCol contains all the poses (voxel_pose,base_pose), highScoreSp contains all the Spheres ordered by score
  */

  Q_EMIT basePlacementProcessStarted();
  score_ = 0;
    
  if (grasp_poses.size() == 0)
    ROS_ERROR_STREAM("Please provide atleast one grasp pose.");

  else if(selected_method_ == 4) // UserIntuition - no need to create the union map
  {
    GRASP_POSES_ = grasp_poses;
    ROS_INFO("Start of computation of the base placement");
    BasePlaceMethodHandler(); 
    // Here depending on the method is done the evaluation of the poses and the score is calculated
    // once the poses are found and the score is calculated the results are also published on the topic reule_aux/bp_results
    /* OLD FOR CYCLE THAT PRINTED THE RESULTS - NOT NEEDED ANYMORE - DONE DIRECTLY DURING WHEN THE SCORE IS CALCULATED
      for (int i = 0; i < final_base_poses.size(); ++i)
      {
        //Transforming the poses, as all are pointing towards

        tf2::Quaternion quat(final_base_poses[i].orientation.x, final_base_poses[i].orientation.y,
                            final_base_poses[i].orientation.z, final_base_poses[i].orientation.w);
        tf2::Matrix3x3 m(quat);

        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        ROS_INFO("Optimal base pose[%d]: Position: %f, %f, %f, Orientation: %f, %f, %f", i + 1, final_base_poses[i].position.x, final_base_poses[i].position.y, final_base_poses[i].position.z, roll, pitch, yaw);
      }
    */

    ROS_INFO("Computation done. Start visualization");
    OuputputVizHandler(final_base_poses);


  }else  {

    if (PoseColFilter.size() == 0)    {
      ROS_WARN("No Inverse Reachability Map found. Please provide an Inverse Reachability map.");
    }    else    {
      sphere_discretization::SphereDiscretization sd;
      baseTrnsCol.clear();
      sphereColor.clear();
      highScoreSp.clear();
      robot_PoseColfilter.clear();
      GRASP_POSES_.clear();

      final_base_poses.clear();
      GRASP_POSES_ = grasp_poses;

      if(selected_method_ == 3) {// findBaseByVerticalRobotModel - extends the computation from the arm base to the base of the entire robot
        transformToRobotbase(PoseColFilter, robot_PoseColfilter); // transforms IRM so that it contains robot base poses instead of arm base
        sd.associatePose(baseTrnsCol, grasp_poses, robot_PoseColfilter, res); //create a point cloud which consists of all of the possible base locations for all grasp poses and a list of base pose orientations
        ROS_INFO("Size of baseTrnsCol dataset: %lu", baseTrnsCol.size());
        createSpheres(baseTrnsCol, sphereColor, highScoreSp, true);
      }else{ // remaining methods: findBaseByPCA, findBaseByGraspReachabilityScore, findBaseByIKSolutionScore
        ROS_INFO("//////////////////////// START UNION MAP CREATION ////////////////////////");
        /*////////////*/std::chrono::high_resolution_clock::time_point start_AP = std::chrono::high_resolution_clock::now(); 
        sd.associatePose(baseTrnsCol, grasp_poses, PoseColFilter, res);
        /*////////////*/std::chrono::high_resolution_clock::time_point finish_AP = std::chrono::high_resolution_clock::now(); 
        /*////////////*/std::chrono::milliseconds AP = std::chrono::duration_cast<std::chrono::milliseconds>(finish_AP - start_AP); 
        /*////////////*/ROS_INFO("Time for AssociatePose: %ld ms", AP.count());  
        ROS_INFO("Size of baseTrnsCol dataset: %lu", baseTrnsCol.size());
        /*////////////*/std::chrono::high_resolution_clock::time_point start_CS = std::chrono::high_resolution_clock::now(); 
        createSpheres(baseTrnsCol, sphereColor, highScoreSp, false);
        /*////////////*/std::chrono::high_resolution_clock::time_point finish_CS = std::chrono::high_resolution_clock::now(); 
        /*////////////*/std::chrono::milliseconds CS = std::chrono::duration_cast<std::chrono::milliseconds>(finish_CS - start_CS); 
        /*////////////*/ROS_INFO("Time for CreateSpheres: %ld ms", CS.count()); 
      }

      ROS_INFO("Union map has been created. Can now visualize Union Map.");
      ROS_INFO("Poses in Union Map: %lu", baseTrnsCol.size());
      ROS_INFO("Spheres in Union Map: %lu", sphereColor.size());
      ROS_INFO("//////////////////////// END UNION MAP CREATION ////////////////////////");


      ROS_INFO("Start of computation of the base placement");
      BasePlaceMethodHandler(); // depending on the method the poses are found (and stored in final_base_poses) and they are published on reule_aux/bp_results
      
      /* OLD FOR CYCLE THAT PRINTED THE RESULTS - NOT NEEDED ANYMORE - DONE DIRECTLY DURING WHEN THE SCORE IS CALCULATED
        for (int i = 0; i < final_base_poses.size(); ++i)
        {
          //Transforming the poses, as all are pointing towards

          tf2::Quaternion quat(final_base_poses[i].orientation.x, final_base_poses[i].orientation.y,
                              final_base_poses[i].orientation.z, final_base_poses[i].orientation.w);
          tf2::Matrix3x3 m(quat);

          double roll, pitch, yaw;
          m.getRPY(roll, pitch, yaw);
          ROS_INFO("Optimal base pose[%d]: Position: %f, %f, %f, Orientation: %f, %f, %f", i + 1, final_base_poses[i].position.x, final_base_poses[i].position.y, final_base_poses[i].position.z, roll, pitch, yaw);
        }
      */
      
      ROS_INFO("Computation done. Start visualization");
      OuputputVizHandler(final_base_poses);  // function to have different showBaseLocations methods
    }
  }
  /* OLD PRINT OF THE BEST POSE - NOT NEEDED ANYMORE - DONE DIRECTLY DURING WHEN THE SCORE IS CALCULATED
    tf2::Quaternion quat(best_pose_.orientation.x, best_pose_.orientation.y, best_pose_.orientation.z, best_pose_.orientation.w);
    tf2::Matrix3x3 m(quat);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("Best pose for this solution: Position: %f, %f, %f, Orientation: %f, %f, %f",
                  best_pose_.position.x, best_pose_.position.y, best_pose_.position.z, roll,pitch, yaw);
 */

   
  Q_EMIT basePlacementProcessCompleted(score_);

  Q_EMIT basePlacementProcessFinished();
  ROS_INFO("FindBase Task Finished");
}

void PlaceBase::BasePlaceMethodHandler()
{
  /* Switch cases for selecting method for base placement
  */
  switch (selected_method_)
  {
    case 0:
    {
      findBaseByPCA();
      break;
    }

    case 1:
    {
      findBaseByGraspReachabilityScore();
      break;
    }

    case 2:
    {
      findBaseByIKSolutionScore();
      break;
    }
    case 3:
    {
      findBaseByVerticalRobotModel();
      break;
    }
   case 4:
   {
      findBaseByUserIntuition();
      break;
   }
   
  }
}

void PlaceBase::OuputputVizHandler(std::vector< geometry_msgs::Pose > po)
{
  /* Switch cases for selecting output type for visualization
  */
  switch (selected_op_type_)
  {
    case 0:
    {
      showBaseLocationsbyArrow(po);
      break;
    }

    case 1:
    {
      showBaseLocationsbyArmModel(po);
      break;
    }

    case 2:
    {
      showBaseLocationsbyRobotModel(po);
      break;
    }
  }
}

void PlaceBase::findBaseByUserIntuition()
{
  ROS_INFO("Finding optimal base pose by user intuition.");
  double s = calculateScoreForRobotBase(GRASP_POSES_, final_base_poses_user);
  score_ = s;
  final_base_poses = final_base_poses_user;
}

void PlaceBase::findBaseByVerticalRobotModel()
{

  ROS_INFO("Finding optimal ROBOT base pose by Vertical robot model.");
  std::vector<geometry_msgs::Pose> base_poses;
  std::vector<geometry_msgs::Pose> base_poses_user;
  int num_of_desired_sp = HIGH_SCORE_SP_+50;
  int num_of_sp = highScoreSp.size();
  if(BASE_LOC_SIZE_>num_of_sp)
    ROS_ERROR("Desired base locations are too high (num_of_sp=%d , num_base_loc=%d). Please reduce it",num_of_sp,BASE_LOC_SIZE_);
  //The iteration is taken 4 just to make sure that the robot models are not so close to each other
  for(int i=0;i<num_of_desired_sp;i+=4)
  {
    geometry_msgs::Pose prob_base_pose;
    prob_base_pose.position.x = highScoreSp[i][0];
    prob_base_pose.position.y = highScoreSp[i][1];
    prob_base_pose.position.z = highScoreSp[i][2];
    prob_base_pose.orientation.x = 0;
    prob_base_pose.orientation.y = 0;
    prob_base_pose.orientation.z = 0;
    prob_base_pose.orientation.w = 1;
    base_poses.push_back(prob_base_pose);
     
  }

  for(int i=0;i<BASE_LOC_SIZE_;++i)
  { // stores only the ones with highest score, as many as requested by user
    base_poses_user.push_back(base_poses[i]);
  }
ROS_DEBUG("-- finito di fare le base poses in vertical rob mod - chiamiamo calculate score");
  double s = calculateScoreForRobotBase(GRASP_POSES_, base_poses_user);
  ROS_DEBUG("- vertRobMod finito score: %f ",s);
  score_ = s;
  final_base_poses = base_poses_user;
}

void PlaceBase::findBaseByPCA()
{
  /* PCA Method: The planner takes desired number of high scoring spheres and implements PCA for finding optimal
   * orientations from all the poses correspond to that sphere. One pose from one sphere.
  */
  ROS_INFO("Finding optimal base pose by PCA.");
  sphere_discretization::SphereDiscretization sd;
  std::vector<geometry_msgs::Pose> pose_scores;
  map_creator::WorkSpace ws;
  for (int i = 0; i < BASE_LOC_SIZE_; ++i)
  {
    map_creator::WsSphere wss;
    wss.point.x = highScoreSp[i][0];
    wss.point.y = highScoreSp[i][1];
    wss.point.z = highScoreSp[i][2];
    
    std::vector< double > basePose;
    basePose.push_back(highScoreSp[i][0]);
    basePose.push_back(highScoreSp[i][1]);
    basePose.push_back(highScoreSp[i][2]);
    std::multimap< std::vector< double >, std::vector< double > >::iterator it;
    for (it = baseTrnsCol.lower_bound(basePose); it != baseTrnsCol.upper_bound(basePose); ++it)
    {
      geometry_msgs::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      wss.poses.push_back(pp);
    }
    ws.WsSpheres.push_back(wss);
  
  }

  for (int i = 0; i < ws.WsSpheres.size(); ++i)
  {
    geometry_msgs::Pose final_base_pose;
    
    sd.findOptimalPosebyPCA(ws.WsSpheres[i].poses, final_base_pose);  // Calling the PCA..After the normalization the results seem suitable
    final_base_pose.position.x = ws.WsSpheres[i].point.x;
    final_base_pose.position.y = ws.WsSpheres[i].point.y;
    final_base_pose.position.z = ws.WsSpheres[i].point.z;

    Eigen::Affine3d final_base_tf;
    tf::poseMsgToEigen(final_base_pose, final_base_tf);
    Eigen::Affine3d rx = Eigen::Affine3d(Eigen::AngleAxisd(-M_PI/2, Eigen::Vector3d(1,0,0)));
    geometry_msgs::Pose new_base_pose;
    tf::poseEigenToMsg( final_base_tf * rx  , new_base_pose);

    pose_scores.push_back(new_base_pose);

  }

  double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
  score_ = s;
  final_base_poses = pose_scores;
}

void PlaceBase::findBaseByGraspReachabilityScore()
{
  /* GraspReachabilityScore Method: The planner takes desired number of high scoring spheres and collects all the poses
     from them. Then calculates reachability of that poses with all the grasp points. The poses that can reach all the
     grasp poses can be considered as optimal base locations.
     //by this method->all the high scoring poses are in same sphere
     alternate method:
     //Take single pose from every high scoring sphere which has the highest hit
     As they are transformations of the grasp poses it is natural that they will point downwards.

  */
  ROS_INFO("Finding optimal base pose by GraspReachabilityScore.");
  sphere_discretization::SphereDiscretization sd;
  kinematics::Kinematics kin;
  std::vector<geometry_msgs::Pose> pose_scores;
  int numofSp = BASE_LOC_SIZE_;

  for(int i=0;i<numofSp;i++)
  {
    std::vector< geometry_msgs::Pose > probBasePoses;
    std::multimap<std::vector<double>, std::vector<double> >::iterator it;
    for(it = baseTrnsCol.lower_bound(highScoreSp[i]); it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
             
    }
    
    std::map<int, geometry_msgs::Pose> basePoseWithHits;
    for(int j=0;j<probBasePoses.size();j++)
    {
      int numofHits = 0;
      for(int j1=0;j1<GRASP_POSES_.size();j1++)
      {
        int nsolns = 0;
        std::vector<double> joint_solns;
        numofHits += kin.isIkSuccesswithTransformedBase(probBasePoses[j], GRASP_POSES_[j1],joint_solns, nsolns);
       }
      basePoseWithHits.insert(std::make_pair(numofHits, probBasePoses[j]));
    }
    std::map<int, geometry_msgs::Pose>::iterator itr;
    itr = basePoseWithHits.end();
    --itr;
    pose_scores.push_back(itr->second);
    double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
    score_ = s;
    final_base_poses = pose_scores;
  }
}

void PlaceBase::findBaseByIKSolutionScore()
{
  /* IKSolutionScore Method: The planner takes desired number of high scoring spheres and collects all the poses from
     them. Then calculates number of Ik solutions of that poses with all the grasp points. The poses that have the
     highest score can be considered as optimal base locations.
     //by this method->all the high scoring poses are in same sphere
     alternate method:
     //Take single pose from every high scoring sphere which has the highest hit
     As they are transformations of the grasp poses, it is natural that they will point downwards.

  */
  ROS_INFO("Finding optimal base pose by GraspReachabilityScore.");
  sphere_discretization::SphereDiscretization sd;
  kinematics::Kinematics kin;
  std::vector<geometry_msgs::Pose> pose_scores;
  int numofSp = BASE_LOC_SIZE_;

  int max_solns = GRASP_POSES_.size() * 8;
  int min_solns = 0;

  for(int i=0;i<numofSp;i++)
  {
    std::vector< geometry_msgs::Pose > probBasePoses;
    std::multimap<std::vector<double>, std::vector<double> >::iterator it;
    for(it = baseTrnsCol.lower_bound(highScoreSp[i]); it != baseTrnsCol.upper_bound(highScoreSp[i]); ++it)
    {
      geometry_msgs::Pose pp;
      sd.convertVectorToPose(it->second, pp);
      probBasePoses.push_back(pp);
       
    }
    
    std::map<double, geometry_msgs::Pose> basePoseWithHits;
    for(int j=0;j<probBasePoses.size();j++)
    {
      int numofHits = 0;
      int solns = 0;
      for(int j1=0;j1<GRASP_POSES_.size();j1++)
      {
        int nsolns = 0;
        std::vector<double> joint_solns;
        numofHits += kin.isIkSuccesswithTransformedBase(probBasePoses[j], GRASP_POSES_[j1],joint_solns, nsolns);
        solns +=nsolns;
       }
      double basePlaceScore = (double(solns) - double(min_solns)) / (double(max_solns) - double(min_solns));
      basePoseWithHits.insert(std::make_pair(basePlaceScore, probBasePoses[j]));
    }
    std::map<double, geometry_msgs::Pose>::iterator itr;
    itr = basePoseWithHits.end();
    --itr;
    pose_scores.push_back(itr->second);
    double s = calculateScoreForArmBase(GRASP_POSES_, pose_scores);
    score_ = s;
    final_base_poses = pose_scores;
  }
}

void PlaceBase::showBaseLocationsbyArrow(std::vector< geometry_msgs::Pose > po)
{
  /* Visualizing base solutions as arrow. Arrows are now pointing towards Z direction.
  */
  ROS_INFO("Showing Base Locations by Arrow: Arrows are pointing in Z direction");
  std::vector<geometry_msgs::Pose> pose_arr;
  for(int i=0;i<po.size();i++)
  {
    tf2::Transform trns;
    tf2::Quaternion quat(po[i].orientation.x, po[i].orientation.y, po[i].orientation.z, po[i].orientation.w);
    tf2::Vector3 vec(po[i].position.x, po[i].position.y, po[i].position.z);
    trns.setOrigin (vec);
    trns.setRotation (quat);

    tf2::Quaternion quat2;
    quat2.setRPY( 0, -M_PI/2,  0);
    tf2::Vector3 vec2(0, 0 , 0);
    tf2::Transform multiplier;
      //multiplier.setIdentity ();
    multiplier.setOrigin (vec2);
    multiplier.setRotation (quat2);

    trns = trns * multiplier;
    tf2::Vector3 new_pose_vec;
    tf2::Quaternion new_pose_quat;
    new_pose_vec = trns.getOrigin ();
    new_pose_quat = trns.getRotation ();
    new_pose_quat.normalize ();

    geometry_msgs::Pose new_pose;
    new_pose.position.x = new_pose_vec[0];
    new_pose.position.y = new_pose_vec[1];
    new_pose.position.z = new_pose_vec[2];
    new_pose.orientation.x = new_pose_quat[0];
    new_pose.orientation.y = new_pose_quat[1];
    new_pose.orientation.z = new_pose_quat[2];
    new_pose.orientation.w = new_pose_quat[3];
    pose_arr.push_back(new_pose);
  }

  ros::NodeHandle nh;
  ros::Publisher marker_pub = nh.advertise< visualization_msgs::MarkerArray >("visualization_marker_array", 1);
  visualization_msgs::MarkerArray markerArr;
  for (int i = 0; i < po.size(); ++i)
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";
    marker.header.stamp = ros::Time::now();
    marker.ns = "points";
    marker.action = visualization_msgs::Marker::ADD;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.lifetime = ros::Duration();
    marker.scale.x = 0.3;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;

    marker.color.r = 0.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0;
    marker.id = i;
  
    marker.pose = pose_arr[i];
    markerArr.markers.push_back(marker);
  }
  marker_pub.publish(markerArr);

}


void PlaceBase::showBaseLocationsbyArmModel(std::vector< geometry_msgs::Pose > po)
{

  ROS_INFO("Showing Base Locations by Arm Model:");
  kinematics::Kinematics k;

  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;
  imServer.reset(new interactive_markers::InteractiveMarkerServer("robot_model", "robot_model", false));
  ros::Duration(0.1).sleep();
  imServer->applyChanges();

  std::vector<visualization_msgs::InteractiveMarker> iMarkers;
  BasePoseJoint base_pose_joints;
  for(int i=0;i<po.size();i++)
  {
    for(int j=0;j<GRASP_POSES_.size();j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      k.isIkSuccesswithTransformedBase(po[i], GRASP_POSES_[j], joint_soln, nsolns);
      std::pair<std::vector<double>, geometry_msgs::Pose > myPair( joint_soln, po[i]);
      base_pose_joints.insert(myPair);
    }
  }
  mark_->makeArmMarker(base_pose_joints, iMarkers, show_ureach_models_);
  for(int i=0;i<iMarkers.size();i++)
    {
         imServer->insert(iMarkers[i]);
    }

  imServer->applyChanges();
  ros::Duration(25).sleep();
}


void PlaceBase::showBaseLocationsbyRobotModel(std::vector<geometry_msgs::Pose> po)
{
  ROS_INFO("Showing Base Locations by Robot Model:");
  //sphere_discretization::SphereDiscretization sd;
  kinematics::Kinematics k;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> imServer;
  imServer.reset(new interactive_markers::InteractiveMarkerServer("robot_model", "robot_model", false));
  ros::Duration(0.1).sleep();
  imServer->applyChanges();
  std::vector<visualization_msgs::InteractiveMarker> iMarkers;
  BasePoseJoint base_pose_joints;
  for(int i=0;i<po.size();i++)
  {
    geometry_msgs::Pose base_pose_at_arm;
    transformFromRobotbaseToArmBase(po[i], base_pose_at_arm);
    for(int j=0;j<GRASP_POSES_.size();j++)
    {
      std::vector<double> joint_soln;
      int nsolns = 0;
      k.isIkSuccesswithTransformedBase(base_pose_at_arm, GRASP_POSES_[j], joint_soln, nsolns);
      std::pair<std::vector<double>, geometry_msgs::Pose > myPair( joint_soln, po[i]);
      base_pose_joints.insert(myPair);
      }
  }

  mark_->makeRobotMarker(base_pose_joints, iMarkers, show_ureach_models_);
  for(int i=0;i<iMarkers.size();i++)
    {
         imServer->insert(iMarkers[i]);
    }

  imServer->applyChanges();
  ros::Duration(25).sleep();
}



void PlaceBase::setReachabilityData(std::multimap< std::vector< double >, std::vector< double > > PoseCollection,
                                    std::multimap< std::vector< double >, double > SphereCollection, float resolution)
{
  /* Setting the reachabilty data
  */
  PoseColFilter = PoseCollection;
  SphereCol = SphereCollection;
  res = resolution;
  ROS_INFO("Size of poses dataset: %lu", PoseColFilter.size());
  ROS_INFO("Size of Sphere dataset: %lu", SphereCol.size());
  ROS_INFO("Resolution: %f", res);
}



void PlaceBase::ShowUnionMap(bool show_map)
{
  /* Slot for showing Union map. Can only be visualized after calling the find base function. Otherwise it will show
   * error message
  */
  ROS_INFO("Showing Union Map:");
  if (sphereColor.size() == 0)
    ROS_INFO("The union map has not been created yet. Please create the Union map by the Find Base button");
  else
  {
    ros::NodeHandle n;
    ros::Publisher workspace_pub = n.advertise< map_creator::WorkSpace >("reachability_map", 1);
    map_creator::WorkSpace ws;
    ws.header.stamp = ros::Time::now();
    ws.header.frame_id = "world";
    ws.resolution = res;
    ROS_DEBUG(" loading spheres");
    for (std::multimap< std::vector< double >, double >::iterator it = sphereColor.begin(); it != sphereColor.end(); ++it)
    {
      map_creator::WsSphere wss;
      wss.point.x = it->first[0];
      wss.point.y = it->first[1];
      wss.point.z = it->first[2];
      wss.ri = it->second;
      ws.WsSpheres.push_back(wss);
    }
    ROS_DEBUG(" publishing spheres");
    workspace_pub.publish(ws);
  }
}

void PlaceBase::clearUnionMap(bool clear_map)
{
  /* The function need to be implemented. The slot is already created in widget
  */
  ROS_INFO("Clearing Union Map:");
}
