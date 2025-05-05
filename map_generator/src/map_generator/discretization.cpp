#include <map_generator/discretization.h>


namespace discretization
{
Discretization::Discretization()
{
  center_ = octomap::point3d(0,0,0);
  resolution_ = 0.08;
  radius_ = 1.0;
  max_depth_ = 16;
  centers_.resize(0);
  poses_.resize(0);
}

Discretization::Discretization(geometry_msgs::Pose pose, double resolution, double radius ):
  resolution_(resolution), radius_(radius)
{
  center_=octomap::point3d(pose.position.x, pose.position.y, pose.position.z);
  max_depth_ = 16;
  centers_.resize(0);
  poses_.resize(0);
 }

octomap::OcTree* Discretization::generateBoxTree(const octomap::point3d &origin, float diameter, float resolution)
{
  octomap::OcTree* tree = new octomap::OcTree(resolution/2);
  float min_z = origin.z() - diameter * 1.5;
  /*if (min_z<-resolution){ ////////////////////////////
    min_z=-resolution; // to exclude points underground from the start - with some "allowance"
  }*/
  for(float x = origin.x() - diameter * 1.5; x<=origin.x() + diameter * 1.5; x+=resolution)
  {
    for(float y = origin.y() - diameter * 1.5; y<=origin.y() + diameter * 1.5; y+=resolution)
    {
      //for (float z = min_z; z <= origin.z() + diameter * 1.5; z += resolution) //////////////////////
      for(float z = origin.y() - diameter * 1.5; z<=origin.z() + diameter * 1.5; z+=resolution)
      {
        octomap::point3d point;
        point.x() = x;
        point.y() = y;
        point.z() = z;
        tree->updateNode(point, true);
      }
    }
  }
  return tree;
}

void Discretization::createCenters(octomap::OcTree *tree, std::vector<geometry_msgs::Point> &centers)
{
  int sphere_count = 0;
  for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth_),end = tree->end_leafs(); it!=end;++it)
    sphere_count++;
  centers.reserve(sphere_count);
  for(octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth_),end = tree->end_leafs(); it!=end;++it)
  {
    geometry_msgs::Point point;
    point.x = (it.getCoordinate()).x();
    point.y = (it.getCoordinate()).y();
    point.z = (it.getCoordinate()).z();
    centers.push_back(point);
  }
}

int Discretization::getNumOfSpheres()
{
  return centers_.size();
}

void Discretization::createPosesOnSphere(const octomap::point3d& origin, const double r, std::vector<geometry_msgs::Pose> &poses)
{
  const double DELTA = M_PI/5.;
  const unsigned MAX_INDEX  (2 * 5 *5);
  static std::vector<geometry_msgs::Point> position_vector(MAX_INDEX);
  static std::vector<tf::Quaternion> quaternion(MAX_INDEX);
  static bool initialized = false;
  if(!initialized)
  {
    initialized = true;
    unsigned index = 0;
    for(double phi = 0; phi<2*M_PI; phi+=DELTA)//Azimuth[0,2PI]
    {
      for(double theta = 0;theta<M_PI;theta +=DELTA) //Elevation[0,2PI]
      {
        position_vector[index].x = cos(phi)*sin(theta);
        position_vector[index].y = sin(phi)*sin(theta);
        position_vector[index].z = cos(theta);
        tf::Quaternion quat;
        quat.setRPY(0, ((M_PI/2)+theta), phi);
        quat.normalize();
        quaternion[index] = quat;
        index++;
      }
    }
  }
  poses.reserve(MAX_INDEX);
  poses.clear();
  geometry_msgs::Pose pose;
  for(int i=0;i<MAX_INDEX;++i)
  {
    pose.position.x = r * position_vector[i].x + origin.x();
    pose.position.y = r * position_vector[i].y + origin.y();
    pose.position.z = r * position_vector[i].z + origin.z();
    pose.orientation.x = quaternion[i].x();
    pose.orientation.y = quaternion[i].y();
    pose.orientation.z = quaternion[i].z();
    pose.orientation.w = quaternion[i].w();
    poses.push_back(pose);
  }
}

void Discretization::createPoses(const std::vector<geometry_msgs::Point> &centers, std::vector<geometry_msgs::Pose> &poses)
{
  poses.reserve(centers.size()*50);
  for(int i=0;i<centers.size();++i)
  {
    map_generator::WsSphere wsSphere;
    wsSphere.point = centers[i];
    octomap::point3d origin = octomap::point3d(centers[i].x, centers[i].y, centers[i].z); 
    static std::vector<geometry_msgs::Pose>  pose;
    createPosesOnSphere(origin, resolution_, pose);
    for(int j=0;j<pose.size();++j)
    {
      poses.push_back(pose[j]);
      wsSphere.poses.push_back(pose[j]);
     }
    ws_.WsSpheres.push_back(wsSphere);
  }
  ws_.resolution=resolution_;
}

int Discretization::getNumOfPoses()
{
  return poses_.size();
}

void Discretization::getInitialWorkspace(map_generator::WorkSpace& ws)
{
  ws = ws_;
}

void Discretization::discretize()
{
  octomap::OcTree* tree = generateBoxTree(center_, radius_, resolution_);
  createCenters(tree, centers_);
  createPoses(centers_, poses_);
}

void Discretization::getCenters(std::vector<geometry_msgs::Point> &points)
{
  points = centers_;
}

void Discretization::poseToEigenVector(const geometry_msgs::Pose& pose, Eigen::VectorXd& vec)
{
  vec << pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z,
      pose.orientation.w;
}

void Discretization::findOptimalPosebyPCA(const std::vector< geometry_msgs::Pose >& probBasePoses,
  geometry_msgs::Pose& final_base_pose)
{
  Eigen::Matrix4d M;
  Eigen::MatrixXd poseData(7, probBasePoses.size());
  for (int i = 0; i < probBasePoses.size(); ++i)
  {
    Eigen::VectorXd vec(7);
    Eigen::Vector4d q(probBasePoses[i].orientation.x, probBasePoses[i].orientation.y, probBasePoses[i].orientation.z, probBasePoses[i].orientation.w);
    poseToEigenVector(probBasePoses[i], vec);
    poseData.col(i) << vec;
    if (i == 0){
      Eigen::Matrix4d D((q * q.transpose()));
      M = D/probBasePoses.size();
    }else{
      Eigen::Matrix4d D((q * q.transpose()));
      M += D/probBasePoses.size();
    }
  }

  Eigen::EigenSolver< Eigen::MatrixXd > eig(M);
  Eigen::VectorXd::Index idx;
  Eigen::VectorXd test = eig.eigenvalues().real();

  int i = test.maxCoeff(&idx);
  Eigen::Vector4d vector = eig.eigenvectors().col(idx).real();
  tf2::Quaternion final_base_quat(vector[0], vector[1], vector[2], vector[3]);
  final_base_quat.normalize();

  final_base_pose.position.x =0;
  final_base_pose.position.y =0;
  final_base_pose.position.z =0;

  final_base_pose.orientation.x = final_base_quat[0];
  final_base_pose.orientation.y = final_base_quat[1];
  final_base_pose.orientation.z = final_base_quat[2];
  final_base_pose.orientation.w =final_base_quat[3];
}


void Discretization::OGM2d_CB(const nav_msgs::OccupancyGrid::ConstPtr& msg){ 
  if(!OGM2d_rcvd_){
    ogm_2d_ = *msg;
    OGM2d_rcvd_ = true;
    ROS_INFO("Received global costmap with resolution: %f, size: %d x %d", ogm_2d_.info.resolution, ogm_2d_.info.width, ogm_2d_.info.height);
  }
}

void Discretization::associatePose(std::multimap< std::vector< double >, std::vector< double > >& baseTrnsCol,
                                         const std::vector< geometry_msgs::Pose >& grasp_poses,
                                         const std::multimap< std::vector< double >, std::vector< double > >& PoseColFilter,
                                         const float resolution, const bool arm_pose, const Eigen::Affine3d arm_to_root_eigen,
                                         const std::string planning_group, moveit::core::RobotStatePtr robot_state_ptr)
{
// retreive the parameters to choose the type of filtering  
    if (!nh.getParam("IKValid_filtering", IKValid_filt_)) {
      ROS_WARN("associatePose - Failed to get param 'IKValid_filt_' - setting to defualt: 'false'");
      IKValid_filt_ = false;
    }
    if (!nh.getParam("OCTOMAP_SRV_filtering", OCTOMAP_srv_filt_)) {
      ROS_WARN("associatePose - Failed to get param 'OCTOMAP_srv_filt_' - setting to defualt: 'false'");
      OCTOMAP_srv_filt_ = false;
    }
    if (!nh.getParam("OCTOMAP_PS_filtering", OCTOMAP_PS_filt_)) {
      ROS_WARN("associatePose - Failed to get param 'OCTOMAP_PS_filt_' - setting to defualt: 'false'");
      OCTOMAP_PS_filt_ = false;
    }
    if (!nh.getParam("OGM2d_filtering", OGM2d_filt_)) {
        ROS_WARN("associatePose - Failed to get param 'OGM2_filt_' - setting to defualt: 'false'");
        OGM2d_filt_ = false;
    }
    if (!nh.getParam("TIAGO_torso_filtering", TIAGO_torso_filt_)) { /////////////////
      ROS_WARN("associatePose - Failed to get param 'TIAGO_torso_filt_' - setting to defualt: 'false'");
      TIAGO_torso_filt_ = false;
    }
// setup the filtering data depending on the filtering used

    reachability::ReachAbility reach(nh, planning_group, true);
    
    if(arm_pose){
      Eigen::Vector3d translation = arm_to_root_eigen.translation();
      Eigen::Quaterniond eigen_quaternion(arm_to_root_eigen.rotation());
      // Convert to tf2::Transform to use into associatePose for the filtering
      arm_to_root_tf_.setOrigin(tf2::Vector3(translation.x(), translation.y(), translation.z()));
      arm_to_root_tf_.setRotation(tf2::Quaternion(eigen_quaternion.x(), eigen_quaternion.y(), eigen_quaternion.z(), eigen_quaternion.w()));
    }

    if(OCTOMAP_srv_filt_){
      OCTOMAP_rcvd_ = false;
      OCTOMAP_client_ = nh.serviceClient<octomap_msgs::GetOctomap>("/octomap_binary");
      octomap_msgs::GetOctomap srv;
      if (OCTOMAP_client_.call(srv)) {
        if (srv.response.map.data.size() > 0) {
          ROS_INFO("Octomap received from /octomap_binary service");/////////
          octomap_msgs::Octomap octomap = srv.response.map;
          octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(octomap);
          collision_octree_ = (octomap::OcTree*) abstract_tree;
          OCTOMAP_rcvd_ = true;
        } else {
          ROS_WARN("Empty octomap received from /octomap_binary service");
          return;
        }
      } else {
        ROS_ERROR("Failed to call /octomap_binary service");
        return ;
      }
    }

    if(OCTOMAP_PS_filt_){
      OCTOMAP_rcvd_ = false;
      OCTOMAP_client_ = nh.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
      moveit_msgs::GetPlanningScene srv;
      srv.request.components.components = moveit_msgs::PlanningSceneComponents::OCTOMAP;
      if (OCTOMAP_client_.call(srv)) {
        if (srv.response.scene.world.octomap.octomap.data.size() > 0) {
          ROS_INFO("Octomap received from /get_planning_scene service");/////////
          octomap_msgs::Octomap octomap = srv.response.scene.world.octomap.octomap;
          octomap::AbstractOcTree* abstract_tree = octomap_msgs::msgToMap(octomap);
          collision_octree_ = (octomap::OcTree*) abstract_tree;
          OCTOMAP_rcvd_ = true;
        } else {
          ROS_WARN("Empty octomap received from /get_planning_scene service");
          return;
        }
      } else {
        ROS_ERROR("Failed to call /get_planning_scene service");
        return ;
      }
      
    }

    if(OGM2d_filt_){ // if I want to use a 2-dimentional Occupancy Grid Map I need these parameters
      nh.getParam("OGM2d_cost_threshold",OGM2d_cost_lim_);
      nh.getParam("OGM2d_topic",OGM2d_topic_);
      OGM2d_rcvd_ = false;
      OGM2d_sub_ = nh.subscribe(OGM2d_topic_, 1, &Discretization::OGM2d_CB, this);
      int wait=0;///////////
      while(!OGM2d_rcvd_ && wait<10){///////////
        ROS_INFO("Waiting to receive occupancy grid map from %s - %d",OGM2d_topic_.c_str(), wait);
        wait+=1;
        ros::Duration(0.1).sleep();
        ros::spinOnce();
      }
      if(wait==10){////////////review
        ROS_ERROR("TIMEDOUT ON WAITING FOR %s", OGM2d_topic_.c_str());
        return;
      }
      OGM2d_sub_.shutdown();
    }

  unsigned char maxDepth = 16;
  float size_of_box = 2;
  octomap::point3d origin = octomap::point3d(0, 0, 0);
  octomap::OcTree* tree = Discretization::generateBoxTree(origin, size_of_box, resolution);
  std::vector< octomap::point3d > spCenter;
  for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(maxDepth), end = tree->end_leafs(); it != end; ++it){
    spCenter.push_back(it.getCoordinate());
  }
  
  // create a point cloud which consists of all of the possible base locations for all grasp poses and a list of base pose orientations
  std::vector< std::pair< std::vector< float >, std::vector< float > > > trns_col;
  trns_col.reserve(grasp_poses.size());
  pcl::PointCloud< pcl::PointXYZ >::Ptr cloud(new pcl::PointCloud< pcl::PointXYZ >);
  for (int i = 0; i < grasp_poses.size(); ++i)
  {
    // get grasp pose in tf format
    tf2::Vector3 grasp_vec(grasp_poses[i].position.x, grasp_poses[i].position.y, grasp_poses[i].position.z);
    tf2::Quaternion grasp_quat(grasp_poses[i].orientation.x, grasp_poses[i].orientation.y, grasp_poses[i].orientation.z, grasp_poses[i].orientation.w);
    grasp_quat.normalize();
    tf2::Transform grasp_trns;
    grasp_trns.setOrigin(grasp_vec);
    grasp_trns.setRotation(grasp_quat);

    // iterate through the inverse reach map "applied" to the grasp pose
    for (std::multimap< std::vector< double >, std::vector< double > >::const_iterator it = PoseColFilter.begin(); it != PoseColFilter.end(); ++it){
      tf2::Vector3 vec(it->second[0], it->second[1], it->second[2]);
      tf2::Quaternion quat(it->second[3], it->second[4], it->second[5], it->second[6]);
      tf2::Transform trns;
      trns.setOrigin(vec);
      trns.setRotation(quat);

      tf2::Transform new_trns;
      new_trns = grasp_trns * trns; 

      tf2::Vector3 new_trans_vec;
      tf2::Quaternion new_trans_quat;
      new_trans_vec = new_trns.getOrigin();
      new_trans_quat = new_trns.getRotation();
      new_trans_quat.normalize();

      if(!arm_pose){// robot base pose (with VerticalRobotModel method)
        if(new_trans_vec[2]< -resolution ||  new_trans_vec[2]> resolution){ // only keep poses that are on the ground + some allowance
          continue;
        }
        //filter out poses that don't have the z axis close to the perpendicular to the ground (valid for navigation)
        double tolerance = 0.2;//////////////////////////////// quite likely too big but for now i keep it like this
        tf2::Matrix3x3 rotation_matrix(new_trans_quat);
        tf2::Vector3 z_axis = rotation_matrix.getColumn(2); // Get the z axis vector then check it is close to vertical
        if(!(fabs(z_axis.x()) < tolerance && fabs(z_axis.y()) < tolerance && fabs(z_axis.z() - 1.0) < tolerance)){
          //ROS_INFO("ORIENT NOT OK: %f %f %f %f",new_trans_quat[0],new_trans_quat[1],new_trans_quat[2],new_trans_quat[3]);////
          continue;
        }else{// adjust orientation to vertical 
          double roll, pitch, yaw;
          tf2::Matrix3x3(new_trans_quat).getRPY(roll, pitch, yaw);
          tf2::Quaternion q_new;
            q_new.setRPY(0, 0, yaw);
            q_new.normalize();
            new_trans_quat = q_new;
            new_trns.setRotation(new_trans_quat);
        }
        //ROS_INFO("ORIENT OK: %f %f %f %f",new_trans_quat[0],new_trans_quat[1],new_trans_quat[2],new_trans_quat[3]);////

      }else{ // arm base pose (other methods)
        if((new_trans_vec[2]<-resolution)){ // remove positions that are below ground (with some allowance)
          continue;           
        }
        if(TIAGO_torso_filt_){ //// extra robot specific filtering
          if(planning_group=="arm"){
            if((new_trans_vec[2]>1.232||new_trans_vec[2]<0.89)){ //outside robot's fisical boundaries (torso_lift_link's height)
              continue;           
            }
          }
          //filter out poses that don't have the z axis close to the perpendicular to the ground (valid for navigation)
          double tolerance = 0.3; ////////////// bigger than what it should but for now i keep it, adjust in move_ to_bp_TIAGO
          tf2::Matrix3x3 rotation_matrix(new_trans_quat);
          tf2::Vector3 z_axis = rotation_matrix.getColumn(2); // Get the z axis vector then check it is close to vertical 
          if(!(fabs(z_axis.x()) < tolerance && fabs(z_axis.y()) < tolerance && fabs(z_axis.z() - 1.0) < tolerance)){
            //ROS_INFO("ORIENT NOT OK: %f %f %f %f",new_trans_quat[0],new_trans_quat[1],new_trans_quat[2],new_trans_quat[3]); /////
            continue;
          }else{// adjust orientation to vertical 
            if(IKValid_filt_){ /////////////////////////////////////////
              double roll, pitch, yaw;
              tf2::Matrix3x3(new_trans_quat).getRPY(roll, pitch, yaw);
              tf2::Quaternion q_new;
              q_new.setRPY(0, 0, yaw);
              q_new.normalize();
              new_trans_quat = q_new;
              new_trns.setRotation(new_trans_quat);
            }
          }
        //ROS_INFO("ORIENT OK: %f %f %f %f",new_trans_quat[0],new_trans_quat[1],new_trans_quat[2],new_trans_quat[3]); /////
        }
      }

      if(OGM2d_rcvd_ && arm_pose ){ // FILTERING WITH OCCUPANCY GRID MAP FOR ARM POSES 
        tf2::Vector3 check_trans_vec;
        /////if(arm_pose){ // i need to find the corresponding robot base pose to check the robot base pose against the costmap
          tf2::Transform robot_pose_trns;
          if(TIAGO_torso_filt_){ // adjust orientation to vertical to make sure that the costmap check is correct for the robot tiago on the ground
            tf2::Transform tiago_trns;
            tiago_trns.setOrigin(new_trans_vec);
            double roll, pitch, yaw;
            tf2::Matrix3x3(new_trans_quat).getRPY(roll, pitch, yaw);
            tf2::Quaternion q_new;
            q_new.setRPY(0, 0, yaw);
            q_new.normalize();
            tiago_trns.setRotation(q_new);
            robot_pose_trns = tiago_trns*arm_to_root_tf_;
          }else{
            robot_pose_trns = new_trns*arm_to_root_tf_;
          }
          check_trans_vec = robot_pose_trns.getOrigin();
        ////}else{ // already is a robot root pose
        ////  check_trans_vec = new_trans_vec;
        ////}
        //COMPUTE COST IN THAT POSITION
          // Convert world coordinates to map indices
          int mx = static_cast<int>((check_trans_vec[0]  - ogm_2d_.info.origin.position.x) / ogm_2d_.info.resolution);
          int my = static_cast<int>((check_trans_vec[1] - ogm_2d_.info.origin.position.y) / ogm_2d_.info.resolution);
          //ROS_DEBUG("Checking point: (%f,%f) - computed mx,my = %d, %d",check_trans_vec[0] , check_trans_vec[1], mx,my);
          if (mx < 0 || mx >= ogm_2d_.info.width || my < 0 || my >= ogm_2d_.info.height){
              ROS_WARN("Point is out of bounds.");
              continue;
          }
          // Get the cost at the specified point
          int index = my * ogm_2d_.info.width + mx;
          int cost = ogm_2d_.data[index];
          ROS_DEBUG("Cost: %d", cost);
        //CHECK THE VALIDITY
          if(cost>OGM2d_cost_lim_){
            ROS_DEBUG("POINT FILTERED OUT");
            continue; //// skip the rest of the for cycle code = don't store the pose into trns_col and cloud
          } 
      }

      std::vector< float > position;
      position.reserve(3);
      position.push_back(new_trans_vec[0]);
      position.push_back(new_trans_vec[1]);
      position.push_back(new_trans_vec[2]);

      std::vector< float > orientation;
      orientation.reserve(4);
      orientation.push_back(new_trans_quat[0]);
      orientation.push_back(new_trans_quat[1]);
      orientation.push_back(new_trans_quat[2]);
      orientation.push_back(new_trans_quat[3]);
      trns_col.push_back(std::pair< std::vector< float >, std::vector< float > >(position, orientation));

      pcl::PointXYZ point;
      point.x = new_trans_vec[0];
      point.y = new_trans_vec[1];
      point.z = new_trans_vec[2];
      cloud->push_back(point);
    }
  }  // done creating base pose cloud

  // Create octree for binning the base poses
  pcl::octree::OctreePointCloudSearch< pcl::PointXYZ > base_poses_octree(resolution);
  base_poses_octree.setInputCloud(cloud);
  base_poses_octree.addPointsFromInputCloud();
  
  // Get bounding box OF THE UNION MAP AT THIS POINT for checking search validity
  double min_x, min_y, min_z, max_x, max_y, max_z;
  base_poses_octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
  ROS_DEBUG("associatePoses: BOUNDING BOX x:[ %f : %f] , y:[ %f : %f] , z:[ %f : %f]", min_x, max_x, min_y, max_y, min_z, max_z);

  int sphCount=0;/////////////////////////
  // add all base poses from cloud to an octree
  for (int i = 0; i < spCenter.size(); i++){
    pcl::PointXYZ searchPoint;
    searchPoint.x = spCenter[i].x();
    searchPoint.y = spCenter[i].y();
    searchPoint.z = spCenter[i].z();

    // Find all base poses that lie in the given voxel
    std::vector< int > pointIdxVec;
    ROS_DEBUG("Search Point X: %f, Y: %f, Z: %f", searchPoint.x, searchPoint.y, searchPoint.z);

    // Check that the search point is valid
    // https://github.com/ros-industrial-consortium/reuleaux/issues/68
    bool isInBox = (searchPoint.x >= min_x && searchPoint.x <= max_x) && (searchPoint.y >= min_y && searchPoint.y <= max_y) && (searchPoint.z >= min_z && searchPoint.z <= max_z);

    if (isInBox){      
      if(OGM2d_rcvd_ and !arm_pose ){ // FILTERING WITH OCCUPANCY GRID MAP FOR ARM POSES
        // Convert world coordinates to map indices
        int mx = static_cast<int>((searchPoint.x  - ogm_2d_.info.origin.position.x) / ogm_2d_.info.resolution);
        int my = static_cast<int>((searchPoint.y - ogm_2d_.info.origin.position.y) / ogm_2d_.info.resolution);
        //ROS_DEBUG("Checking point: (%f,%f) - computed mx,my = %d, %d",searchPoint.x , searchPoint.y, mx,my);
        if (mx < 0 || mx >= ogm_2d_.info.width || my < 0 || my >= ogm_2d_.info.height){
            ROS_WARN("Point is out of bounds.");
            continue;
        }
        // Get the cost at the specified point
        int index = my * ogm_2d_.info.width + mx;
        int cost = ogm_2d_.data[index];
        ROS_DEBUG("Cost: %d", cost);
      //CHECK THE VALIDITY
        if(cost>OGM2d_cost_lim_){
          ROS_DEBUG("POINT FILTERED OUT");
          continue; //// skip the rest of the for cycle code = don't store the pose into trns_col and cloud
        } 
      }      
      sphCount+=1;/////////////////////////////
      //// other option for filtering with OGM - discarted
      base_poses_octree.voxelSearch(searchPoint, pointIdxVec);
                
      if (pointIdxVec.size() > 0){
        std::vector< double > voxel_pos;
        voxel_pos.reserve(3);
        voxel_pos.push_back(searchPoint.x);
        voxel_pos.push_back(searchPoint.y);
        voxel_pos.push_back(searchPoint.z);
        if(OCTOMAP_rcvd_){ // FILTERING WITH AN OCTOMAP - filter the nodes in space that are accupied (before checking if they "contain" possible base poses)
          if(arm_pose){ // Find the node corresponding to that point 
            octomap::OcTreeNode* node = collision_octree_->search(searchPoint.x, searchPoint.y, searchPoint.z);
            if(!node || collision_octree_->isNodeOccupied(node)){
                continue;
            }  
          }else{ // check the whole column of nodes in the octomap (kind of like checking the projection of the map onto the ground)
            bool collision = false;
            double coll_tree_res = collision_octree_->getResolution();
            double mx,my,max_z;
            collision_octree_->getMetricMax(mx, my, max_z);
            for (double i = coll_tree_res*1.25; i < max_z; i+=coll_tree_res){
              octomap::OcTreeNode* node = collision_octree_->search(searchPoint.x, searchPoint.y, i);
              if(node && collision_octree_->isNodeOccupied(node)){
                collision = true;
                break;
              }
            }
            if(collision){
              continue;
            }
          }
        }
        for (size_t j = 0; j < pointIdxVec.size(); ++j){ // For a given voxel, add all base poses to the multimap for later retreival
          // Get the base pose for a given index found in a voxel
          std::vector< double > base_pose_vec;
          base_pose_vec.reserve(3);
          std::vector< float > position = trns_col[pointIdxVec[j]].first;
          //base_pose_vec.push_back(double(position[0]));
          //base_pose_vec.push_back(double(position[1]));
          //base_pose_vec.push_back(double(position[2]));
          base_pose_vec.push_back(voxel_pos[0]);
          base_pose_vec.push_back(voxel_pos[1]);
          base_pose_vec.push_back(voxel_pos[2]);
          std::vector< float > orientation = trns_col[pointIdxVec[j]].second;
          base_pose_vec.push_back(double(orientation[0]));
          base_pose_vec.push_back(double(orientation[1]));
          base_pose_vec.push_back(double(orientation[2]));
          base_pose_vec.push_back(double(orientation[3]));

          // FILTER OUT POSES DON'T GIVE A VALID JOINT SOLUTION
          if(IKValid_filt_){
            //if(!use_IKFast_)   -  IRL i should always be able to use this bc i always have moveit going anyway
            geometry_msgs::Pose base_pose;

            if(arm_pose){ // if it is a arm base pose then I need to transform it to the robot base pose
              tf2::Vector3 vec(base_pose_vec[0], base_pose_vec[1], base_pose_vec[2]);
              tf2::Quaternion quat(base_pose_vec[3], base_pose_vec[4], base_pose_vec[5], base_pose_vec[6]);
              tf2::Transform trns;
              trns.setOrigin(vec);
              trns.setRotation(quat);
              tf2::Transform robot_pose_trns;
              robot_pose_trns = trns*arm_to_root_tf_;
              tf2::Vector3 robot_vec = robot_pose_trns.getOrigin();
              tf2::Quaternion robot_quat = robot_pose_trns.getRotation();
              robot_quat.normalize();
              base_pose.position.x = robot_vec[0];
              base_pose.position.y = robot_vec[1];
              base_pose.position.z = robot_vec[2];
              base_pose.orientation.x = robot_quat[0];
              base_pose.orientation.y = robot_quat[1];
              base_pose.orientation.z = robot_quat[2];
              base_pose.orientation.w = robot_quat[3];
            }else{ // if it is a robot base pose
              utility::vectorToPose(base_pose_vec, base_pose);
            }
            std::vector<double> sol = reach.getValidIKSol(base_pose, robot_state_ptr, grasp_poses[i]);
            if(sol.size() == 0){
              continue; // no valid IK solution
            }else if (sol.size()==1){ // meaning it is = -999999
              ROS_ERROR("associatePose - could not perform the IK check");
              return;
            }
          }
          baseTrnsCol.insert(std::pair< std::vector< double >, std::vector< double > >(voxel_pos, base_pose_vec));
        }
      }
    }
  }

  ROS_INFO("//////////////////////////// dimensione for graspxIRM = %d",(grasp_poses.size()*PoseColFilter.size()));
  ROS_INFO("//////////////////////////// trns_col size = %d", trns_col.size());
  ROS_INFO("//////////////////////////// for spCenter = %d", spCenter.size());
  ROS_INFO("//////////////////////////// sfere per cui fa il voxel search = %d", sphCount);
  
}

} //end namespace 
