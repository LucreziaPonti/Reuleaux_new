#ifndef DISCRETIZATION_H
#define DISCRETIZATION_H
#include <iostream>
#include <ros/ros.h>

#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Point.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <map_generator/WorkSpace.h>
#include <tf/LinearMath/Quaternion.h>

#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_ros/surface/convex_hull.h>
#include <pcl_ros/transforms.h>

#include <nav_msgs/OccupancyGrid.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/GetOctomap.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h> 
#include <octomap_msgs/conversions.h>
#include <octomap/AbstractOcTree.h>


#include <map_generator/utility.h>
#include <map_generator/reachability.h>

namespace discretization
{

class Discretization
{
public:
  Discretization();
  Discretization(geometry_msgs::Pose pose, double resolution = 0.1, double radius = 0.8);

  int getNumOfSpheres();
  int getNumOfPoses();
  void getInitialWorkspace(map_generator::WorkSpace& ws);
  void discretize();

  void getCenters(std::vector<geometry_msgs::Point>& points);

  octomap::OcTree* generateBoxTree(const octomap::point3d& origin, float diameter, float resolution);
  void createPosesOnSphere(const octomap::point3d& origin, const double r, std::vector<geometry_msgs::Pose> &poses);
  
  //! Converts geometry_msgs::Pose to Eigen Vector
  void poseToEigenVector(const geometry_msgs::Pose& pose, Eigen::VectorXd& vec);
  //! Finds optimal pose of given poses by Principal Component Optimization
  void findOptimalPosebyPCA(const std::vector< geometry_msgs::Pose >& probBasePoses, geometry_msgs::Pose& final_base_pose);
  //! Given grasp poses and multimap structure of an inverse reachability map, transforms every pose of the ir map with
  //grasp poses, and calculates nearest neighbor search to associate poses with belonging spheres.
  void associatePose(std::multimap<std::vector<double>, std::vector<double> > &baseTrnsCol,
    const std::vector< geometry_msgs::Pose >& grasp_poses,
    const std::multimap<std::vector<double>, std::vector<double> > &PoseColFilter, const float resolution, 
    const bool arm_pose, const Eigen::Affine3d arm_to_root_eigen,
    const std::string planning_group, moveit::core::RobotStatePtr robot_state_ptr);


  /*  //! Compare two vectors, of length 3, for multimap search ////////////non penso venga usato
  struct vec_comp_
  {
    bool operator()(const std::vector< float >& v1, const std::vector< float >& v2) const
    {
      // TODO: need to add tolerance as a function of the map resolution; resolution maybe needs to be a class variable
      // but this appears to work fine for now
      float tol = 0.001;
      return (fabs(v1[0] - v2[0]) < tol) && (fabs(v1[1] - v2[1]) < tol) && (fabs(v1[2] - v2[2]) < tol);
    }
  };
    */
private:
  
  void createCenters(octomap::OcTree* tree, std::vector<geometry_msgs::Point>& centers);
  void createPoses(const std::vector<geometry_msgs::Point>& centers, std::vector<geometry_msgs::Pose>& poses);
  
  double resolution_;
  double radius_;
  octomap::point3d center_;
  unsigned char max_depth_;
  std::vector<geometry_msgs::Point> centers_;
  std::vector<geometry_msgs::Pose> poses_;
  map_generator::WorkSpace ws_;

// FILTERING OF UNION MAP IN ASSOCIATE POSES /////////////////////////////////
  ros::NodeHandle nh; 
  // PARAMETERS FOR THE FILTERING OF THE UNION MAP IN ASSOCIATEPOSES
  bool IKValid_filt_; // to perform the IK request to check for valid IK solutions

  tf2::Transform arm_to_root_tf_;
  bool TIAGO_torso_filt_;

  // 2-dimentional OCCUPANCY GRID MAP (e.g. global costmap - projected map of the octomap)
  bool OGM2d_filt_;
  ros::Subscriber OGM2d_sub_;
  std::string OGM2d_topic_;
  bool OGM2d_rcvd_ = false;
  int OGM2d_cost_lim_; // limit of the "cell"'s cost after which the point is filtered out 
  nav_msgs::OccupancyGrid ogm_2d_;
  void OGM2d_CB(const nav_msgs::OccupancyGrid::ConstPtr& msg);

  // octomap (form OCTOMAP_SERVER or from PLANNING SCENE)
  bool OCTOMAP_srv_filt_; // get octomap from octomap server
  bool OCTOMAP_PS_filt_; // get octomap from planning scene

  ros::ServiceClient OCTOMAP_client_;
  bool OCTOMAP_rcvd_ = false;
  octomap::OcTree* collision_octree_;

};

}//end namespace reuleaux


#endif // DISCRETIZATION_H
