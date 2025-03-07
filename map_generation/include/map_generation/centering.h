#ifndef CENTERING_H
#define CENTERING_H
#include <ros/ros.h>
#include <map_generation/WorkSpace.h>
#include <map_generation/utility.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>

namespace reuleaux
{
class Centering
{
public:
  Centering(ros::NodeHandle& node, geometry_msgs::Pose arm_base_pose);
  void setOriginalWorkspace(const map_generation::WorkSpace& og_initial_ws);
  void setInitialWorkspace(const map_generation::WorkSpace& initial_ws);
  void getFinalWorkspace(map_generation::WorkSpace& final_ws);
  bool createCenteredWorkspace();


private:
  geometry_msgs::Pose arm_base_pose_;
  void transformTaskpose(const geometry_msgs::Pose& arm_base_pose, const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);
  bool createCentering(const map_generation::WorkSpace& ws);


  ros::NodeHandle nh_;
  map_generation::WorkSpace init_ws_;
  map_generation::WorkSpace final_ws_;
  int pose_size_;
  int sphere_size_;


};

}//end namespace reuleaux

#endif // CENTERING_H
