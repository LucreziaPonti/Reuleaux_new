#ifndef CENTERING_H
#define CENTERING_H
#include <ros/ros.h>
#include <map_generator/WorkSpace.h>
#include <map_generator/utility.h>
#include <Eigen/Eigen>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <cmath>
namespace centering
{
class Centering
{
public:
  Centering(ros::NodeHandle& node, geometry_msgs::Pose arm_base_pose);
  void setOriginalWorkspace(const map_generator::WorkSpace& og_initial_ws);
  void setInitialWorkspace(const map_generator::WorkSpace& initial_ws);
  void getFinalWorkspace(map_generator::WorkSpace& final_ws);
  bool createCenteredWorkspace();


private:
  geometry_msgs::Pose arm_base_pose_;
  void transformTaskpose(const geometry_msgs::Pose& arm_base_pose, const geometry_msgs::Pose& pose_in, geometry_msgs::Pose& pose_out);
  bool createCentering(const map_generator::WorkSpace& ws);
  float truncateToDecimalPlaces(float value, int decimalPlaces);

  ros::NodeHandle nh_;
  map_generator::WorkSpace init_ws_;
  map_generator::WorkSpace final_ws_;
  int pose_size_;
  int sphere_size_;


};

}//end namespace reuleaux

#endif // CENTERING_H
