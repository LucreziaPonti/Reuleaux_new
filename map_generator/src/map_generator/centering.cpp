#include<map_generator/centering.h>

namespace centering
{
Centering::Centering(ros::NodeHandle& node, geometry_msgs::Pose arm_base_pose)
  :arm_base_pose_(arm_base_pose)
{
  nh_ = node;
  final_ws_.WsSpheres.clear();
  init_ws_.WsSpheres.clear();

}


 void Centering::setInitialWorkspace(const map_generator::WorkSpace &initial_ws)
 {
   init_ws_ = initial_ws;
   utility::getPoseAndSphereSize(init_ws_, sphere_size_, pose_size_);
 }

 void Centering::getFinalWorkspace(map_generator::WorkSpace &final_ws)
 {
   final_ws = final_ws_;
 }

 void Centering::transformTaskpose(const geometry_msgs::Pose &arm_base_pose, const geometry_msgs::Pose &pose_in, geometry_msgs::Pose &pose_out)
 {
   //First get the transform between the center to world (inv)
   Eigen::Affine3d arm_base_pose_tf;
   tf::poseMsgToEigen(arm_base_pose, arm_base_pose_tf); //(world)T(arm_base)
   Eigen::Affine3d arm_base_pose_to_world_tf = arm_base_pose_tf.inverse(); //(arm_base)T(world)
   //Get the transform between the task pose to world
   Eigen::Affine3d reach_pose_tf;
   tf::poseMsgToEigen(pose_in, reach_pose_tf);//(world)T(pose)
   //transform the task pose to base pose at center. Now we can get Ik for this pose
    tf::poseEigenToMsg(arm_base_pose_to_world_tf*reach_pose_tf, pose_out); //(arm_base)T(pose)=(arm_base)T(world)*(world)T(pose)
 }


 bool Centering::createCenteredWorkspace()
 {
   if(createCentering(init_ws_))
     return true;
   else
     return false;
 }

float Centering::truncateToDecimalPlaces(float value, int decimalPlaces) {
  float factor = std::pow(10.0f, decimalPlaces);
  return std::trunc(value * factor) / factor;
}

 bool Centering::createCentering(const map_generator::WorkSpace& ws)
 {
  
   int sp_size = sphere_size_;
   for(int i=0;i<sp_size;++i){
      map_generator::WsSphere wss;
      ROS_DEBUG("Centering sphere: %d / %d", i+1,sp_size);
      //ROS_INFO("value of sphere %d : %f,%f,%f",i+1,ws.WsSpheres[i].point.x,ws.WsSpheres[i].point.y,ws.WsSpheres[i].point.z);
      geometry_msgs::Pose og_spherepose;
      og_spherepose.position.x=ws.WsSpheres[i].point.x;
      og_spherepose.position.y=ws.WsSpheres[i].point.y;
      og_spherepose.position.z=ws.WsSpheres[i].point.z;
      og_spherepose.orientation.w=1; //needed to have a valid pose to apply the transform to 
      geometry_msgs::Pose centered_spherepose;
      transformTaskpose(arm_base_pose_,og_spherepose,centered_spherepose);
      wss.point.x=truncateToDecimalPlaces(centered_spherepose.position.x, 6);
      wss.point.y=truncateToDecimalPlaces(centered_spherepose.position.y, 6);
      wss.point.z=truncateToDecimalPlaces(centered_spherepose.position.z, 6);
      //ROS_INFO("centered sphere %d : %f,%f,%f",i+1,wss.point.x,wss.point.y,wss.point.z);

      for(int j=0;j<ws.WsSpheres[i].poses.size();++j)      {
        //ROS_DEBUG("Centering pose %d of sphere %d",j+1,i+1);
        geometry_msgs::Pose centered_pose;
        transformTaskpose(arm_base_pose_,ws.WsSpheres[i].poses[j],centered_pose);
        //// extra thing bc it does wierd stuff (change signs in some values)
        centered_pose.orientation.x=ws.WsSpheres[i].poses[j].orientation.x;
        centered_pose.orientation.y=ws.WsSpheres[i].poses[j].orientation.y;
        centered_pose.orientation.z=ws.WsSpheres[i].poses[j].orientation.z;
        centered_pose.orientation.w=ws.WsSpheres[i].poses[j].orientation.w;
        //ROS_INFO("centered pose %d : %f,%f,%f",j+1,centered_pose.position.x,centered_pose.position.y,centered_pose.position.z);
        wss.poses.push_back(centered_pose); 
      }
      wss.ri = ws.WsSpheres[i].ri;

    final_ws_.WsSpheres.push_back(wss);
   }

    
   final_ws_.resolution = init_ws_.resolution;

   // Added as there was no return from this function, not sure if there ought to be a false case...
   return true;
 }
 
}

