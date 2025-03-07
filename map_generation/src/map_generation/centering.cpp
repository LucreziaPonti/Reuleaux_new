#include<map_generation/centering.h>

namespace reuleaux
{
Centering::Centering(ros::NodeHandle& node, geometry_msgs::Pose arm_base_pose)
  :arm_base_pose_(arm_base_pose)
{
  nh_ = node;
  final_ws_.WsSpheres.clear();
  init_ws_.WsSpheres.clear();

}


 void Centering::setInitialWorkspace(const map_generation::WorkSpace &initial_ws)
 {
   init_ws_ = initial_ws;
   reuleaux::getPoseAndSphereSize(init_ws_, sphere_size_, pose_size_);
 }

 void Centering::getFinalWorkspace(map_generation::WorkSpace &final_ws)
 {
   final_ws = final_ws_;
   /*ROS_INFO("CHECK IN get final ws ");
    for(int i=0;i<final_ws.WsSpheres.size();++i){
      ROS_INFO("centered sphere %d : %f,%f,%f",i+1,final_ws.WsSpheres[i].point.x,final_ws.WsSpheres[i].point.y,final_ws.WsSpheres[i].point.z);
      for(int j=0;j<final_ws.WsSpheres[i].poses.size();++i){
        ROS_INFO("centered pose %d : %f,%f,%f",j+1,final_ws.WsSpheres[i].poses[j].position.x,final_ws.WsSpheres[i].poses[j].position.y,final_ws.WsSpheres[i].poses[j].position.z);
      }
    }*/
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

 bool Centering::createCentering(const map_generation::WorkSpace& ws)
 {
   map_generation::WsSphere wss;
   int sp_size = sphere_size_;
   for(int i=0;i<sp_size;++i){
      ROS_DEBUG("Centering sphere: %d / %d", i+1,sp_size);
      //geometry_msgs::Pose og_spherepose;
      //og_spherepose.position=ws.WsSpheres[i].point;
      //geometry_msgs::Pose centered_spherepose;
      //transformTaskpose(arm_base_pose_,og_spherepose,centered_spherepose);
      
      //wss.point.x=centered_spherepose.position.x;
      //wss.point.y=centered_spherepose.position.y;
      //wss.point.z=centered_spherepose.position.z;
      //metodo brutale:
      wss.point.x=ws.WsSpheres[i].point.x-arm_base_pose_.position.x;
      wss.point.y=ws.WsSpheres[i].point.y-arm_base_pose_.position.y;
      wss.point.z=ws.WsSpheres[i].point.z-arm_base_pose_.position.z;
      //ROS_INFO("debug---- sphere pose %f %f %f --- %f %f %f ",ws.WsSpheres[i].point.x,ws.WsSpheres[i].point.y,ws.WsSpheres[i].point.z,wss.point.x,wss.point.y,wss.point.z);
      for(int j=0;j<ws.WsSpheres[i].poses.size();++j)      {
        ROS_DEBUG("Centering pose %d of sphere %d",j+1,i+1);
        //geometry_msgs::Pose og_pose= ws.WsSpheres[i].poses[j];
        geometry_msgs::Pose centered_pose;
        //transformTaskpose(arm_base_pose_,og_pose,centered_pose);
        centered_pose.position.x=ws.WsSpheres[i].poses[j].position.x-arm_base_pose_.position.x;
        centered_pose.position.y=ws.WsSpheres[i].poses[j].position.y-arm_base_pose_.position.y;
        centered_pose.position.z=ws.WsSpheres[i].poses[j].position.z-arm_base_pose_.position.z;
        //ROS_INFO("debug---- pose %f %f %f --- %f %f %f ",ws.WsSpheres[i].poses[j].position.x,ws.WsSpheres[i].poses[j].position.y,ws.WsSpheres[i].poses[j].position.z,centered_pose.position.x,centered_pose.position.y,centered_pose.position.z);
        centered_pose.orientation=ws.WsSpheres[i].poses[j].orientation;

        wss.poses.push_back(centered_pose); 
      }
      wss.ri = ws.WsSpheres[i].ri;

    final_ws_.WsSpheres.push_back(wss);
   }

    
   final_ws_.resolution = init_ws_.resolution;
   /*int posess_n;
    reuleaux::getPoseAndSphereSize(final_ws_, sp_size, posess_n);
    ROS_INFO("CHECK IN CENTERING SP= %d , POS= %d",sp_size,posess_n);
    for(int i=0;i<sp_size;++i){
      ROS_INFO("centered sphere %d : %f,%f,%f",i+1,final_ws_.WsSpheres[i].point.x,final_ws_.WsSpheres[i].point.y,final_ws_.WsSpheres[i].point.z);
      for(int j=0;j<final_ws_.WsSpheres[i].poses.size();++i){
        ROS_INFO("centered pose %d : %f,%f,%f",j+1,final_ws_.WsSpheres[i].poses[j].position.x,final_ws_.WsSpheres[i].poses[j].position.y,final_ws_.WsSpheres[i].poses[j].position.z);
      }
    }*/

   // Added as there was no return from this function, not sure if there ought to be a false case...
   return true;
 }
 
}

