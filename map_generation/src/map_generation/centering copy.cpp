#include<map_generation/centering.h>

namespace reuleaux
{
Centering::Centering(ros::NodeHandle& node, geometry_msgs::Pose center_pose)
  :center_pose_(center_pose)
{
  nh_ = node;
  final_ws_.WsSpheres.clear();
  init_ws_.WsSpheres.clear();
  og_init_ws_.WsSpheres.clear();

}

  void Centering::setOriginalWorkspace(const map_generation::WorkSpace &og_initial_ws)
 {
   og_init_ws_ = og_initial_ws;
   reuleaux::getPoseAndSphereSize(og_init_ws_, og_sphere_size_, og_pose_size_);
 }

 void Centering::setInitialWorkspace(const map_generation::WorkSpace &initial_ws)
 {
   init_ws_ = initial_ws;
   reuleaux::getPoseAndSphereSize(init_ws_, sphere_size_, pose_size_);
 }

 void Centering::getFinalWorkspace(map_generation::WorkSpace &final_ws)
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

 bool Centering::createCentering(const map_generation::WorkSpace& ws)
 {
   reuleaux::MultiMap ws_map;
   reuleaux::MapVecDouble sp_map;
   int sp_size = sphere_size_;
   for(int i=0;i<sp_size;++i)
   {
      ROS_DEBUG("Centering sphere: %d / %d", i+1,sp_size);
      geometry_msgs::Pose og_spherepose;
      og_spherepose.position=ws.WsSpheres[i].point;
      geometry_msgs::Pose centered_spherepose;
      transformTaskpose(center_pose_,og_spherepose,centered_spherepose);
      std::vector<double> sp_vec;
      reuleaux::pointToVector(centered_spherepose.position, sp_vec);

      for(int j=0;j<ws.WsSpheres[i].poses.size();++j)      {
        ROS_DEBUG("Centering pose %d of sphere %d",j+1,i+1);
        geometry_msgs::Pose og_pose= ws.WsSpheres[i].poses[j];
        geometry_msgs::Pose centered_pose;
        transformTaskpose(center_pose_,og_pose,centered_pose);
        std::vector<double> sp_pose;
        reuleaux::poseToVector(centered_pose, sp_pose);
        ws_map.insert(std::make_pair(sp_vec, sp_pose));
        
      }
   }

   // il resto lo lascio come faceva in reachability .... dovrebbe ridare le stesse cose solo traslate teoricamente
   // !!!!!! mi servono le dimesioni/quantità del workspace iniziale iniziale per ricalcolare bene d 
   for(reuleaux::MultiMap::iterator it=ws_map.begin(); it!=ws_map.end();++it)
   {
     std::vector<double> sp_coord = it->first;
     float d = float(ws_map.count(sp_coord)) / (og_pose_size_ /og_sphere_size_) * 100; 
     sp_map.insert(std::make_pair(it->first, double(d)));
   }
   for (reuleaux::MapVecDouble::iterator it = sp_map.begin(); it != sp_map.end(); ++it)
   {
     map_generation::WsSphere wss;
     wss.point.x = (it->first)[0];
     wss.point.y = (it->first)[1];
     wss.point.z = (it->first)[2];
     wss.ri = it->second;
     for (MultiMap::iterator it1 = ws_map.lower_bound(it->first); it1 != ws_map.upper_bound(it->first); ++it1)
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
}
