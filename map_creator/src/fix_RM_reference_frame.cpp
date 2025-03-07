#include <ros/ros.h>
#include <ros/package.h>

#include <pcl/point_cloud.h>
#include <pcl/octree/octree.h>

#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>

#include <map_creator/sphere_discretization.h>
#include <map_creator/kinematics.h>
#include <map_creator/hdf5_dataset.h>
#include "map_creator/WorkSpace.h"
#include <map>

#include <tf2/LinearMath/Transform.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <ctime>

#include <string>
#include <time.h>

#include <boost/format.hpp>
//struct stat st;

int main(int argc, char **argv)
{
ros::init(argc, argv, "fix_RM_reference_frame");
  ros::NodeHandle n;
  // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1);
  time_t startit, finish;
  time(&startit);
  kinematics::Kinematics k;
  std::string file;
  std::string path(ros::package::getPath("map_generation") + "/maps/");
  std::string filename;
  const char *input_FILE;

  if (argc < 2)
  {
    ROS_ERROR_STREAM("Please provide the name of the reachability map. If you have not created it yet, Please create "
                     "the map by running the create reachability map node in map_creator or map_generation package");
    return 0;
  }

  else if (argc == 2)
  {
    ROS_INFO("Creating map with default name.");
    input_FILE = argv[1];
    if(!boost::filesystem::exists(input_FILE))
    {
      ROS_ERROR("Input file does not exist");
      return false;
    }
    else
    {
      float res;
      hdf5_dataset::Hdf5Dataset h5_res(argv[1]);
      h5_res.open();
      h5_res.h5ToResolution(res);
      h5_res.close();
      file =  str(boost::format("%s_%d_reachability_NEW_frame.h5") % k.getRobotName() % res);
      filename = path + file;
    }
  }

  else if (argc == 3)
  {
    input_FILE = argv[1];
    std::string str(argv[2]);
    if(!boost::filesystem::exists(input_FILE))
    {
      ROS_ERROR("Input file does not exist");
      return false;
    }
    else
    {
      if(std::strchr(str.c_str(), '/'))
      {
        filename = argv[2];
      }
      else
        filename = path + str;
    }
  }

  ROS_INFO("creo la trasforamta da usare per cambiare le posizioni");
  //serve la trasformata (arm_1_link)T(base_footprint)
  tf2::Vector3 vec(0.062, 0, -0.888);
  ROS_INFO("trasf: %f %f %f ",vec[0], vec[1], vec[2]);
  tf2::Quaternion quat(0,0,0,1);
  tf2::Transform frame_transf;
  frame_transf.setOrigin(vec);
  frame_transf.setRotation(quat);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {
    MultiMapPtr pose_col_filter;
    MapVecDoublePtr sphere_col;
    float res;
    ROS_INFO("DEBUG------ inizio estrazione dati da mappa originale");
    hdf5_dataset::Hdf5Dataset h5file(input_FILE);
    h5file.open();
    h5file.loadMapsFromDataset(pose_col_filter, sphere_col, res);
    ROS_INFO("DEBUG------ inizio pose col modified");
    
    std::multimap< std::vector< float >, std::vector< float > > trns_col;
    for(MultiMapPtr::iterator it = pose_col_filter.begin(); it!= pose_col_filter.end(); ++it)
    {
      tf2::Vector3 vec1((*it->first)[0], (*it->first)[1], (*it->first)[2]);
      tf2::Quaternion quat1((*it->first)[3], (*it->first)[4], (*it->first)[5], (*it->first)[6]);
      tf2::Transform trns1;
      trns1.setOrigin(vec1);
      trns1.setRotation(quat1);
      tf2::Transform trns1_mod;
      trns1_mod = frame_transf*trns1;
      tf2::Vector3 vec1_mod;
      tf2::Quaternion quat1_mod;
      vec1_mod = trns1_mod.getOrigin();
      quat1_mod = trns1_mod.getRotation();
      quat1_mod.normalize();
      std::vector<float> pose1; 
      pose1.push_back(vec1_mod[0]);
      pose1.push_back(vec1_mod[1]);
      pose1.push_back(vec1_mod[2]);
      pose1.push_back(quat1_mod[0]);
      pose1.push_back(quat1_mod[1]);
      pose1.push_back(quat1_mod[2]);
      pose1.push_back(quat1_mod[3]);
      tf2::Vector3 vec2((*it->second)[0], (*it->second)[1], (*it->second)[2]);
      tf2::Quaternion quat2((*it->second)[3], (*it->second)[4], (*it->second)[5], (*it->second)[6]);
      tf2::Transform trns2;
      trns2.setOrigin(vec2);
      trns2.setRotation(quat2);
      tf2::Transform trns2_mod;
      trns2_mod = frame_transf*trns2;
      tf2::Vector3 vec2_mod;
      tf2::Quaternion quat2_mod;
      vec2_mod = trns2_mod.getOrigin();
      quat2_mod = trns2_mod.getRotation();
      quat2_mod.normalize();
      std::vector<float> pose2; 
      pose2.push_back(vec2_mod[0]);
      pose2.push_back(vec2_mod[1]);
      pose2.push_back(vec2_mod[2]);
      pose2.push_back(quat2_mod[0]);
      pose2.push_back(quat2_mod[1]);
      pose2.push_back(quat2_mod[2]);
      pose2.push_back(quat2_mod[3]);
     
      
      trns_col.insert(std::pair< std::vector< float >, std::vector< float > >(pose1, pose2));
    }
    MultiMapPtr pose_col_modified;
    std::multimap< std::vector< float >, std::vector< float > >::iterator it1;
    for (it1 = trns_col.begin(); it1 != trns_col.end(); ++it1)
    {
      std::vector< double >* pose1 = new std::vector<double>();
      pose1->push_back(it1->first[0]);
      pose1->push_back(it1->first[1]);
      pose1->push_back(it1->first[2]);
      pose1->push_back(it1->first[3]);
      pose1->push_back(it1->first[4]);
      pose1->push_back(it1->first[5]);
      pose1->push_back(it1->first[6]);
      std::vector< double >* pose2 = new std::vector<double>();
      pose2->push_back(it1->second[0]);
      pose2->push_back(it1->second[1]);
      pose2->push_back(it1->second[2]);
      pose2->push_back(it1->second[3]);
      pose2->push_back(it1->second[4]);
      pose2->push_back(it1->second[5]);
      pose2->push_back(it1->second[6]);

      pose_col_modified.insert(std::make_pair(pose1, pose2));
    }

    std::multimap< std::vector< float >, double> trns_sphere_col;
    for(MapVecDoublePtr::iterator it = sphere_col.begin(); it!= sphere_col.end(); ++it)
    {
      tf2::Vector3 vec1((*it->first)[0], (*it->first)[1], (*it->first)[2]);
      tf2::Quaternion quat1((*it->first)[3], (*it->first)[4], (*it->first)[5], (*it->first)[6]);
      tf2::Transform trns1;
      trns1.setOrigin(vec1);
      trns1.setRotation(quat1);
      tf2::Transform trns1_mod;
      trns1_mod = frame_transf*trns1;
      tf2::Vector3 vec1_mod;
      tf2::Quaternion quat1_mod;
      vec1_mod = trns1_mod.getOrigin();
      quat1_mod = trns1_mod.getRotation();
      quat1_mod.normalize();
      std::vector<float> pose1; 
      pose1.push_back(vec1_mod[0]);
      pose1.push_back(vec1_mod[1]);
      pose1.push_back(vec1_mod[2]);
      pose1.push_back(quat1_mod[0]);
      pose1.push_back(quat1_mod[1]);
      pose1.push_back(quat1_mod[2]);
      pose1.push_back(quat1_mod[3]);
      
      double idx= it->second;

      trns_sphere_col.insert(std::pair< std::vector< float >, double >(pose1, idx));
    }
    MapVecDoublePtr sphere_color;
    std::multimap< std::vector< float >, double>::iterator it2;
    for (it2 = trns_sphere_col.begin(); it2 != trns_sphere_col.end(); ++it2)
    {
      std::vector< double >* pose1 = new std::vector<double>();
      pose1->push_back(it2->first[0]);
      pose1->push_back(it2->first[1]);
      pose1->push_back(it2->first[2]);
      pose1->push_back(it2->first[3]);
      pose1->push_back(it2->first[4]);
      pose1->push_back(it2->first[5]);
      pose1->push_back(it2->first[6]);
      sphere_color.insert(std::make_pair(pose1, it2->second));
    }

   ROS_INFO("All the poses have Processed. Now saving data to a inverse Reachability Map.");

   hdf5_dataset::Hdf5Dataset irm_h5(filename);
   irm_h5.saveReachMapsToDataset(pose_col_modified, sphere_color, res);


   time(&finish);
   double dif = difftime(finish, startit);
   ROS_INFO("Elasped time is %.2lf seconds.", dif);
   ROS_INFO("Completed");
   ros::spinOnce();
    // sleep(10000);
   return 1;
   loop_rate.sleep();
   count;
  }

  return 0;
}
