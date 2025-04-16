// The spheres and poses are fused in a single dataset, instead of two datasets for sphere and poses
#include <ros/ros.h>
#include <ros/package.h>
#include <octomap/octomap.h>
#include <octomap/MapCollection.h>
#include <octomap/math/Utils.h>
#include <tf2/LinearMath/Quaternion.h>
#include "geometry_msgs/PoseArray.h"
#include <map>
#include <sys/types.h>
#include <sys/stat.h>
#include <string>
#include <sstream>
#include <map_generator/utility.h>
#include <map_generator/discretization.h>
#include <map_generator/kinematics.h>
#include <map_generator/hdf5_dataset.h>

#include <boost/format.hpp>

//struct stat st;

typedef std::vector<std::pair< std::vector< double >, const std::vector< double >* > > MultiVector;
//typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMap;

bool isFloat(std::string s)
{
  std::istringstream iss(s);
  float dummy;
  iss >> std::noskipws >> dummy;
  return iss && iss.eof();  // Result converted to bool
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "OG_create_RM");
  ros::NodeHandle n;
  ros::Time startit = ros::Time::now();
  float resolution = 0.08;
  kinematics::Kinematics k;
  std::string file = str(boost::format("%s_r%d_reachability.h5") % k.getRobotName() % resolution);
  std::string path(ros::package::getPath("map_generator") + "/maps/");
  std::string filename;
  if (argc == 2)
  {
    if (!isFloat(argv[1]))
    {
      ROS_ERROR_STREAM("Probably you have just provided only the map filename. Hey!! The first argument is the "
                       "resolution.");
      return 0;
    }
    resolution = atof(argv[1]);
    file = str(boost::format("%s_%d_OGreachability.h5") % k.getRobotName() % resolution);
    filename = path + file;
  }

  else if (argc == 3)
  {
    std::string name;
    name = argv[2];
    if (!isFloat(argv[1]) && isFloat(argv[2]))
    {
      ROS_ERROR_STREAM("Hey!! The first argument is the resolution and the second argument is the map filename. You "
                       "messed up.");
      return 0;
    }

    else
    {
      resolution = atof(argv[1]);
      std::string str(argv[2]);
      if(std::strchr(str.c_str(), '/'))
      {
        filename = argv[2];
      }
      else
        filename = path + str;
    }
  }
  else if (argc < 2)
  {
    ROS_INFO("You have not provided any argument. So taking default values.");
    filename = path + file;
  }
  // ros::Publisher workspace_pub = n.advertise<map_generator::WorkSpace>("workspace", 10);
  ros::Rate loop_rate(10);

  int count = 0;

  while (ros::ok())
  {
    unsigned char max_depth = 16;
    unsigned char minDepth = 0;

    // A box of radius 1 is created. It will be the size of the robot+1.5. Then the box is discretized by voxels of
    // specified resolution
    // The center of every voxels are stored in a vector

    discretization::Discretization sd;
    float r = 1;
    octomap::point3d origin = octomap::point3d(0, 0, 0);  // This point will be the base of the robot
    octomap::OcTree *tree = sd.generateBoxTree(origin, r, resolution);
    std::vector< octomap::point3d > new_data;
    ROS_INFO("Creating the box and discretizing with resolution: %f", resolution);
    int sphere_count = 0;
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      sphere_count++;
    }
    new_data.reserve(sphere_count);
    for (octomap::OcTree::leaf_iterator it = tree->begin_leafs(max_depth), end = tree->end_leafs(); it != end; ++it)
    {
      new_data.push_back(it.getCoordinate());
    }

    ROS_INFO("Total no of spheres now: %lu", new_data.size());
    ROS_INFO("Please hold ON. Now discretizing spheres. May take some time");

    
    // Every discretized points on spheres are converted to pose and all the poses are saved in a multimap with their
    // corresponding sphere centers
    // If the resolution is 0.01 the programs not responds

    float radius = resolution;

    utility::VecVecDouble sphere_coord;
    sphere_coord.resize( new_data.size() );

    MultiVector pose_col;
    pose_col.reserve( new_data.size() * 50);

    for (int i = 0; i < new_data.size(); i++)
    {
      static std::vector< geometry_msgs::Pose > pose;
      utility::octreePointToVector(new_data[i], sphere_coord[i]);

      sd.createPosesOnSphere(new_data[i], radius, pose);
      for (int j = 0; j < pose.size(); j++)
      {
        static std::vector< double > point_on_sphere;
        utility::poseToVector(pose[j], point_on_sphere);
        pose_col.push_back( std::make_pair(point_on_sphere, &sphere_coord[i]));
      }
    }

    ROS_INFO("Discretization completed - %lu poses created", pose_col.size());
    ROS_INFO("Please hold ON. Now checking for IK solutions. May take some time");
    // Every pose is checked for IK solutions. The reachable poses and the their corresponsing joint solutions are
    // stored. Only the First joint solution is stored. We may need this solutions in the future. Otherwise we can show
    // the robot dancing with the joint solutions in a parallel thread
    // TODO Support for more than 6DOF robots needs to be implemented.


    utility::MultiMapPtr pose_col_filter;
    utility::VecVecDouble ik_solutions;
    ik_solutions.reserve( pose_col.size() );

    for (MultiVector::iterator it = pose_col.begin(); it != pose_col.end(); ++it)
    {
      static std::vector< double > joints(6);
      int solns;
      if (k.isIKSuccess(it->first, joints, solns))
      {
        pose_col_filter.insert( std::make_pair( it->second, &(it->first)));
        ik_solutions.push_back(joints);
        // cout<<it->first[0]<<" "<<it->first[1]<<" "<<it->first[2]<<" "<<it->first[3]<<" "<<it->first[4]<<"
        // "<<it->first[5]<<" "<<it->first[6]<<endl;
      }
    }

    ROS_INFO("Total number of poses: %lu", pose_col.size());
    ROS_INFO("Total number of reachable poses: %lu", pose_col_filter.size());

    // The centers of reachable spheres are stored in a map. This data will be utilized in visualizing the spheres in
    // the visualizer.
    // TODO there are several maps are implemented. We can get rid of few maps and run large loops. The complexity of
    // accessing map is Olog(n)

    utility::MapVecDoublePtr sphere_color;

    for (utility::MultiMapPtr::iterator it = pose_col_filter.begin(); it != pose_col_filter.end(); ++it)
    {
      const std::vector<double>* sphere_coord    = it->first;
      //const std::vector<double>* point_on_sphere = it->second;

      // Reachability Index D=R/N*100;
      float d = float(pose_col_filter.count(sphere_coord)) / (pose_col.size() / new_data.size()) * 100;
      sphere_color.insert( std::make_pair(it->first, double(d)));
    }

    ROS_INFO("No of spheres reachable: %lu", sphere_color.size());

    ROS_INFO("MAP CREATION COMPLETED - NOW STORING IN .h5 FILE");
    // Creating maps now
//Saving map to dataset
    hdf5_dataset::Hdf5Dataset h5(filename);
    h5.saveReachData(pose_col_filter, sphere_color, resolution);

    double dif = ros::Duration( ros::Time::now() - startit).toSec();
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
