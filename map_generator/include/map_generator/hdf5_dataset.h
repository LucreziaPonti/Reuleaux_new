#ifndef HDF5_DATASET_H
#define HDF5_DATASET_H
//#include "H5Cpp.h"
//#include <hdf5.h>
#include <hdf5/serial/hdf5.h>
#include <iostream>
#include <sys/stat.h>
#include <unistd.h>
#include <ros/ros.h>
#include <map_generator/WorkSpace.h>
#include <map_generator/utility.h>



namespace hdf5_dataset
{
struct stat st;

class Hdf5Dataset
{
public:
  Hdf5Dataset(std::string path, std::string filename);
  Hdf5Dataset(std::string fullpath);
  bool save(const map_generator::WorkSpace& ws);
  bool saveReachData( utility::MultiMapPtr& poses,  utility::MapVecDoublePtr& spheres, float resolution); //Saves Mutimap and Map to database and closes
  bool loadMapsFromDataset(utility::MultiMapPtr& poses, utility::MapVecDoublePtr& spheres); //Creates exact same Poses MultiMap that was stored with address variation
  bool loadMapsFromDataset(utility::MultiMapPtr& poses, utility::MapVecDoublePtr& spheres, float &resolution); //with resolution
  bool loadMapsFromDataset(utility::MultiMap& poses, utility::MapVecDouble& spheres); //Loads the pose and sphere
  bool loadMapsFromDataset(utility::MultiMap& Poses, utility::MapVecDouble& Spheres, float &resolution); //with resolution
  

  void close();
  bool open();
  bool getResolution(float& resolution);


private:
  bool checkPath(std::string path);
  bool checkfilename(std::string filename);
  void createPath(std::string path);
  bool saveMap(const utility::VecVecDouble& poses, const utility::VecVecDouble& spheres, const utility::VecDouble& ri, const double resolution);
  bool saveWorkspaceToMap(const map_generator::WorkSpace& ws);

  bool h5ToMultiMapPosesAndSpheres(utility::MultiMapPtr& pose_col, utility::MapVecDoublePtr& sphere_col); //loads the whole data with same address structure as stored in .h5
  bool h5ToMultiMapPoses(utility::MultiMap& pose_col, utility::MapVecDouble& sphere_col); //accesses the poses and spheres data in the poses dataset
  bool h5ToMultiMapPoses(utility::MultiMap& pose_col); //Accessess only the data from poses dataset regardless of address
  bool h5ToMultiMapSpheres(utility::MapVecDouble& sphere_col); //Accessess only the data from spheres dataset regardless of address



  std::string path_;
  std::string filename_;
  hid_t file_, group_poses_, group_spheres_;
  hid_t poses_dataset_, sphere_dataset_;
  hid_t attr_;
  float res_;
  map_generator::WorkSpace ws_;
  utility::MultiMap mMap_;
  utility::MapVecDouble mvec_;


};

}

#endif // HDF5_DATASET_H
