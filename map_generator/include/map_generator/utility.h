#ifndef UTILITY_H
#define UTILITY_H

#include <boost/format.hpp>

#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include "map_generator/WorkSpace.h"
#include <octomap/octomap.h>

#include <iostream>

namespace utility
{

typedef std::vector<double> VecDouble;
typedef std::vector<VecDouble > VecVecDouble;
typedef std::multimap< VecDouble, VecDouble > MultiMap;
typedef std::multimap< const std::vector< double >*, const std::vector< double >* > MultiMapPtr;
typedef std::map<VecDouble, double> MapVecDouble;
typedef std::map< const std::vector< double >*, double > MapVecDoublePtr;

void pointToVector(const geometry_msgs::Point& point, std::vector<double>& data);
geometry_msgs::Point vectorToPoint(const std::vector<double>& data);

void octreePointToVector(const octomap::point3d point, std::vector< double >& data);
void vectorToOctreePoint(const std::vector< double > data, octomap::point3d &point);

void poseToVector(const geometry_msgs::Pose& pose, std::vector<double>& data);
void vectorToPose(const std::vector< double >& data, geometry_msgs::Pose &pose);

void getPoseAndSphereSize(const map_generator::WorkSpace& ws, int &sphere_size, int &pose_size);

std::string getRobotName(const std::string pkg_name);
std::string createName(const std::string& pkg_name, const std::string& group_name, double& res);


}//end namespace 




#endif // UTILITY_H
