#include <ros/ros.h>
#include "map_generator/WorkSpace.h"
#include <map_generator/hdf5_dataset.h>

int main(int argc, char **argv)
{
  if (argc < 3)
  {
    ROS_ERROR_STREAM("USAGE: load_reachability_map <hdf5_file_path> <reference_frame_id> \n"
                     "reference_frame_id: RM -> manipulator root frame / IRM -> end effector frame");
    return 1;
  }else{
    ROS_WARN_STREAM("arguments entered: " << argv[1] << "\n " << argv[2] << "\n"); 
    ros::init(argc, argv, "load_reachability_map");
    ros::NodeHandle n;

    
    // TODO: It can be published as a latched topic. So the whole message will be published just once and stay on the
    // topic
    ros::Publisher workspace_pub = n.advertise< map_generator::WorkSpace >("reachability_map", 1);
    // bool latchOn = 1;
    // ros::Publisher workspace_pub = n.advertise<map_generator::WorkSpace>("reachability_map", 1, latchOn);
    ros::Rate loop_rate(10);

    int count = 0;

    hdf5_dataset::Hdf5Dataset h5(argv[1]);
    h5.open();

    utility::MultiMapPtr pose_col_filter;
    utility::MapVecDoublePtr sphere_col;
    float res;
    h5.loadMapsFromDataset(pose_col_filter, sphere_col, res);

    // Creating messages
    map_generator::WorkSpace ws;
    ws.header.stamp = ros::Time::now();
    ws.header.frame_id = argv[2];
    ws.resolution = res;
    for (utility::MapVecDoublePtr::iterator it = sphere_col.begin(); it != sphere_col.end(); ++it)
    {
       map_generator::WsSphere wss;
       wss.point.x = (*it->first)[0];
       wss.point.y = (*it->first)[1];
       wss.point.z = (*it->first)[2];
       wss.ri = it->second;
       for (utility::MultiMapPtr::iterator it1 = pose_col_filter.lower_bound(it->first); it1 != pose_col_filter.upper_bound(it->first); ++it1)
       {
          geometry_msgs::Pose pp;
          pp.position.x = it1->second->at(0);
          pp.position.y = it1->second->at(1);
          pp.position.z = it1->second->at(2);
          pp.orientation.x = it1->second->at(3);
          pp.orientation.y = it1->second->at(4);
          pp.orientation.z = it1->second->at(5);
          pp.orientation.w = it1->second->at(6);
          wss.poses.push_back(pp);
        }
        ws.WsSpheres.push_back(wss);
      }

    while (ros::ok())
    {
      workspace_pub.publish(ws);

      ros::spinOnce();
      sleep(5);
      ++count;
    }
  }
  return 0;
}
