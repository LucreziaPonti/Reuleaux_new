#include <ros/ros.h>
#include "map_creator/WorkSpace.h"
#include <map_creator/hdf5_dataset.h>

int main(int argc, char **argv)
{
  if (argc < 2)
  {
    ROS_ERROR_STREAM("Please provide the name of the reachability map. If you have not created it yet, Please create "
                     "the map by running the create reachability map launch file in map_creator");
    return 1;
  }
  else
  {
    ros::init(argc, argv, "workspace");
    ros::NodeHandle n;

    // TODO: It can be published as a latched topic. So the whole message will be published just once and stay on the
    // topic
    ros::Publisher workspace_pub = n.advertise< map_creator::WorkSpace >("reachability_map", 1);
    // bool latchOn = 1;
    // ros::Publisher workspace_pub = n.advertise<map_creator::WorkSpace>("reachability_map", 1, latchOn);
    ros::Rate loop_rate(10);

    int count = 0;

    hdf5_dataset::Hdf5Dataset h5(argv[1]);
    h5.open();

    MultiMapPtr pose_col_filter;
    MapVecDoublePtr sphere_col;
    float res;
    h5.loadMapsFromDataset(pose_col_filter, sphere_col, res);

    // Creating messages
    map_creator::WorkSpace ws;
    ws.header.stamp = ros::Time::now();
    if (argc < 3){
    	ROS_INFO_STREAM("NO ADDITIONAL ARG - THE MAP IS ASSUMED TO BE AN INVERSE RM - REFERENCE FRAME ID SET TO arm_tool_link");
    	ws.header.frame_id = "arm_tool_link";
    }else{
      if (argv[2][0]=='c'||argv[2][0]=='C'){
    	  ROS_INFO_STREAM("ADDITIONAL ARG USED = C/c- THE MAP IS ASSUMED TO BE A RM FROM MAP_CREATOR - REFERENCE FRAME ID SET TO torso_lift_link");
    	  ws.header.frame_id = "torso_lift_link";
      }else if (argv[2][0]=='g'||argv[2][0]=='G'){
    	  ROS_INFO_STREAM("ADDITIONAL ARG USED = G/g- THE MAP IS ASSUMED TO BE A RM FROM MAP_GENERATION not centered - REFERENCE FRAME ID SET TO base_footprint");
    	  ws.header.frame_id = "base_footprint";
      }else{
    	  ROS_INFO_STREAM("ADDITIONAL ARG USED is not acceptable option (c=MAP_CREATOR / g=MAP_GENERATION) - REFERENCE FRAME ID SET TO base_link");
    	  ws.header.frame_id = "base_link";
      }

    }
    ws.resolution = res;

    for (MapVecDoublePtr::iterator it = sphere_col.begin(); it != sphere_col.end(); ++it)
    {
       map_creator::WsSphere wss;
       wss.point.x = (*it->first)[0];
       wss.point.y = (*it->first)[1];
       wss.point.z = (*it->first)[2];
       wss.ri = it->second;

       for (MultiMapPtr::iterator it1 = pose_col_filter.lower_bound(it->first); it1 != pose_col_filter.upper_bound(it->first); ++it1)
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
