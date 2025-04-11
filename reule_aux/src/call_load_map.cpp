#include <ros/ros.h>
#include <moveit_msgs/LoadMap.h>

int main(int argc, char** argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "call_load_map");
    ros::NodeHandle nh;

    // Check if the filename argument is provided
    if (argc != 2) {
        ROS_ERROR("Usage: rosrun reule_aux load_map_node <octomap_filename>");
        return 1;
    }

    // Get the filename from the arguments and append ".bt"
    std::string filename = argv[1];
    filename += ".bt";
    std::string path = "/home/tjark/catkin_ws/src/Reuleaux_new/reule_aux/octomap_server_utils/maps/";
    filename = path + filename;
    // Create a service client for /move_group/load_map
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::LoadMap>("/move_group/load_map");
    moveit_msgs::LoadMap srv;
    srv.request.filename = filename;

    // Call the service
    if (client.call(srv)) {
        if (srv.response.success) {
            ROS_INFO("Successfully loaded map: %s", filename.c_str());
        } else {
            ROS_ERROR("Failed to load map: %s", filename.c_str());
        }
    } else {
        ROS_ERROR("Failed to call service /move_group/load_map");
    }

    return 0;
}