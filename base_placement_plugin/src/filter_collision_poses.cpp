/*
    function check_collision_objects:
        receives a pose
        creates a client for the get_planning_scene service that returns the enitre planning scene - saves all the collision objects infos       
        for every collison object checks if the pose received is within its "bounds" 
        if it is = COLLISION = return false = not acceptable (stops checking for the other objects - onee is enough)
        if it's not = NO COLLISION - keep checking for the next object
        if no collision = no return false = exits the for loop = NO COLLISIONS = return true = acceptable pose
        !! extra check: if checkZ=true = arm pose -- check that pose is compatible with robot's constraints 
            (for tiago the arm is mounted on the torso which has some height contstraints)

    IDEA : function check_collision_octomap    
        circa same logic but using the octomap 
    or some other map used for collision checking in navigation
*/

#include <base_placement_plugin/filter_collision_poses.h>

#include <moveit/planning_scene/planning_scene.h>


FilterCollisionPoses::FilterCollisionPoses(){};

bool FilterCollisionPoses::check_collision_objects( ros::NodeHandle& node, const float x, const float y, const float z, bool checkZ){
    ROS_DEBUG("INIZIO CHECK COLLISION OBJECTS for pose: %f, %f, %f", x,y,z);

    planning_scene_client = node.serviceClient<moveit_msgs::GetPlanningScene>("/get_planning_scene");
    moveit_msgs::GetPlanningScene srv;
    //srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;
    if (!planning_scene_client.call(srv))
    {
        ROS_ERROR("Can't obtain planning scene - assuming all poses valid");
        return true;
    }
    ROS_DEBUG("PLANNING SCENE OTTENUTA");
    std::vector<moveit_msgs::CollisionObject> objects = srv.response.scene.world.collision_objects;
    if (objects.size()==0){
        ROS_WARN("No collision objects in the scene - assuming all poses valid");
        return true;
    }
    for(int i=0; i<objects.size();i++){
        ROS_DEBUG("object %d - name %s", i, objects[i].id.c_str());
        if(objects[i].id!="sink_furniture"){ // lo escludo perchè ci sono dei problemi di posizionamento che lo mettono dove non è irl
            //ROS_DEBUG("pos = %f , %f , %f ", objects[i].pose.position.x,objects[i].pose.position.y,objects[i].pose.position.z);
            //store the position values (shorter names for variables=less messy code)
            co_x=objects[i].pose.position.x;
            co_y=objects[i].pose.position.y;
            co_z=objects[i].pose.position.z;
            //ROS_DEBUG("primitive type: %d (box=1,cylinder=3)", objects[i].primitives[0].type);

            //store the values of dimensions depending on the primitive type - here we only used box and cylinder
            if(objects[i].primitives[0].type==1){ //BOX
                //ROS_DEBUG("dimensions: %f , %f , %f", objects[i].primitives[0].dimensions[0],objects[i].primitives[0].dimensions[1],objects[i].primitives[0].dimensions[2]); 
                dX=objects[i].primitives[0].dimensions[0]/2+0.25;
                dY=objects[i].primitives[0].dimensions[1]/2+0.25;
                dZ=objects[i].primitives[0].dimensions[2]/2+0.15;
                // the +0.15 is mainly for the tables 

            }else if (objects[i].primitives[0].type==3){ //CYLINDER
                //ROS_DEBUG("dimensions: %f , %f ", objects[i].primitives[0].dimensions[0],objects[i].primitives[0].dimensions[1]); 
                dX=objects[i].primitives[0].dimensions[1];
                dY=objects[i].primitives[0].dimensions[1];
                dZ=objects[i].primitives[0].dimensions[0]/2;
            }


            //PERFORM THE FILTERING
            if(checkZ){
                if((x>(co_x-(dX+0.08)))&&(x<(co_x+(dX+0.08)))&&(y>(co_y-(dY+0.065)))&&(y<(co_y+(dY+0.065)))){//inside the bounds of the object -- not acceptable pose
                    ROS_DEBUG("NOT acceptable pose - %s ",objects[i].id.c_str());
                    return false;
                }
                if((z>1.232||z<0.89)){ //outside robot's fisical boundaries (torso_lift_link's height)
                    ROS_DEBUG("NOT acceptable pose - robot physical boundaries");
                    return false;            
                }
            }else if((x>(co_x-dX))&&(x<(co_x+dX))&&(y>(co_y-dY))&&(y<(co_y+dY))){//inside the bounds of the object -- not acceptable pose
                ROS_DEBUG("NOT acceptable pose - %s ",objects[i].id.c_str());
                return false;
            }
        }
        // if not in collision goes on to check next object
    }
    //not in collision with any object -- acceptable pose
    ROS_DEBUG("acceptable pose");
    return true;
}
