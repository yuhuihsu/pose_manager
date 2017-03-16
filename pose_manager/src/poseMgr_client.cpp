#include "ros/ros.h"
#include "pose_manager/add.h"
#include "pose_manager/LocationArray.h"
#include "pose_manager/Location.h"
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poseMgr_client");
  if (argc != 3)
  {
    ROS_INFO("usage: poseMgr_client X Y");
    return 1;
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::add>("add_location");
  pose_manager::add srv;

  pose_manager::LocationArray locationArray;
  pose_manager::Location location;
  location.pose.position.x = 3;
  location.pose.position.y = 4;
  location.name = "hihi";
  locationArray.location.push_back(location);
  
  pose_manager::Location location1;
  location1.pose.position.x = 3;
  location1.pose.position.y = 4;
  location1.name = "kerker";
  locationArray.location.push_back(location1);

  srv.request.location = locationArray;
  ROS_INFO("XDDDD");

  if (client.call(srv))
  {
    ROS_INFO("hihi finished");
  }
  else
  {
    ROS_INFO("XDDDD222");
    
    ROS_ERROR("Failed to call service add_location");
    return 1;
  }


  return 0;
}