#include "pose_manager/Location.h"
#include "pose_manager/LocationArray.h"
#include "pose_manager/insert.h"
#include "pose_manager/deleteName.h"
#include "pose_manager/deletePose.h"
#include "pose_manager/deleteAll.h"
#include "pose_manager/queryName.h"
#include "pose_manager/queryPose.h"
#include "pose_manager/queryAll.h"
#include "pose_manager/update.h"
#include "ros/ros.h"
#include <cstdlib>

void insert()
{
  /** Insert **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::insert>("insert_location");
  pose_manager::insert srv;

  pose_manager::LocationArray locationArray;
  pose_manager::Location location;
  location.pose.position.x = 3;
  location.pose.position.y = 4;
  location.name = "livingroom";
  locationArray.location.push_back(location);

  pose_manager::Location location1;
  location1.pose.position.x = 6;
  location1.pose.position.y = 9;
  location1.name = "kitchen";
  locationArray.location.push_back(location1);

  srv.request.location = locationArray;

  if (client.call(srv))
  {
    ROS_INFO("Inset finished");
    ROS_INFO("result : %d", srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service insert_location");
  }
}

void deleteName()
{
  /** Delete by Name **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::deleteName>("delete_location_by_name");
  pose_manager::deleteName deleteName_srv;
  deleteName_srv.request.name = "livingroom";
  if (client.call(deleteName_srv))
  {
    ROS_INFO("Delete name finished");
    ROS_INFO("result : %d", deleteName_srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service delete_location_by_name");
  }
}

void deletePose()
{
  /** Delete by Pose **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::deletePose>("delete_location_by_pose");
  pose_manager::deletePose deletePose_srv;
  geometry_msgs::Pose pose;
  pose.position.x = 3;
  pose.position.y = 4;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;

  deletePose_srv.request.pose = pose;

  if (client.call(deletePose_srv))
  {
    ROS_INFO("Delete Pose finished");
    ROS_INFO("result : %d", deletePose_srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service delete_location_by_name");
  }
}

void deleteAll()
{
  /** Delete all **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::deleteAll>("delete_All");
  pose_manager::deleteAll deleteAll_srv;
  // deleteAll_srv.request.execute = true;

  if (client.call(deleteAll_srv))
  {
    ROS_INFO("Delete finished");
    if (deleteAll_srv.response.result)
    {
      ROS_INFO("Delete succeed");
    }
    else
    {
      ROS_INFO("Delete failed");
    }
  }
  else
  {
    ROS_ERROR("Failed to call service delete_All");
  }
}
void queryName()
{
  /** Query Name **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::queryName>("query_Name");
  pose_manager::queryName queryName_srv;

  geometry_msgs::Pose pose;
  pose.position.x = 3;
  pose.position.y = 4;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  queryName_srv.request.pose = pose;

  if (client.call(queryName_srv))
  {
    ROS_INFO("Query finished");
    if (queryName_srv.response.result)
    {
      ROS_INFO("Name : %s", queryName_srv.response.name.c_str());
    }
  }
  else
  {
    ROS_ERROR("Failed to call service query_Pose");
  }
}

void queryPose()
{
  /** Query Pose **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::queryPose>("query_Pose");
  pose_manager::queryPose queryPose_srv;

  queryPose_srv.request.name = "kitchen";
  if (client.call(queryPose_srv))
  {
    ROS_INFO("Query finished");
    if (queryPose_srv.response.result)
    {
      ROS_INFO("pose position x %f", queryPose_srv.response.pose.position.x);
      ROS_INFO("pose position y %f", queryPose_srv.response.pose.position.y);
      ROS_INFO("pose position z %f", queryPose_srv.response.pose.position.z);
      ROS_INFO("pose orientation x %f", queryPose_srv.response.pose.orientation.x);
      ROS_INFO("pose orientation y %f", queryPose_srv.response.pose.orientation.y);
      ROS_INFO("pose orientation z %f", queryPose_srv.response.pose.orientation.z);
      ROS_INFO("pose posiorientationtion w %f", queryPose_srv.response.pose.orientation.w);
    }
  }
  else
  {
    ROS_ERROR("Failed to call service query_Pose");
  }
}

void queryAll()
{
  /** Query all **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::queryAll>("query_All");
  pose_manager::queryAll queryAll_srv;

  queryAll_srv.request.query = true;
  ROS_INFO("XDDDDDDDDDDD111");

  if (client.call(queryAll_srv))
  {
    ROS_INFO("Query finished");
    // if (queryAll_srv.response.result){
    //   ROS_INFO("XDDDDDDDDDDD");
    // }
    if (queryAll_srv.response.result)
    {

      int iSize = queryAll_srv.response.locationArray.location.size();
      ROS_INFO("iSize %d\n", iSize);
      for (int iNum = 0; iNum < iSize; iNum++)
      {
        ROS_INFO("name %s", queryAll_srv.response.locationArray.location[iNum].name.c_str());
        ROS_INFO("pose position x %f", queryAll_srv.response.locationArray.location[iNum].pose.position.x);
        ROS_INFO("pose position y %f", queryAll_srv.response.locationArray.location[iNum].pose.position.y);
        ROS_INFO("pose position z %f", queryAll_srv.response.locationArray.location[iNum].pose.position.z);
        ROS_INFO("pose orientation x %f", queryAll_srv.response.locationArray.location[iNum].pose.orientation.x);
        ROS_INFO("pose orientation y %f", queryAll_srv.response.locationArray.location[iNum].pose.orientation.y);
        ROS_INFO("pose orientation z %f", queryAll_srv.response.locationArray.location[iNum].pose.orientation.z);
        ROS_INFO("pose posiorientationtion w %f", queryAll_srv.response.locationArray.location[iNum].pose.orientation.w);
      }
    }
  }
  else
  {
    ROS_ERROR("Failed to call service query_Pose");
  }
}

void update()
{
  /** Update **/
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<pose_manager::update>("update_location");
  pose_manager::update update_srv;
  pose_manager::LocationArray locationArray;
  pose_manager::Location location;

  location.pose.position.x = 36;
  location.pose.position.y = 46;
  location.pose.position.z = 50;
  location.name = "livingroom";
  locationArray.location.push_back(location);
  update_srv.request.location = locationArray;
  if (client.call(update_srv))
  {
    ROS_INFO("Update finished");
    ROS_INFO("result : %d", update_srv.response.result);
  }
  else
  {
    ROS_ERROR("Failed to call service update_location");
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poseMgr_client");
  if (argc != 2)
  {
    ROS_INFO("usage: poseMgr_client $mode \nMode selection\n");
    ROS_INFO("1.insert  \n2.deleteName  \n3.deletePose  \n4.deleteAll");
    ROS_INFO("5.queryName  \n6.queryPose  \n7.queryAll \n8.update");
    return 1;
  }

  switch (atoll(argv[1]))
  {
  case 1:
    insert();
    break;
  case 2:
    deleteName();
    break;
  case 3:
    deletePose();
    break;
  case 4:
    deleteAll();
    break;
  case 5:
    queryName();
    break;
  case 6:
    queryPose();
    break;
  case 7:
    queryAll();
    break;
  case 8:
    update();
    break;
  }

  return 0;
}