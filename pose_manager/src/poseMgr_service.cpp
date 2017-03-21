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
#include <iostream>
#include <sqlite3.h>
#include <sstream>
#include <string>

const bool DBG = true;
const std::string TableName = "LOCATION";
const char *DBName = "poseMgr.db";
const float UNKNOWN = -10000.0;

std::string convertToString(float fOri)
{
  std::string sResult;
  std::ostringstream ss;
  ss << fOri;
  return ss.str();
}

bool insertQuery(pose_manager::Location location, sqlite3 *db)
{
  char *errMsg = NULL;
  const char *sql;
  std::string sResult;

  sResult += "INSERT INTO " + TableName + " VALUES(";
  sResult += "\"" + location.name + "\", ";
  sResult += convertToString(location.pose.position.x) + ", ";
  sResult += convertToString(location.pose.position.y) + ", ";
  sResult += convertToString(location.pose.position.z) + ", ";
  sResult += convertToString(location.pose.orientation.x) + ", ";
  sResult += convertToString(location.pose.orientation.y) + ", ";
  sResult += convertToString(location.pose.orientation.z) + ", ";
  sResult += convertToString(location.pose.orientation.w) + ")";
  sql = sResult.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Insert ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK)
  {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    return false;
  }
  return true;
}

bool insert(pose_manager::insert::Request &req,
            pose_manager::insert::Response &res)
{
  sqlite3 *db;
  int iSize = 0;
  bool result = true;
  iSize = req.location.location.size();
  sqlite3_open(DBName, &db);
  for (int iNum = 0; iNum < iSize; iNum++)
  {
    result = insertQuery(req.location.location[iNum], db);
    result |= result;
  }
  sqlite3_close(db);
  res.result = result;

  return true;
}

bool updateQuery(pose_manager::Location location, sqlite3 *db)
{
  char *errMsg = NULL;
  const char *sql;
  std::string sQuery;
  char **result;
  int iRows, iCols, iIndex;

  sQuery += "SELECT * FROM " + TableName + " WHERE NAME = \"" + location.name + "\" AND ";
  sQuery += "POSE_X = " + convertToString(location.pose.position.x) + " AND ";
  sQuery += "POSE_Y = " + convertToString(location.pose.position.y) + " AND ";
  sQuery += "POSE_Z = " + convertToString(location.pose.position.z) + " AND ";
  sQuery += "ORIENTATION_X = " + convertToString(location.pose.orientation.x) + " AND ";
  sQuery += "ORIENTATION_Y = " + convertToString(location.pose.orientation.y) + " AND ";
  sQuery += "ORIENTATION_Z = " + convertToString(location.pose.orientation.z) + " AND ";
  sQuery += "ORIENTATION_W = " + convertToString(location.pose.orientation.w);
  sql = sQuery.c_str();
  fprintf(stderr, "sql %s\n", sql);
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK)
  {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    return false;
  }
  else
  {
    sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);
    if (iRows == iCols && iRows == 0 && iCols == 0)
    {
      fprintf(stderr, "\t> Error: no data\n");
      return false;
    }
  }
  sQuery = "";
  sQuery += "UPDATE " + TableName + " SET ";
  sQuery += "POSE_X = " + convertToString(location.pose.position.x) + ", ";
  sQuery += "POSE_Y = " + convertToString(location.pose.position.y) + ", ";
  sQuery += "POSE_Z = " + convertToString(location.pose.position.z) + ", ";
  sQuery += "ORIENTATION_X = " + convertToString(location.pose.orientation.x) + ", ";
  sQuery += "ORIENTATION_Y = " + convertToString(location.pose.orientation.y) + ", ";
  sQuery += "ORIENTATION_Z = " + convertToString(location.pose.orientation.z) + ", ";
  sQuery += "ORIENTATION_W = " + convertToString(location.pose.orientation.w) + " ";
  sQuery += "WHERE NAME = \"" + location.name + "\"";
  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Update ==\n");
    fprintf(stderr, "sql %s\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK)
  {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    return false;
  }
  return true;
}

bool update(pose_manager::update::Request &req,
            pose_manager::update::Response &res)
{
  sqlite3 *db;
  int iSize = 0;
  bool result = true;
  iSize = req.location.location.size();
  sqlite3_open(DBName, &db);
  for (int iNum = 0; iNum < iSize; iNum++)
  {
    result = updateQuery(req.location.location[iNum], db);
    result |= result;
  }
  sqlite3_close(db);
  res.result = result;

  return true;
}

bool deleteNameQuery(std::string name, sqlite3 *db)
{
  char *errMsg = NULL;
  const char *sql;
  std::string sQuery;

  sQuery += "DELETE FROM " + TableName + " WHERE NAME = \"" + name + "\"";
  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Delete Name ==\n");
    fprintf(stderr, "sql %s\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK)
  {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    return false;
  }
  return true;
}

bool deletebyName(pose_manager::deleteName::Request &req,
                  pose_manager::deleteName::Response &res)
{
  sqlite3 *db;
  bool result;

  sqlite3_open(DBName, &db);
  result = deleteNameQuery(req.name, db);
  sqlite3_close(db);
  if (result)
  {
    res.result = true;
  }
  else
  {
    res.result = false;
  }
  return true;
}

bool deletePoseQuery(geometry_msgs::Pose pose, sqlite3 *db)
{
  char *errMsg = NULL;
  const char *sql;
  std::string sQuery;

  sQuery += "DELETE FROM " + TableName + " WHERE ";
  sQuery += "POSE_X = " + convertToString(pose.position.x) + " AND ";
  sQuery += "POSE_Y = " + convertToString(pose.position.y) + " AND ";
  sQuery += "POSE_Z = " + convertToString(pose.position.z) + " AND ";
  sQuery += "ORIENTATION_X = " + convertToString(pose.orientation.x) + " AND ";
  sQuery += "ORIENTATION_Y = " + convertToString(pose.orientation.y) + " AND ";
  sQuery += "ORIENTATION_Z = " + convertToString(pose.orientation.z) + " AND ";
  sQuery += "ORIENTATION_W = " + convertToString(pose.orientation.w);
  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Delete Pose ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK)
  {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    return false;
  }
  return true;
}

bool deletebyPose(pose_manager::deletePose::Request &req,
                  pose_manager::deletePose::Response &res)
{
  sqlite3 *db;
  bool result;

  sqlite3_open(DBName, &db);
  result = deletePoseQuery(req.pose, db);
  sqlite3_close(db);
  if (result)
  {
    res.result = true;
  }
  else
  {
    res.result = false;
  }
  return true;
}

geometry_msgs::Pose queryPoseQuery(std::string sName, sqlite3 *db)
{
  char *errMsg = NULL;
  const char *sql;
  char **result;
  int iRows, iCols, iIndex;
  std::string sQuery;
  geometry_msgs::Pose pose;

  sQuery += "SELECT * FROM " + TableName + " WHERE NAME = \"" + sName + "\"";
  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Query Pose ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);

  if (iRows != 0 && iCols != 0)
  {
    iIndex = iCols;
    iIndex++;
    pose.position.x = atof(result[iIndex]);
    iIndex++;
    pose.position.y = atof(result[iIndex]);
    iIndex++;
    pose.position.z = atof(result[iIndex]);
    iIndex++;
    pose.orientation.x = atof(result[iIndex]);
    iIndex++;
    pose.orientation.y = atof(result[iIndex]);
    iIndex++;
    pose.orientation.z = atof(result[iIndex]);
    iIndex++;
    pose.orientation.w = atof(result[iIndex]);
  }
  else
  {
    pose.position.x = UNKNOWN;
  }
  sqlite3_free_table(result);
  return pose;
}

bool queryPose(pose_manager::queryPose::Request &req,
               pose_manager::queryPose::Response &res)
{
  sqlite3 *db;
  geometry_msgs::Pose pose;
  bool result;

  sqlite3_open(DBName, &db);
  pose = queryPoseQuery(req.name, db);
  sqlite3_close(db);
  if (pose.position.x != UNKNOWN)
  {
    res.pose = pose;
    res.result = true;
  }
  else
  {
    res.result = false;
  }
  return true;
}

std::string queryNameQuery(geometry_msgs::Pose pose, sqlite3 *db)
{
  char *errMsg = NULL;
  const char *sql;
  char **result;
  int iRows, iCols, iIndex;
  std::string sQuery;
  std::string sName;

  sQuery += "SELECT NAME FROM " + TableName + " WHERE ";
  sQuery += "POSE_X = " + convertToString(pose.position.x) + " AND ";
  sQuery += "POSE_Y = " + convertToString(pose.position.y) + " AND ";
  sQuery += "POSE_Z = " + convertToString(pose.position.z) + " AND ";
  sQuery += "ORIENTATION_X = " + convertToString(pose.orientation.x) + " AND ";
  sQuery += "ORIENTATION_Y = " + convertToString(pose.orientation.y) + " AND ";
  sQuery += "ORIENTATION_Z = " + convertToString(pose.orientation.z) + " AND ";
  sQuery += "ORIENTATION_W = " + convertToString(pose.orientation.w);

  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Query Name ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);
  if (iRows != 0 && iCols != 0)
  {
    iIndex = iCols;
    sName = result[iIndex];
  }
  else
  {
    return "";
  }
  return sName;
}

bool queryName(pose_manager::queryName::Request &req,
               pose_manager::queryName::Response &res)
{
  sqlite3 *db;
  std::string sName;

  sqlite3_open(DBName, &db);

  sName = queryNameQuery(req.pose, db);
  if (sName != "")
  {
    res.name = sName;
    res.result = true;
  }
  else
  {
    res.result = false;
  }
  sqlite3_close(db);
  return true;
}

pose_manager::LocationArray queryAllQuery(sqlite3 *db)
{
  pose_manager::LocationArray locationArray;
  char *errMsg = NULL;
  const char *sql;
  char **result;
  int iRows, iCols, iIndex;
  std::string sQuery;
  geometry_msgs::Pose pose;
  pose_manager::Location location;

  sQuery += "SELECT * FROM " + TableName;
  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Query all ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);

  if (iRows != 0 && iCols != 0)
  {
    iIndex = iCols;

    for (int iRow = 0; iRow < iRows; iRow++)
    {
      location.name = result[iIndex];
      iIndex++;
      pose.position.x = atof(result[iIndex]);
      iIndex++;
      pose.position.y = atof(result[iIndex]);
      iIndex++;
      pose.position.z = atof(result[iIndex]);
      iIndex++;
      pose.orientation.x = atof(result[iIndex]);
      iIndex++;
      pose.orientation.y = atof(result[iIndex]);
      iIndex++;
      pose.orientation.z = atof(result[iIndex]);
      iIndex++;
      pose.orientation.w = atof(result[iIndex]);
      iIndex++;
      location.pose = pose;
      locationArray.location.push_back(location);
    }
  }
  else
  {
    location.name = "";
    locationArray.location.push_back(location);
  }
  sqlite3_free_table(result);
  return locationArray;
}

bool queryAll(pose_manager::queryAll::Request &req,
              pose_manager::queryAll::Response &res)
{
  sqlite3 *db;
  pose_manager::LocationArray locationArray;

  sqlite3_open(DBName, &db);
  locationArray = queryAllQuery(db);
  if (locationArray.location[0].name != "")
  {
    res.locationArray = locationArray;
    res.result = true;
  }
  else
  {
    res.result = false;
  }
  sqlite3_close(db);
  return true;
}

bool deleteAll(pose_manager::deleteAll::Request &req,
               pose_manager::deleteAll::Response &res)
{
  sqlite3 *db;
  char *errMsg = NULL;
  const char *sql;
  std::string sQuery;

  pose_manager::LocationArray locationArray;
  sqlite3_open(DBName, &db);

  sQuery += "DELETE FROM " + TableName;
  sql = sQuery.c_str();
  if (DBG)
  {
    fprintf(stderr, "== Delete all ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK)
  {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    res.result = false;
  }
  res.result = true;
  return true;
}

void createTable(sqlite3 *db)
{
  const char *sql;
  char *errMsg = NULL;
  std::string create = "CREATE TABLE " + TableName + " ("
                                                     "NAME TEXT PRIMARY KEY NOT NULL,"
                                                     "POSE_X FLOAT NOT NULL,"
                                                     "POSE_Y FLOAT NOT NULL,"
                                                     "POSE_Z FLOAT NOT NULL,"
                                                     "ORIENTATION_X FLOAT NOT NULL,"
                                                     "ORIENTATION_Y FLOAT NOT NULL,"
                                                     "ORIENTATION_Z FLOAT NOT NULL,"
                                                     "ORIENTATION_W FLOAT NOT NULL)";

  sql = create.c_str();
  sqlite3_exec(db, sql, 0, 0, &errMsg);
}

void createDB()
{
  sqlite3 *db;
  char *zErrMsg = 0;
  int rc;

  rc = sqlite3_open(DBName, &db);

  if (rc)
  {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
  }
  else
  {
    fprintf(stderr, "Opened database successfully\n");
  }
  createTable(db);
  sqlite3_close(db);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "poseMgr_server");
  ros::NodeHandle n;

  ros::ServiceServer insert_service = n.advertiseService("insert_location", insert);
  ros::ServiceServer update_service = n.advertiseService("update_location", update);
  ros::ServiceServer deleteName_service = n.advertiseService("delete_location_by_name", deletebyName);
  ros::ServiceServer deletePose_service = n.advertiseService("delete_location_by_pose", deletebyPose);
  ros::ServiceServer deleteAll_service = n.advertiseService("delete_All", deleteAll);
  ros::ServiceServer queryPose_service = n.advertiseService("query_Pose", queryPose);
  ros::ServiceServer queryName_service = n.advertiseService("query_Name", queryName);
  ros::ServiceServer queryAll_service = n.advertiseService("query_All", queryAll);

  createDB();
  ROS_INFO("Service ready\n");
  ros::spin();
  return 0;
}