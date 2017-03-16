#include "ros/ros.h"
#include "pose_manager/LocationArray.h"
#include "pose_manager/Location.h"
#include "pose_manager/add.h"
#include <stdio.h>
#include <fstream>
#include <iterator>
#include <string>
#include <cstdlib>
#include <sstream>
#include <iostream>
#include <fstream>
#include <sqlite3.h>

bool writeFile(pose_manager::LocationArray locationArray)
{
  std::fstream fout;
  fout.open("test.txt", std::ios::out | std::ios::app);
  if (fout.fail())
  {
    ROS_INFO("open file failed");
    return false;
  }

  for (int iNum = 0; iNum < locationArray.location.size(); iNum++)
  {
    std::string result;
    std::ostringstream ss;

    result += locationArray.location[iNum].name;

    ss << locationArray.location[iNum].pose.position.x;
    ss << "\t";
    ss << locationArray.location[iNum].pose.position.y;
    ss << "\t";
    ss << locationArray.location[iNum].pose.position.z;
    ss << "\t";
    ss << locationArray.location[iNum].pose.orientation.x;
    ss << "\t";
    ss << locationArray.location[iNum].pose.orientation.y;
    ss << "\t";
    ss << locationArray.location[iNum].pose.orientation.z;
    ss << "\t";
    ss << locationArray.location[iNum].pose.orientation.w;
    ss << "\t";
    result += "\t" + (ss.str());
    fout << result << std::endl;
  }
  fout.close();
  return true;
}

bool add(pose_manager::add::Request &req,
         pose_manager::add::Response &res)
{
  ROS_INFO("Receive request");
  int iSize = 0;
  iSize = req.location.location.size();
  ROS_INFO("size: %d", iSize);
  res.result = true;
  writeFile(req.location);

  return true;
}

bool createTable(sqlite3 *db)
{
  char *errMsg = NULL;
  char *sql = "CREATE TABLE LOCATION(" \ 
              "ID INT PRIMARY KEY NOT NULL," \
              "NAME TEXT NOT NULL," \
              "POSE_X FLOAT NOT NULL" \
              "POSE_Y FLOAT NOT NULL" \
              "POSE_Z FLOAT NOT NULL" \
              "ORIENTATION_X FLOAT NOT NULL" \
              "ORIENTATION_Y FLOAT NOT NULL" \
              "ORIENTATION_Z FLOAT NOT NULL" \
              "ORIENTATION_W FLOAT NOT NULL";
  sqlite3_exec(db, sql, 0, 0, &errMsg);

}

bool createDB()
{
  sqlite3 *db;
  char *zErrMsg = 0;
  int rc;

  rc = sqlite3_open("test.db", &db);

  if (rc)
  {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
    return (0);
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

  ros::ServiceServer service = n.advertiseService("add_location", add);
  ROS_INFO("Ready to add two ints.");
  createDB();
  ros::spin();
  return 0;
}
