#include "location_manager/Location.h"
#include "location_manager/LocationArray.h"
#include "location_manager/addLocation.h"
#include "location_manager/addLocationWithPose.h"
#include "location_manager/deleteAllLocation.h"
#include "location_manager/deleteLocation.h"
#include "location_manager/queryAllLocation.h"
#include "location_manager/queryLocation.h"
#include "location_manager/queryLocationWithPose.h"
#include "ros/ros.h"
#include <cstdlib>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <sqlite3.h>
#include <sstream>
#include <string>

const bool DBG = true;
const std::string TableName = "LOCATION";
const char *DBName = "locationMgr.db";
const float UNKNOWN = -10000.0;

geometry_msgs::Pose curPose;

void currentPoseCallback(
    const geometry_msgs::PoseStamped::ConstPtr &currentPose) {
  curPose.position.x = currentPose->pose.position.x;
  curPose.position.y = currentPose->pose.position.y;
  curPose.position.z = currentPose->pose.position.z;
  curPose.orientation.x = currentPose->pose.orientation.x;
  curPose.orientation.y = currentPose->pose.orientation.y;
  curPose.orientation.z = currentPose->pose.orientation.z;
  curPose.orientation.w = currentPose->pose.orientation.w;
}

std::string convertToString(float fOri) {
  std::string sQuery;
  std::ostringstream ss;
  ss << fOri;
  return ss.str();
}

bool addLocation(location_manager::addLocation::Request &req,
                 location_manager::addLocation::Response &res) {
  const char *sql;
  sqlite3 *db;
  int iSize = 0;
  bool result = true;
  char *errMsg = NULL;
  std::string sQuery;
  sqlite3_open(DBName, &db);

  sQuery += "Replace INTO " + TableName + " values(";
  sQuery += "\"" + req.name + "\", ";
  sQuery += convertToString(curPose.position.x) + ", ";
  sQuery += convertToString(curPose.position.y) + ", ";
  sQuery += convertToString(curPose.position.z) + ", ";
  sQuery += convertToString(curPose.orientation.x) + ", ";
  sQuery += convertToString(curPose.orientation.y) + ", ";
  sQuery += convertToString(curPose.orientation.z) + ", ";
  sQuery += convertToString(curPose.orientation.w) + ")";
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== addLocation ==\n");
    fprintf(stderr, "sql=> %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK) {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    res.result = false;
  } else {
    res.result = true;
  }
  sqlite3_close(db);
  return true;
}

bool addLocationWithPose(location_manager::addLocationWithPose::Request &req,
                         location_manager::addLocationWithPose::Response &res) {
  const char *sql;
  sqlite3 *db;
  int iSize = 0;
  bool result = true;
  char *errMsg = NULL;
  std::string sQuery;
  sqlite3_open(DBName, &db);

  sQuery += "Replace INTO " + TableName + " values(";
  sQuery += "\"" + req.name + "\", ";
  sQuery += convertToString(req.pose.position.x) + ", ";
  sQuery += convertToString(req.pose.position.y) + ", ";
  sQuery += convertToString(req.pose.position.z) + ", ";
  sQuery += convertToString(req.pose.orientation.x) + ", ";
  sQuery += convertToString(req.pose.orientation.y) + ", ";
  sQuery += convertToString(req.pose.orientation.z) + ", ";
  sQuery += convertToString(req.pose.orientation.w) + ")";
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== addLocationWithPose ==\n");
    fprintf(stderr, "sql=> %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK) {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    res.result = false;
  } else {
    res.result = true;
  }
  return true;
}
bool deleteLocation(location_manager::deleteLocation::Request &req,
                    location_manager::deleteLocation::Response &res) {
  const char *sql;
  sqlite3 *db;
  char *errMsg = NULL;
  std::string sQuery;

  sqlite3_open(DBName, &db);
  sQuery += "Delete from " + TableName + " WHERE NAME = \"" + req.name + "\"";
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== deleteLocation ==\n");
    fprintf(stderr, "sql=> %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK) {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    res.result = false;
  } else {
    res.result = true;
  }
  sqlite3_close(db);
  return true;
}

bool deleteAllLocation(location_manager::deleteAllLocation::Request &req,
                       location_manager::deleteAllLocation::Response &res) {
  sqlite3 *db;
  char *errMsg = NULL;
  const char *sql;
  std::string sQuery;

  sqlite3_open(DBName, &db);
  sQuery += "DELETE FROM " + TableName;
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== Delete all ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  if (sqlite3_exec(db, sql, 0, 0, &errMsg) != SQLITE_OK) {
    fprintf(stderr, "\t> Error: %s\n", errMsg);
    res.result = false;
  }
  res.result = true;
  return true;
}

bool queryLocation(location_manager::queryLocation::Request &req,
                   location_manager::queryLocation::Response &res) {
  sqlite3 *db;
  const char *sql;
  char **result;
  char *errMsg = NULL;
  int iRows, iCols, iIndex;
  std::string sQuery;

  sqlite3_open(DBName, &db);
  sQuery += "SELECT * FROM " + TableName + " WHERE NAME = \"" + req.name + "\"";
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== queryLocation ==\n");
    fprintf(stderr, "sql=> %s\n\n", sql);
  }
  sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);
  if (iRows != 0 && iCols != 0) {
    iIndex = iCols;
    iIndex++;
    res.pose.position.x = atof(result[iIndex]);
    iIndex++;
    res.pose.position.y = atof(result[iIndex]);
    iIndex++;
    res.pose.position.z = atof(result[iIndex]);
    iIndex++;
    res.pose.orientation.x = atof(result[iIndex]);
    iIndex++;
    res.pose.orientation.y = atof(result[iIndex]);
    iIndex++;
    res.pose.orientation.z = atof(result[iIndex]);
    iIndex++;
    res.pose.orientation.w = atof(result[iIndex]);
    // res.pose = pose;
    res.result = true;
  } else {
    res.result = false;
  }
  sqlite3_free_table(result);
  sqlite3_close(db);
  return true;
}

bool queryAllLocation(location_manager::queryAllLocation::Request &req,
                      location_manager::queryAllLocation::Response &res) {
  sqlite3 *db;
  const char *sql;
  char **result;
  char *errMsg = NULL;
  int iRows, iCols, iIndex;
  std::string sQuery;
  geometry_msgs::Pose pose;
  location_manager::Location location;
  location_manager::LocationArray locationArray;

  sqlite3_open(DBName, &db);
  sQuery += "SELECT * FROM " + TableName;
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== queryAllLocation ==\n");
    fprintf(stderr, "sql %s\n\n", sql);
  }
  sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);

  if (iRows != 0 && iCols != 0) {
    iIndex = iCols;

    for (int iRow = 0; iRow < iRows; iRow++) {
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
    res.result = true;
    res.locationArray = locationArray;
  } else {
    location.name = "";
    locationArray.location.push_back(location);
    res.result = false;
  }
  sqlite3_free_table(result);
  sqlite3_close(db);
  return true;
}

bool queryLocationWithPose(location_manager::queryLocationWithPose::Request &req,
                           location_manager::queryLocationWithPose::Response &res) {
  sqlite3 *db;
  const char *sql;
  char **result;
  char *errMsg = NULL;
  int iRows, iCols, iIndex;
  std::string sQuery;

  sqlite3_open(DBName, &db);
  sQuery += "SELECT NAME FROM " + TableName + " WHERE ";
  sQuery += "POSE_X = " + convertToString(req.pose.position.x) + " AND ";
  sQuery += "POSE_Y = " + convertToString(req.pose.position.y) + " AND ";
  sQuery += "POSE_Z = " + convertToString(req.pose.position.z) + " AND ";
  sQuery += "ORIENTATION_X = " + convertToString(req.pose.orientation.x) + " AND ";
  sQuery += "ORIENTATION_Y = " + convertToString(req.pose.orientation.y) + " AND ";
  sQuery += "ORIENTATION_Z = " + convertToString(req.pose.orientation.z) + " AND ";
  sQuery += "ORIENTATION_W = " + convertToString(req.pose.orientation.w);
  sql = sQuery.c_str();
  if (DBG) {
    fprintf(stderr, "== queryLocationWithPose ==\n");
    fprintf(stderr, "sql=> %s\n\n", sql);
  }
  sqlite3_get_table(db, sql, &result, &iRows, &iCols, &errMsg);
  if (iRows != 0 && iCols != 0) {
      iIndex = iCols;
      res.name = result[iIndex];
      res.result = true;

  } else {
    res.result = false;
  }
  sqlite3_free_table(result);
  sqlite3_close(db);

  return true;
}
void createTable(sqlite3 *db) {
  const char *sql;
  char *errMsg = NULL;
  std::string create = "CREATE TABLE " + TableName +
                       " ("
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

void createDB() {
  sqlite3 *db;
  char *zErrMsg = 0;
  int rc;

  rc = sqlite3_open(DBName, &db);

  if (rc) {
    fprintf(stderr, "Can't open database: %s\n", sqlite3_errmsg(db));
  } else {
    fprintf(stderr, "Opened database successfully\n");
  }
  createTable(db);
  sqlite3_close(db);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "lcoationMgr_server");
  ros::NodeHandle n;

  ros::Subscriber sub =
      n.subscribe("/andbot/current_position", 10, currentPoseCallback);

  ros::ServiceServer addLocation_srv =
      n.advertiseService("addLocation", addLocation);
  ros::ServiceServer addLocationWithPose_srv =
      n.advertiseService("addLocationWithPose", addLocationWithPose);

  ros::ServiceServer deleteLocation_srv =
      n.advertiseService("deleteLocation", deleteLocation);
  ros::ServiceServer deleteAll_service =
      n.advertiseService("deleteAllLocation", deleteAllLocation);
  ros::ServiceServer queryLocation_srv =
      n.advertiseService("queryLocation", queryLocation);
  ros::ServiceServer queryAll_srv =
      n.advertiseService("queryAllLocation", queryAllLocation);
  ros::ServiceServer queryLocationWithPose_srv =
      n.advertiseService("queryLocationWithPose", queryLocationWithPose);
  createDB();
  ROS_INFO("Service ready\n");
  ros::spin();
  return 0;
}