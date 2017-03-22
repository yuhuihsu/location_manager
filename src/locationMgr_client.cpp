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

void addLocation() {
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::addLocation>("addLocation");
  location_manager::addLocation srv;

  srv.request.name = "livingroom";

  if (client.call(srv)) {
    ROS_INFO("addLocation finished");
    if (srv.response.result) {
      ROS_INFO("addLocation succeed");
    } else {
      ROS_INFO("addLocation failed");
    }
  } else {
    ROS_ERROR("Failed to call service addLocation");
  }
}

void addLocationWithPose() {
  /** Insert **/
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::addLocationWithPose>(
          "addLocationWithPose");
  location_manager::addLocationWithPose srv;

  location_manager::Location location;
  srv.request.pose.position.x = 3;
  srv.request.pose.position.y = 4;
  srv.request.name = "livingroom";

  if (client.call(srv)) {
    ROS_INFO("Inset finished");
    ROS_INFO("result : %d", srv.response.result);
  } else {
    ROS_ERROR("Failed to call service addLocationWithPose");
  }
}

void deleteLocation() {
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::deleteLocation>("deleteLocation");
  location_manager::deleteLocation srv;

  srv.request.name = "livingroom22";

  if (client.call(srv)) {
    ROS_INFO("deleteLocation finished");
    if (srv.response.result) {
      ROS_INFO("deleteLocation succeed");
    } else {
      ROS_INFO("deleteLocation failed");
    }
  } else {
    ROS_ERROR("Failed to call service deleteLocation");
  }
}

void deleteAllLocation() {
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::deleteAllLocation>("deleteAllLocation");
  location_manager::deleteAllLocation srv;

  if (client.call(srv)) {
    ROS_INFO("deleteLocatideleteAllLocationon finished");
    if (srv.response.result) {
      ROS_INFO("deleteAllLocation succeed");
    } else {
      ROS_INFO("deleteAllLocation failed");
    }
  } else {
    ROS_ERROR("Failed to call service deleteAllLocation");
  }
}
void queryLocation() {
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::queryLocation>("queryLocation");
  location_manager::queryLocation queryLocationWithPose_srv;

  queryLocationWithPose_srv.request.name = "kitchen";
  if (client.call(queryLocationWithPose_srv)) {
    ROS_INFO("queryLocation finished");
    if (queryLocationWithPose_srv.response.result) {
      ROS_INFO("pose position x %f",
               queryLocationWithPose_srv.response.pose.position.x);
      ROS_INFO("pose position y %f",
               queryLocationWithPose_srv.response.pose.position.y);
      ROS_INFO("pose position z %f",
               queryLocationWithPose_srv.response.pose.position.z);
      ROS_INFO("pose orientation x %f",
               queryLocationWithPose_srv.response.pose.orientation.x);
      ROS_INFO("pose orientation y %f",
               queryLocationWithPose_srv.response.pose.orientation.y);
      ROS_INFO("pose orientation z %f",
               queryLocationWithPose_srv.response.pose.orientation.z);
      ROS_INFO("pose orientation w %f",
               queryLocationWithPose_srv.response.pose.orientation.w);
    } else {
      ROS_INFO("No data");
    }
  } else {
    ROS_ERROR("Failed to call service queryLocation");
  }
}

void queryAllLocation() {
  ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::queryAllLocation>("queryAllLocation");
  location_manager::queryAllLocation queryAllLocation_srv;
  queryAllLocation_srv.request.execute = true;

  if (client.call(queryAllLocation_srv)) {
    ROS_INFO("Query finished");
    if (queryAllLocation_srv.response.result) {

      int iSize = queryAllLocation_srv.response.locationArray.location.size();
      ROS_INFO("iSize %d\n", iSize);
      for (int iNum = 0; iNum < iSize; iNum++) {
        ROS_INFO("name %s",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .name.c_str());
        ROS_INFO("\tpose position x %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.position.x);
        ROS_INFO("\tpose position y %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.position.y);
        ROS_INFO("\tpose position z %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.position.z);
        ROS_INFO("\tpose orientation x %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.orientation.x);
        ROS_INFO("\tpose orientation y %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.orientation.y);
        ROS_INFO("\tpose orientation z %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.orientation.z);
        ROS_INFO("\tpose posiorientationtion w %f",
                 queryAllLocation_srv.response.locationArray.location[iNum]
                     .pose.orientation.w);
      }
    }
  } else {
    ROS_ERROR("Failed to call service queryAllLocation");
  }
}
void tet(){

}

void queryLocationWithPose() {
    ros::NodeHandle n;
  ros::ServiceClient client =
      n.serviceClient<location_manager::queryLocationWithPose>(
          "queryLocationWithPose");
  location_manager::queryLocationWithPose queryLocationWithPose_srv;

  geometry_msgs::Pose pose;
  pose.position.x = 3;
  pose.position.y = 4;
  pose.position.z = 0;
  pose.orientation.x = 0;
  pose.orientation.y = 0;
  pose.orientation.z = 0;
  pose.orientation.w = 0;
  queryLocationWithPose_srv.request.pose = pose;

  if (client.call(queryLocationWithPose_srv)) {
    ROS_INFO("Query finished");
    if (queryLocationWithPose_srv.response.result) {
      ROS_INFO("Name : %s", queryLocationWithPose_srv.response.name.c_str());
    } else {
      ROS_INFO("No data");
    }
  } else {
    ROS_ERROR("Failed to call service queryLocationWithPose");
  }
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "locationMgr_client");
  if (argc != 2) {
    ROS_INFO("usage: locationMgr_client $mode \nMode selection\n");
    ROS_INFO("\n1.addLocation  \n2.addLocationWithPose  \n3.deleteLocation  "
             "\n4.deleteAllLocation \n5.queryLocation \n6.queryAllLocation "
             "\n7.queryLocationWithPose");
    return 1;
  }

  switch (atoll(argv[1])) {
  case 1:
    addLocation();
    break;
  case 2:
    addLocationWithPose();
    break;
  case 3:
    deleteLocation();
    break;
  case 4:
    deleteAllLocation();
    break;
  case 5:
    queryLocation();
    break;
  case 6:
    queryAllLocation();
    break;
  case 7:
    queryLocationWithPose();
    break;
  default:
    break;
  }

  return 0;
}