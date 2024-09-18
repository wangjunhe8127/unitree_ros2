#include <math.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/imu.h>
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

#include <chrono>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <iostream>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>

#include "builtin_interfaces/msg/time.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/synchronizer.h"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/time.hpp"
#include "rmw/qos_profiles.h"
#include "rmw/types.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/transform_datatypes.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_broadcaster.h"

using namespace std;

const double PI = 3.1415926;

string pathFolder;
double vehicleLength = 0.6;
double vehicleWidth = 0.6;
double sensorOffsetX = 0;
double sensorOffsetY = 0;
bool twoWayDrive = true;
double laserVoxelSize = 0.05;
double terrainVoxelSize = 0.2;
bool useTerrainAnalysis = true;
bool checkObstacle = true;
bool checkRotObstacle = false;
double adjacentRange = 3.5;
double obstacleHeightThre = 0.2;
double groundHeightThre = 0.1;
double costHeightThre = 0.1;
double costScore = 0.02;
bool useCost = false;
const int laserCloudStackNum = 1;
int laserCloudCount = 0;
int pointPerPathThre = 2;
double minRelZ = -0.5;
double maxRelZ = 0.5;
double maxSpeed = 1.0;
double dirWeight = 0.02;
double dirThre = 90.0;
bool dirToVehicle = false;
double pathScale = 1.0;
double minPathScale = 0.75;
double pathScaleStep = 0.25;
bool pathScaleBySpeed = true;
double minPathRange = 1.0;
double pathRangeStep = 0.5;
bool pathRangeBySpeed = true;
bool pathCropByGoal = true;

double goalCloseDis = 1.0;
double goalClearRange = 0.5;
double goalX = 0;
double goalY = 0;

// bool autonomyMode = false;
double autonomySpeed = 1.0;
double joyToSpeedDelay = 2.0;
double joyToCheckObstacleDelay = 5.0;
float joySpeed = 0;
float joySpeedRaw = 0;
float joyDir = 0;

const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.55;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;

pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudCrop(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr terrainCloudDwz(
    new pcl::PointCloud<pcl::PointXYZI>());

pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloud(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr plannerCloudCrop(
    new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];

int pathList[pathNum] = {0};
float endDirPathList[pathNum] = {0};
int clearPathList[36 * pathNum] = {0};
float pathPenaltyList[36 * pathNum] = {0};
float clearPathPerGroupScore[36 * groupNum] = {0};
std::vector<int> correspondences[gridVoxelNum];

bool newLaserCloud = false;
bool newTerrainCloud = false;

double odomTime = 0;
double joyTime = 0;

float vehicleRoll = 0, vehiclePitch = 0, vehicleYaw = 0;
float vehicleX = 0, vehicleY = 0, vehicleZ = 0;

pcl::VoxelGrid<pcl::PointXYZI> terrainDwzFilter;
rclcpp::Node::SharedPtr nh;

void odometryHandler(const nav_msgs::msg::Odometry::ConstSharedPtr odom) {
  odomTime = rclcpp::Time(odom->header.stamp).seconds();
  double roll, pitch, yaw;
  geometry_msgs::msg::Quaternion geoQuat = odom->pose.pose.orientation;
  tf2::Matrix3x3(tf2::Quaternion(geoQuat.x, geoQuat.y, geoQuat.z, geoQuat.w))
      .getRPY(roll, pitch, yaw);

  vehicleRoll = roll;
  vehiclePitch = pitch;
  vehicleYaw = yaw;
  vehicleX = odom->pose.pose.position.x - cos(yaw) * sensorOffsetX +
             sin(yaw) * sensorOffsetY;
  vehicleY = odom->pose.pose.position.y - sin(yaw) * sensorOffsetX -
             cos(yaw) * sensorOffsetY;
  std::cout << "plannr_x: " << vehicleX <<std::endl;
  std::cout << "yaw:" << yaw << std::endl;
  vehicleZ = odom->pose.pose.position.z;
  newTerrainCloud = true;
}

void terrainCloudHandler(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr terrainCloud2) {
  if (useTerrainAnalysis) {
    std::cout << "plan_cloud_size: " << terrainCloud->points.size() << std::endl;
    terrainCloud->clear();
    pcl::fromROSMsg(*terrainCloud2, *terrainCloud);

    pcl::PointXYZI point;
    terrainCloudCrop->clear();
    int terrainCloudSize = terrainCloud->points.size();
    for (int i = 0; i < terrainCloudSize; i++) {
      point = terrainCloud->points[i];

      float pointX = point.x;
      float pointY = point.y;
      float pointZ = point.z;

      float dis = sqrt((pointX - vehicleX) * (pointX - vehicleX) +
                       (pointY - vehicleY) * (pointY - vehicleY));
      if (dis < adjacentRange &&
          (point.intensity > obstacleHeightThre || useCost)) {
        point.x = pointX;
        point.y = pointY;
        point.z = pointZ;
        terrainCloudCrop->push_back(point);
      }
    }

    terrainCloudDwz->clear();
    terrainDwzFilter.setInputCloud(terrainCloudCrop);
    terrainDwzFilter.filter(*terrainCloudDwz);

    newTerrainCloud = true;
  }
}

void goalHandler(const geometry_msgs::msg::PointStamped::ConstSharedPtr goal) {
  std::cout << "waypoint: " << goal->point.x << " " << goal->point.y <<std::endl;
  goalX = goal->point.x;
  goalY = goal->point.y;
}

int readPlyHeader(FILE *filePtr) {
  char str[50];
  int val, pointNum;
  string strCur, strLast;
  while (strCur != "end_header") {
    val = fscanf(filePtr, "%s", str);
    if (val != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    strLast = strCur;
    strCur = string(str);

    if (strCur == "vertex" && strLast == "element") {
      val = fscanf(filePtr, "%d", &pointNum);
      if (val != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }
    }
  }

  return pointNum;
}

void readStartPaths() {
  string fileName = pathFolder + "/startPaths.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int pointNum = readPlyHeader(filePtr);

  pcl::PointXYZ point;
  int val1, val2, val3, val4, groupID;
  for (int i = 0; i < pointNum; i++) {
    val1 = fscanf(filePtr, "%f", &point.x);
    val2 = fscanf(filePtr, "%f", &point.y);
    val3 = fscanf(filePtr, "%f", &point.z);
    val4 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    if (groupID >= 0 && groupID < groupNum) {
      startPaths[groupID]->push_back(point);
    }
  }

  fclose(filePtr);
}

void readPathList() {
  string fileName = pathFolder + "/pathList.ply";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  if (pathNum != readPlyHeader(filePtr)) {
    RCLCPP_INFO(nh->get_logger(), "Incorrect path number, exit.");
    exit(1);
  }

  int val1, val2, val3, val4, val5, pathID, groupID;
  float endX, endY, endZ;
  for (int i = 0; i < pathNum; i++) {
    val1 = fscanf(filePtr, "%f", &endX);
    val2 = fscanf(filePtr, "%f", &endY);
    val3 = fscanf(filePtr, "%f", &endZ);
    val4 = fscanf(filePtr, "%d", &pathID);
    val5 = fscanf(filePtr, "%d", &groupID);

    if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    if (pathID >= 0 && pathID < pathNum && groupID >= 0 && groupID < groupNum) {
      pathList[pathID] = groupID;
      endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / PI;
    }
  }

  fclose(filePtr);
}

void readCorrespondences() {
  string fileName = pathFolder + "/correspondences.txt";

  FILE *filePtr = fopen(fileName.c_str(), "r");
  if (filePtr == NULL) {
    RCLCPP_INFO(nh->get_logger(), "Cannot read input files, exit.");
    exit(1);
  }

  int val1, gridVoxelID, pathID;
  for (int i = 0; i < gridVoxelNum; i++) {
    val1 = fscanf(filePtr, "%d", &gridVoxelID);
    if (val1 != 1) {
      RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
      exit(1);
    }

    while (1) {
      val1 = fscanf(filePtr, "%d", &pathID);
      if (val1 != 1) {
        RCLCPP_INFO(nh->get_logger(), "Error reading input files, exit.");
        exit(1);
      }

      if (pathID != -1) {
        if (gridVoxelID >= 0 && gridVoxelID < gridVoxelNum && pathID >= 0 &&
            pathID < pathNum) {
          correspondences[gridVoxelID].push_back(pathID);
        }
      } else {
        break;
      }
    }
  }

  fclose(filePtr);
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  nh = rclcpp::Node::make_shared("localPlanner");

  nh->declare_parameter<std::string>("pathFolder", pathFolder);
  nh->declare_parameter<double>("vehicleLength", vehicleLength);
  nh->declare_parameter<double>("vehicleWidth", vehicleWidth);
  nh->declare_parameter<double>("sensorOffsetX", sensorOffsetX);
  nh->declare_parameter<double>("sensorOffsetY", sensorOffsetY);
  nh->declare_parameter<bool>("twoWayDrive", twoWayDrive);
  nh->declare_parameter<double>("laserVoxelSize", laserVoxelSize);
  nh->declare_parameter<double>("terrainVoxelSize", terrainVoxelSize);
  nh->declare_parameter<bool>("useTerrainAnalysis", useTerrainAnalysis);
  nh->declare_parameter<bool>("checkObstacle", checkObstacle);
  nh->declare_parameter<bool>("checkRotObstacle", checkRotObstacle);
  nh->declare_parameter<double>("adjacentRange", adjacentRange);
  nh->declare_parameter<double>("obstacleHeightThre", obstacleHeightThre);
  nh->declare_parameter<double>("groundHeightThre", groundHeightThre);
  nh->declare_parameter<double>("costHeightThre", costHeightThre);
  nh->declare_parameter<double>("costScore", costScore);
  nh->declare_parameter<bool>("useCost", useCost);
  nh->declare_parameter<int>("pointPerPathThre", pointPerPathThre);
  nh->declare_parameter<double>("maxSpeed", maxSpeed);
  nh->declare_parameter<double>("dirWeight", dirWeight);
  nh->declare_parameter<double>("dirThre", dirThre);
  nh->declare_parameter<bool>("dirToVehicle", dirToVehicle);
  nh->declare_parameter<double>("pathScale", pathScale);
  nh->declare_parameter<double>("minPathScale", minPathScale);
  nh->declare_parameter<double>("pathScaleStep", pathScaleStep);
  nh->declare_parameter<bool>("pathScaleBySpeed", pathScaleBySpeed);
  nh->declare_parameter<double>("minPathRange", minPathRange);
  nh->declare_parameter<double>("pathRangeStep", pathRangeStep);
  nh->declare_parameter<bool>("pathRangeBySpeed", pathRangeBySpeed);
  nh->declare_parameter<bool>("pathCropByGoal", pathCropByGoal);
  // nh->declare_parameter<bool>("autonomyMode", autonomyMode);
  nh->declare_parameter<double>("autonomySpeed", autonomySpeed);
  nh->declare_parameter<double>("joyToSpeedDelay", joyToSpeedDelay);
  nh->declare_parameter<double>("joyToCheckObstacleDelay",
                                joyToCheckObstacleDelay);
  nh->declare_parameter<double>("goalClearRange", goalClearRange);
  nh->declare_parameter<double>("goalX", goalX);
  nh->declare_parameter<double>("goalY", goalY);

  nh->get_parameter("pathFolder", pathFolder);
  nh->get_parameter("vehicleLength", vehicleLength);
  nh->get_parameter("vehicleWidth", vehicleWidth);
  nh->get_parameter("sensorOffsetX", sensorOffsetX);
  nh->get_parameter("sensorOffsetY", sensorOffsetY);
  nh->get_parameter("twoWayDrive", twoWayDrive);
  nh->get_parameter("laserVoxelSize", laserVoxelSize);
  nh->get_parameter("terrainVoxelSize", terrainVoxelSize);
  nh->get_parameter("useTerrainAnalysis", useTerrainAnalysis);
  nh->get_parameter("checkObstacle", checkObstacle);
  nh->get_parameter("checkRotObstacle", checkRotObstacle);
  nh->get_parameter("adjacentRange", adjacentRange);
  nh->get_parameter("obstacleHeightThre", obstacleHeightThre);
  nh->get_parameter("groundHeightThre", groundHeightThre);
  nh->get_parameter("costHeightThre", costHeightThre);
  nh->get_parameter("costScore", costScore);
  nh->get_parameter("useCost", useCost);
  nh->get_parameter("pointPerPathThre", pointPerPathThre);
  nh->get_parameter("maxSpeed", maxSpeed);
  nh->get_parameter("dirWeight", dirWeight);
  nh->get_parameter("dirThre", dirThre);
  nh->get_parameter("dirToVehicle", dirToVehicle);
  nh->get_parameter("pathScale", pathScale);
  nh->get_parameter("minPathScale", minPathScale);
  nh->get_parameter("pathScaleStep", pathScaleStep);
  nh->get_parameter("pathScaleBySpeed", pathScaleBySpeed);
  nh->get_parameter("minPathRange", minPathRange);
  nh->get_parameter("pathRangeStep", pathRangeStep);
  nh->get_parameter("pathRangeBySpeed", pathRangeBySpeed);
  nh->get_parameter("pathCropByGoal", pathCropByGoal);
  // nh->get_parameter("autonomyMode", autonomyMode);
  nh->get_parameter("autonomySpeed", autonomySpeed);
  nh->get_parameter("joyToSpeedDelay", joyToSpeedDelay);
  nh->get_parameter("joyToCheckObstacleDelay", joyToCheckObstacleDelay);
  nh->get_parameter("goalCloseDis", goalCloseDis);
  nh->get_parameter("goalClearRange", goalClearRange);
  nh->get_parameter("goalX", goalX);
  nh->get_parameter("goalY", goalY);

  auto subOdometry = nh->create_subscription<nav_msgs::msg::Odometry>(
      "/state_estimation", 5, odometryHandler);

  auto subTerrainCloud = nh->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/terrain_map", 5, terrainCloudHandler);

  auto subGoal = nh->create_subscription<geometry_msgs::msg::PointStamped>(
      "/way_point", 5, goalHandler);

  auto pubPath = nh->create_publisher<nav_msgs::msg::Path>("/path", 5);
  nav_msgs::msg::Path path;

  joySpeed = autonomySpeed / maxSpeed;
  for (int i = 0; i < groupNum; i++) {
    startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
  }
  for (int i = 0; i < gridVoxelNum; i++) {
    correspondences[i].resize(0);
  }

  terrainDwzFilter.setLeafSize(terrainVoxelSize, terrainVoxelSize,
                               terrainVoxelSize);

  readStartPaths();
  readPathList();
  readCorrespondences();

  RCLCPP_INFO(nh->get_logger(), "Initialization complete.");

  rclcpp::Rate rate(10);
  bool status = rclcpp::ok();
  while (status) {
    rclcpp::spin_some(nh);

    // 更新障碍物云图
    if (newTerrainCloud) {
      newTerrainCloud = false;
      plannerCloud->clear();
      *plannerCloud = *terrainCloudDwz;
    }
    // 初始化roll, pitch, yaw
    float sinVehicleRoll = sin(vehicleRoll);
    float cosVehicleRoll = cos(vehicleRoll);
    float sinVehiclePitch = sin(vehiclePitch);
    float cosVehiclePitch = cos(vehiclePitch);
    float sinVehicleYaw = sin(vehicleYaw);
    float cosVehicleYaw = cos(vehicleYaw);

    // 障碍物高度图筛选
    pcl::PointXYZI point;
    plannerCloudCrop->clear();
    int plannerCloudSize = plannerCloud->points.size();
    for (int i = 0; i < plannerCloudSize; i++) {
      float pointX1 = plannerCloud->points[i].x - vehicleX;
      float pointY1 = plannerCloud->points[i].y - vehicleY;
      float pointZ1 = plannerCloud->points[i].z - vehicleZ;

      point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
      point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
      point.z = pointZ1;
      point.intensity = plannerCloud->points[i].intensity;

      float dis = sqrt(point.x * point.x + point.y * point.y);
      if (dis < adjacentRange && ((point.z > minRelZ && point.z < maxRelZ))) {
        plannerCloudCrop->push_back(point);
      }
    }

    // 目标点相对关系计算
    float pathRange = adjacentRange;
    if (pathRangeBySpeed) pathRange = adjacentRange * joySpeed;
    if (pathRange < minPathRange) pathRange = minPathRange;
    float relativeGoalDis = adjacentRange;
    float relativeGoalX = ((goalX - vehicleX) * cosVehicleYaw +
                           (goalY - vehicleY) * sinVehicleYaw);
    float relativeGoalY = (-(goalX - vehicleX) * sinVehicleYaw +
                           (goalY - vehicleY) * cosVehicleYaw);
    relativeGoalDis =
        sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
    joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / PI;
    std::cout << "goal target:" << joyDir << std::endl;
    if (!twoWayDrive) {
      if (joyDir > 90.0)
        joyDir = 90.0;
      else if (joyDir < -90.0)
        joyDir = -90.0;
    }

    // 搜索path
    bool pathFound = false;
    float defPathScale = pathScale;
    if (pathScaleBySpeed) pathScale = defPathScale * joySpeed;
    if (pathScale < minPathScale) pathScale = minPathScale;

    while (pathScale >= minPathScale && pathRange >= minPathRange) {
      for (int i = 0; i < 36 * pathNum; i++) {
        clearPathList[i] = 0;
        pathPenaltyList[i] = 0;
      }
      for (int i = 0; i < 36 * groupNum; i++) {
        clearPathPerGroupScore[i] = 0;
      }

      float minObsAngCW = -180.0;
      float minObsAngCCW = 180.0;
      float diameter = sqrt(vehicleLength / 2.0 * vehicleLength / 2.0 +
                            vehicleWidth / 2.0 * vehicleWidth / 2.0);
      float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / PI;
      int plannerCloudCropSize = plannerCloudCrop->points.size();
      for (int i = 0; i < plannerCloudCropSize; i++) {
        float x = plannerCloudCrop->points[i].x / pathScale;
        float y = plannerCloudCrop->points[i].y / pathScale;
        float h = plannerCloudCrop->points[i].intensity;
        float dis = sqrt(x * x + y * y);

        if (dis < pathRange / pathScale &&
            (dis <= (relativeGoalDis + goalClearRange) / pathScale ||
             !pathCropByGoal) &&
            checkObstacle) {
          for (int rotDir = 0; rotDir < 36; rotDir++) {
            float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            if ((angDiff > dirThre && !dirToVehicle) ||
                (fabs(10.0 * rotDir - 180.0) > dirThre &&
                 fabs(joyDir) <= 90.0 && dirToVehicle) ||
                ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) &&
                 fabs(joyDir) > 90.0 && dirToVehicle)) {
              continue;
            }

            float x2 = cos(rotAng) * x + sin(rotAng) * y;
            float y2 = -sin(rotAng) * x + cos(rotAng) * y;

            float scaleY = x2 / gridVoxelOffsetX +
                           searchRadius / gridVoxelOffsetY *
                               (gridVoxelOffsetX - x2) / gridVoxelOffsetX;

            int indX = int((gridVoxelOffsetX + gridVoxelSize / 2 - x2) /
                           gridVoxelSize);
            int indY =
                int((gridVoxelOffsetY + gridVoxelSize / 2 - y2 / scaleY) /
                    gridVoxelSize);
            if (indX >= 0 && indX < gridVoxelNumX && indY >= 0 &&
                indY < gridVoxelNumY) {
              int ind = gridVoxelNumY * indX + indY;
              int blockedPathByVoxelNum = correspondences[ind].size();
              for (int j = 0; j < blockedPathByVoxelNum; j++) {
                if (h > obstacleHeightThre || !useTerrainAnalysis) {
                  clearPathList[pathNum * rotDir + correspondences[ind][j]]++;
                } else {
                  if (pathPenaltyList[pathNum * rotDir +
                                      correspondences[ind][j]] < h &&
                      h > groundHeightThre) {
                    pathPenaltyList[pathNum * rotDir +
                                    correspondences[ind][j]] = h;
                  }
                }
              }
            }
          }
        }
        if (dis < diameter / pathScale &&
            (fabs(x) > vehicleLength / pathScale / 2.0 ||
             fabs(y) > vehicleWidth / pathScale / 2.0) &&
            (h > obstacleHeightThre || !useTerrainAnalysis) &&
            checkRotObstacle) {
          float angObs = atan2(y, x) * 180.0 / PI;
          if (angObs > 0) {
            if (minObsAngCCW > angObs - angOffset)
              minObsAngCCW = angObs - angOffset;
            if (minObsAngCW < angObs + angOffset - 180.0)
              minObsAngCW = angObs + angOffset - 180.0;
          } else {
            if (minObsAngCW < angObs + angOffset)
              minObsAngCW = angObs + angOffset;
            if (minObsAngCCW > 180.0 + angObs - angOffset)
              minObsAngCCW = 180.0 + angObs - angOffset;
          }
        }
      }

      if (minObsAngCW > 0) minObsAngCW = 0;
      if (minObsAngCCW < 0) minObsAngCCW = 0;

      for (int i = 0; i < 36 * pathNum; i++) {
        // 当前Path方向
        int rotDir = int(i / pathNum);
        // 与目标方向的角度差值
        float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
        // 归一化到正负pi
        if (angDiff > 180.0) {
          angDiff = 360.0 - angDiff;
        }
        // 抛弃轨迹逻辑
        // dirToVehicle:默认为false
        // 当前角度差大于90度时，抛弃该轨迹
        if ((angDiff > dirThre && !dirToVehicle) ||
            (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 &&
             dirToVehicle) ||
            ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) &&
             fabs(joyDir) > 90.0 && dirToVehicle)) {
          continue;
        }
        // 避障需求：默认clearPathList为0，pointPerPathThre为2
        if (clearPathList[i] < pointPerPathThre) {
          float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
          if (penaltyScore < costScore) penaltyScore = costScore;

          float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] -
                               (10.0 * rotDir - 180.0));
          // 角度差归一化
          if (dirDiff > 360.0) {
            dirDiff -= 360.0;
          }
          if (dirDiff > 180.0) {
            dirDiff = 360.0 - dirDiff;
          }
          float rotDirW;
          // 轨迹航向归一化到x轴
          if (rotDir < 18)
            rotDirW = fabs(fabs(rotDir - 9) + 1);
          else
            rotDirW = fabs(fabs(rotDir - 27) + 1);
          float groupDirW = 4 - fabs(pathList[i % pathNum] - 3);
          // float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW *
          //               rotDirW * rotDirW * rotDirW * penaltyScore;
          //距离比较近的情况下重新算score
          // if (relativeGoalDis < goalCloseDis)
          float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW *
                    groupDirW * penaltyScore;
          if (score > 0) {
            clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] +=
                score;
          }
        }
      }
      // 计算避障情况下选择哪组轨迹
      float maxScore = 0;
      int selectedGroupID = -1;
      for (int i = 0; i < 36 * groupNum; i++) {
        int rotDir = int(i / groupNum);
        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;
        float rotDeg = 10.0 * rotDir;
        if (rotDeg > 180.0) rotDeg -= 360.0;
        if (maxScore < clearPathPerGroupScore[i] &&
            ((rotAng * 180.0 / PI > minObsAngCW &&
              rotAng * 180.0 / PI < minObsAngCCW) ||
             (rotDeg > minObsAngCW && rotDeg < minObsAngCCW && twoWayDrive) ||
             !checkRotObstacle)) {
          maxScore = clearPathPerGroupScore[i];
          selectedGroupID = i;
        }
      }
      // 根据选择的组对轨迹进行赋值
      if (selectedGroupID >= 0) {
        int rotDir = int(selectedGroupID / groupNum);
        float rotAng = (10.0 * rotDir - 180.0) * PI / 180;

        selectedGroupID = selectedGroupID % groupNum;
        int selectedPathLength = startPaths[selectedGroupID]->points.size();
        path.poses.resize(selectedPathLength);
        for (int i = 0; i < selectedPathLength; i++) {
          float x = startPaths[selectedGroupID]->points[i].x;
          float y = startPaths[selectedGroupID]->points[i].y;
          float z = startPaths[selectedGroupID]->points[i].z;
          float dis = sqrt(x * x + y * y);

          if (dis <= pathRange / pathScale &&
              dis <= relativeGoalDis / pathScale) {
            path.poses[i].pose.position.x =
                pathScale * (cos(rotAng) * x - sin(rotAng) * y);
            path.poses[i].pose.position.y =
                pathScale * (sin(rotAng) * x + cos(rotAng) * y);
            path.poses[i].pose.position.z = pathScale * z;
          } else {
            path.poses.resize(i);
            break;
          }
        }

        path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
        path.header.frame_id = "map";
        pubPath->publish(path);
      }

      if (selectedGroupID < 0) {
        if (pathScale >= minPathScale + pathScaleStep) {
          pathScale -= pathScaleStep;
          pathRange = adjacentRange * pathScale / defPathScale;
        } else {
          pathRange -= pathRangeStep;
        }
      } else {
        pathFound = true;
        break;
      }
    }
    pathScale = defPathScale;
    // 搜索失败发送停止轨迹
    if (!pathFound) {
      path.poses.resize(1);
      path.poses[0].pose.position.x = 0;
      path.poses[0].pose.position.y = 0;
      path.poses[0].pose.position.z = 0;

      path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      path.header.frame_id = "map";
      pubPath->publish(path);
    }

    status = rclcpp::ok();
    rate.sleep();
  }

  return 0;
}