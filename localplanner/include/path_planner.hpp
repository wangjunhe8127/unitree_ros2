#include "common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <nav_msgs/msg/path.hpp>
#include <pcl/filters/voxel_grid.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <string>
#include <vector>
using namespace std;
const int pathNum = 343;
const int groupNum = 7;
float gridVoxelSize = 0.02;
float searchRadius = 0.55;
float gridVoxelOffsetX = 3.2;
float gridVoxelOffsetY = 4.5;
const int gridVoxelNumX = 161;
const int gridVoxelNumY = 451;
const int gridVoxelNum = gridVoxelNumX * gridVoxelNumY;
const int laserCloudStackNum = 1;
pcl::PointCloud<pcl::PointXYZI>::Ptr
    plannerCloud(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr
    plannerCloudCrop(new pcl::PointCloud<pcl::PointXYZI>());
int pathList[pathNum] = {0};
class PathPlanCore {
public:
  PathPlanCore() {}
  void init() {
    for (int i = 0; i < laserCloudStackNum; i++) {
      laserCloudStack[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    for (int i = 0; i < groupNum; i++) {
      startPaths[i].reset(new pcl::PointCloud<pcl::PointXYZ>());
    }
    for (int i = 0; i < pathNum; i++) {
      paths[i].reset(new pcl::PointCloud<pcl::PointXYZI>());
    }
    for (int i = 0; i < gridVoxelNum; i++) {
      correspondences[i].resize(0);
    }
    readStartPaths();
    readPaths();
    readPathList();
    readCorrespondences();
  }
  bool process(const unitree::planning::StatePoint &loc_point,
               const unitree::planning::StatePoint &goal_point,
               pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudDwz,
               nav_msgs::msg::Path &path) {
    double joySpeed = 1.0;
    double adjacentRange = 3.0;
    // 循环保存点云
    laserCloudStack[laserCloudCount]->clear();
    *laserCloudStack[laserCloudCount] = *laserCloudDwz;
    laserCloudCount = (laserCloudCount + 1) % laserCloudStackNum;
    // 待处理点云
    plannerCloud->clear();
    for (int i = 0; i < laserCloudStackNum; i++) {
      *plannerCloud += *laserCloudStack[i];
    }
    // ego sin/cos
    float sinVehicleYaw = sin(loc_point.heading);
    float cosVehicleYaw = cos(loc_point.heading);
    // 根据距离选取点云，并将点云转换到ego坐标系
    pcl::PointXYZI point;
    plannerCloudCrop->clear();
    int plannerCloudSize = plannerCloud->points.size();
    for (int i = 0; i < plannerCloudSize; i++) {
      float pointX1 = plannerCloud->points[i].x - loc_point.x;
      float pointY1 = plannerCloud->points[i].y - loc_point.y;
      float pointZ1 = plannerCloud->points[i].z - loc_point.z;

      point.x = pointX1 * cosVehicleYaw + pointY1 * sinVehicleYaw;
      point.y = -pointX1 * sinVehicleYaw + pointY1 * cosVehicleYaw;
      point.z = pointZ1;
      point.intensity = plannerCloud->points[i].intensity;

      float dis = sqrt(point.x * point.x + point.y * point.y);
      if (dis < adjacentRange) {
        plannerCloudCrop->push_back(point);
      }
    }
    //
    float pathRange = adjacentRange;
    pathRange = adjacentRange * joySpeed;
    if (pathRange < minPathRange)
      pathRange = minPathRange;
    // 计算目标点到ego距离，并转换到ego坐标系
    float relativeGoalDis;
    float relativeGoalX = ((goal_point.x - loc_point.x) * cosVehicleYaw +
                           (goal_point.y - loc_point.y) * sinVehicleYaw);
    float relativeGoalY = (-(goal_point.x - loc_point.x) * sinVehicleYaw +
                           (goal_point.y - loc_point.y) * cosVehicleYaw);
    relativeGoalDis =
        sqrt(relativeGoalX * relativeGoalX + relativeGoalY * relativeGoalY);
    // 计算目标角度，每次限制在正负90度
    joyDir = atan2(relativeGoalY, relativeGoalX) * 180 / M_PI;
    if (joyDir > 90.0)
      joyDir = 90.0;
    else if (joyDir < -90.0)
      joyDir = -90.0;
    //
    bool pathFound = false;
    float defPathScale = pathScale;
    pathScale = defPathScale * joySpeed;
    if (pathScale < minPathScale)
      pathScale = minPathScale;
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
      float angOffset = atan2(vehicleWidth, vehicleLength) * 180.0 / M_PI;
      int plannerCloudCropSize = plannerCloudCrop->points.size();
      for (int i = 0; i < plannerCloudCropSize; i++) {
        // 缩放局部坐标系下点云位置
        float x = plannerCloudCrop->points[i].x / pathScale;
        float y = plannerCloudCrop->points[i].y / pathScale;
        float h = plannerCloudCrop->points[i].intensity;
        float dis = sqrt(x * x + y * y);
        // 这个点是否有效
        if (dis < pathRange / pathScale) {
          // 这个点处有效的探索方向
          for (int rotDir = 0; rotDir < 36; rotDir++) {
            // 正负180
            float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
            // 计算和target的error的abs
            float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
            // 同样归一化到正负180
            if (angDiff > 180.0) {
              angDiff = 360.0 - angDiff;
            }
            // 无效的方向
            if (angDiff > dirThre && !dirToVehicle) {
              continue;
            }
            //
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
              // 局部唯一坐标
              int ind = gridVoxelNumY * indX + indY;
              int blockedPathByVoxelNum = correspondences[ind].size();
              for (int j = 0; j < blockedPathByVoxelNum; j++) {
                if (h > obstacleHeightThre) {
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
      }

      if (minObsAngCW > 0)
        minObsAngCW = 0;
      if (minObsAngCCW < 0)
        minObsAngCCW = 0;

      for (int i = 0; i < 36 * pathNum; i++) {
        int rotDir = int(i / pathNum);
        float angDiff = fabs(joyDir - (10.0 * rotDir - 180.0));
        if (angDiff > 180.0) {
          angDiff = 360.0 - angDiff;
        }
        if ((angDiff > dirThre && !dirToVehicle) ||
            (fabs(10.0 * rotDir - 180.0) > dirThre && fabs(joyDir) <= 90.0 &&
             dirToVehicle) ||
            ((10.0 * rotDir > dirThre && 360.0 - 10.0 * rotDir > dirThre) &&
             fabs(joyDir) > 90.0 && dirToVehicle)) {
          continue;
        }

        if (clearPathList[i] < pointPerPathThre) {
          float penaltyScore = 1.0 - pathPenaltyList[i] / costHeightThre;
          if (penaltyScore < costScore)
            penaltyScore = costScore;

          float dirDiff = fabs(joyDir - endDirPathList[i % pathNum] -
                               (10.0 * rotDir - 180.0));
          if (dirDiff > 360.0) {
            dirDiff -= 360.0;
          }
          if (dirDiff > 180.0) {
            dirDiff = 360.0 - dirDiff;
          }

          float rotDirW;
          if (rotDir < 18)
            rotDirW = fabs(fabs(rotDir - 9) + 1);
          else
            rotDirW = fabs(fabs(rotDir - 27) + 1);
          float groupDirW = 4 - fabs(pathList[i % pathNum] - 3);

          float score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * rotDirW *
                        rotDirW * rotDirW * rotDirW * penaltyScore;
          if (relativeGoalDis < goalCloseDis)
            score = (1 - sqrt(sqrt(dirWeight * dirDiff))) * groupDirW *
                    groupDirW * penaltyScore;
          if (score > 0) {
            clearPathPerGroupScore[groupNum * rotDir + pathList[i % pathNum]] +=
                score;
          }
        }
      }

      float maxScore = 0;
      int selectedGroupID = -1;
      for (int i = 0; i < 36 * groupNum; i++) {
        int rotDir = int(i / groupNum);
        float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;
        float rotDeg = 10.0 * rotDir;
        if (rotDeg > 180.0)
          rotDeg -= 360.0;
        if (maxScore < clearPathPerGroupScore[i] &&
            ((rotAng * 180.0 / M_PI > minObsAngCW &&
              rotAng * 180.0 / M_PI < minObsAngCCW) ||
             (rotDeg > minObsAngCW && rotDeg < minObsAngCCW))) {
          maxScore = clearPathPerGroupScore[i];
          selectedGroupID = i;
        }
      }
      // result
      if (selectedGroupID >= 0) {
        int rotDir = int(selectedGroupID / groupNum);
        float rotAng = (10.0 * rotDir - 180.0) * M_PI / 180;

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
        break;
        pathFound = true;
      } else {
        if (pathScale >= minPathScale + pathScaleStep) {
          pathScale -= pathScaleStep;
          pathRange = adjacentRange * pathScale / defPathScale;
        } else {
          pathRange -= pathRangeStep;
        }
      }
    }
    pathScale = defPathScale;
    if (!pathFound) {
      path.poses.resize(1);
      path.poses[0].pose.position.x = 0;
      path.poses[0].pose.position.y = 0;
      path.poses[0].pose.position.z = 0;
      path.header.stamp = rclcpp::Time(static_cast<uint64_t>(odomTime * 1e9));
      path.header.frame_id = "vehicle";
    }
    return true;
  }

private:
  int readPlyHeader(FILE *filePtr) {
    char str[50];
    int val, pointNum;
    string strCur, strLast;
    while (strCur != "end_header") {
      val = fscanf(filePtr, "%s", str);
      if (val != 1) {
        exit(1);
      }

      strLast = strCur;
      strCur = string(str);

      if (strCur == "vertex" && strLast == "element") {
        val = fscanf(filePtr, "%d", &pointNum);
        if (val != 1) {
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
      exit(1);
    }

    if (pathNum != readPlyHeader(filePtr)) {
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
        exit(1);
      }

      if (pathID >= 0 && pathID < pathNum && groupID >= 0 &&
          groupID < groupNum) {
        pathList[pathID] = groupID;
        endDirPathList[pathID] = 2.0 * atan2(endY, endX) * 180 / M_PI;
      }
    }

    fclose(filePtr);
  }
  void readCorrespondences() {
    string fileName = pathFolder + "/correspondences.txt";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      exit(1);
    }

    int val1, gridVoxelID, pathID;
    for (int i = 0; i < gridVoxelNum; i++) {
      val1 = fscanf(filePtr, "%d", &gridVoxelID);
      if (val1 != 1) {
        exit(1);
      }

      while (1) {
        val1 = fscanf(filePtr, "%d", &pathID);
        if (val1 != 1) {
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
  void readPaths() {
    string fileName = pathFolder + "/paths.ply";

    FILE *filePtr = fopen(fileName.c_str(), "r");
    if (filePtr == NULL) {
      exit(1);
    }

    int pointNum = readPlyHeader(filePtr);

    pcl::PointXYZI point;
    int pointSkipNum = 30;
    int pointSkipCount = 0;
    int val1, val2, val3, val4, val5, pathID;
    for (int i = 0; i < pointNum; i++) {
      val1 = fscanf(filePtr, "%f", &point.x);
      val2 = fscanf(filePtr, "%f", &point.y);
      val3 = fscanf(filePtr, "%f", &point.z);
      val4 = fscanf(filePtr, "%d", &pathID);
      val5 = fscanf(filePtr, "%f", &point.intensity);

      if (val1 != 1 || val2 != 1 || val3 != 1 || val4 != 1 || val5 != 1) {
        exit(1);
      }

      if (pathID >= 0 && pathID < pathNum) {
        pointSkipCount++;
        if (pointSkipCount > pointSkipNum) {
          paths[pathID]->push_back(point);
          pointSkipCount = 0;
        }
      }
    }

    fclose(filePtr);
  }
  std::string pathFolder = "/home/unitree/code/unitree_ros2/localplanner/paths";
  double odomTime = 0.0;
  double dirWeight = 0.02;
  double goalCloseDis = 0.4;
  double costScore = 0.02;
  int pointPerPathThre = 2;
  double goalClearRange = 0.5;
  double joyDir = 0.0;
  double dirThre = 90.0;
  bool dirToVehicle = false;
  double laserVoxelSize = 0.05;
  double vehicleLength = 0.3;
  double vehicleWidth = 0.7;
  double minPathScale = 0.5;
  double pathScale = 0.75;
  double minPathRange = 1.0;
  int laserCloudCount = 0;
  double pathRangeStep = 0.5;
  double pathScaleStep = 0.25;
  double obstacleHeightThre = 0.3;
  double groundHeightThre = 0.1;
  double costHeightThre = 0.1;

  std::vector<int> correspondences[gridVoxelNum];
  pcl::PointCloud<pcl::PointXYZI>::Ptr paths[pathNum];
  pcl::PointCloud<pcl::PointXYZ>::Ptr startPaths[groupNum];

  pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudStack[laserCloudStackNum];
  float endDirPathList[pathNum] = {0};
  int clearPathList[36 * pathNum] = {0};
  float pathPenaltyList[36 * pathNum] = {0};
  float clearPathPerGroupScore[36 * groupNum] = {0};
};