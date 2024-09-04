#include "common.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <nav_msgs/msg/path.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>

#include <string>
#include <vector>
using namespace std;
class Controller {
public:
  Controller() {}
  void process(const unitree::planning::StatePoint &loc_point,
               const nav_msgs::msg::Path &path, const bool &stop,
               unitree_api::msg::Request &req) {
    //   float vehicleXRel = cos(vehicleYawRec) * (vehicleX - vehicleXRec)
    //                     + sin(vehicleYawRec) * (vehicleY - vehicleYRec);
    //   float vehicleYRel = -sin(vehicleYawRec) * (vehicleX - vehicleXRec)
    //                     + cos(vehicleYawRec) * (vehicleY - vehicleYRec);
    float vehicleXRel = 0.0;
    float vehicleYRel = 0.0;
    int pathSize = path.poses.size();
    float endDisX = path.poses[pathSize - 1].pose.position.x - vehicleXRel;
    float endDisY = path.poses[pathSize - 1].pose.position.y - vehicleYRel;
    float endDis = sqrt(endDisX * endDisX + endDisY * endDisY);
    float disX, disY, dis;
    // 找预瞄点
    while (pathPointID < pathSize - 1) {
      disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
      disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
      dis = sqrt(disX * disX + disY * disY);
      if (dis < lookAheadDis) {
        pathPointID++;
      } else {
        break;
      }
    }
    // 计算航向误差
    disX = path.poses[pathPointID].pose.position.x - vehicleXRel;
    disY = path.poses[pathPointID].pose.position.y - vehicleYRel;
    dis = sqrt(disX * disX + disY * disY);
    float pathDir = atan2(disY, disX);
    float dirDiff = -pathDir;
    if (dirDiff > M_PI)
      dirDiff -= 2 * M_PI;
    else if (dirDiff < -M_PI)
      dirDiff += 2 * M_PI;
    if (dirDiff > M_PI)
      dirDiff -= 2 * M_PI;
    else if (dirDiff < -M_PI)
      dirDiff += 2 * M_PI;
    // 计算yaw_rate
    if (fabs(vehicleSpeed) < 2.0 * maxAccel / 100.0)
      vehicleYawRate = -stopYawRateGain * dirDiff;
    else
      vehicleYawRate = -yawRateGain * dirDiff;
    if (vehicleYawRate > maxYawRate * M_PI / 180.0)
      vehicleYawRate = maxYawRate * M_PI / 180.0;
    else if (vehicleYawRate < -maxYawRate * M_PI / 180.0)
      vehicleYawRate = -maxYawRate * M_PI / 180.0;
    if (pathSize <= 1 || (dis < stopDisThre)) {
      vehicleYawRate = 0;
    }
    // 计算speed
    float joySpeed2 = maxSpeed * joySpeed;
    if (pathSize <= 1) {
      joySpeed2 = 0;
    } else if (endDis / slowDwnDisThre < joySpeed) {
      joySpeed2 *= endDis / slowDwnDisThre;
    }
    float joySpeed3 = joySpeed2;
    if ((fabs(dirDiff) < dirDiffThre ||
         (dis < goalCloseDis && fabs(dirDiff) < omniDirDiffThre)) &&
        dis > stopDisThre) {
      if (vehicleSpeed < joySpeed3)
        vehicleSpeed += maxAccel / 100.0;
      else if (vehicleSpeed > joySpeed3)
        vehicleSpeed -= maxAccel / 100.0;
    } else {
      if (vehicleSpeed > 0)
        vehicleSpeed -= maxAccel / 100.0;
      else if (vehicleSpeed < 0)
        vehicleSpeed += maxAccel / 100.0;
    }

    if (fabs(vehicleSpeed) > noRotSpeed)
      vehicleYawRate = 0;
    if (stop) {
      vehicleSpeed = 0.0;
      vehicleYawRate = 0.0;
    }
    pubSkipCount--;
    if (pubSkipCount < 0) {
      if (fabs(vehicleSpeed) <= maxAccel / 100.0) {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = 0;
      } else {
        cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;
        cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;
      }
      cmd_vel.twist.angular.z = vehicleYawRate;
      pubSkipCount = 1;
      if (cmd_vel.twist.linear.x == 0 && cmd_vel.twist.linear.y == 0 &&
          cmd_vel.twist.angular.z == 0) {
        sport_req.StopMove(req);
      } else {
        sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y,
                       cmd_vel.twist.angular.z);
      }
    }
  }

private:
  double yawRateGain = 7.5;
  double lookAheadDis = 0.5;
  double maxAccel = 2.0;
  double stopYawRateGain = 1.5;
  double maxYawRate = 80.0;
  double stopDisThre = 0.3;
  double maxSpeed = 1.0;
  double slowDwnDisThre = 0.75;
  double slowTime1 = 2.0;
  double slowTime2 = 2.0;
  double slowRate2 = 0.5;
  double slowRate1 = 0.25;
  double omniDirDiffThre = 1.5;
  double goalCloseDis = 0.4;
  double vehicleSpeed = 0.0;
  double vehicleYawRate = 0.0;
  int pubSkipCount = 1;
  double noRotSpeed = 10.0;
  double dirDiffThre = 0.4;
  int pathPointID = 0;
  double joySpeed = 0.7;
  double slowInitTime = 0.0;
  geometry_msgs::msg::TwistStamped cmd_vel;
  SportClient sport_req;
};
