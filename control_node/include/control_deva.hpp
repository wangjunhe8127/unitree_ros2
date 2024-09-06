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
    pathPointID = 0;
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
    double yaw_rate_w = 1.0;
    std::cout << "all_num:" << (pathSize - 1)  <<std::endl;
    int end_diff_idx = pathSize - 1 - pathPointID;
    if (end_diff_idx < 5) {
      yaw_rate_w = 0.0;
    }
    disX = path.poses[pathPointID].pose.position.x;
    disY = path.poses[pathPointID].pose.position.y;
    dis = sqrt(disX * disX + disY * disY);
    float pathDir = atan2(disY, disX);
    float dirDiff = -pathDir;
    std::cout << "dis:" << dis<<std::endl;
    std::cout << "pathDir:" << pathDir<<std::endl;

    std::cout << "pathPointID:" << pathPointID <<std::endl;
    std::cout << "disX:" << disX <<std::endl;
    std::cout << "disY:" << disY <<std::endl;

    if (dirDiff > M_PI)
      dirDiff -= 2 * M_PI;
    else if (dirDiff < -M_PI)
      dirDiff += 2 * M_PI;
    if (dirDiff > M_PI)
      dirDiff -= 2 * M_PI;
    else if (dirDiff < -M_PI)
      dirDiff += 2 * M_PI;
     std::cout << "pathDir:" << pathDir<<std::endl;
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
     std::cout << "vehicleYawRate:" << vehicleYawRate<<std::endl;
    // 计算speed
    float joySpeed2 = maxSpeed * joySpeed;
    if (pathSize <= 1) {
      joySpeed2 = 0;
      std::cout << "joySpeed2" << joySpeed2 << std::endl;
    } else if (endDis / slowDwnDisThre < joySpeed) {
      joySpeed2 *= endDis/slowDwnDisThre;
    }
    float joySpeed3 = joySpeed2;
    std::cout << "joySpeed3" << joySpeed3 << std::endl;
    if ((fabs(dirDiff) < dirDiffThre ||
         (dis < goalCloseDis && fabs(dirDiff) < omniDirDiffThre)) &&
        dis > stopDisThre) {

      if (vehicleSpeed < joySpeed3){
                vehicleSpeed += maxAccel / 100.0;
        std::cout << "vehicleSpeed1" << vehicleSpeed << std::endl;
      }

      else if (vehicleSpeed > joySpeed3){
         vehicleSpeed -= maxAccel / 100.0;
               std::cout << "vehicleSpeed2" << vehicleSpeed << std::endl;
      }
       
    } else {
      if (vehicleSpeed > 0) {
        vehicleSpeed -= maxAccel / 100.0;
        std::cout << "vehicleSpeed3" << vehicleSpeed << std::endl;
      }
        
      else if (vehicleSpeed < 0){
        vehicleSpeed += maxAccel / 100.0;
std::cout << "vehicleSpeed4" << vehicleSpeed << std::endl;
      }
        
    }
std::cout << "vehicleSpeed" << vehicleSpeed << std::endl;
    if (fabs(vehicleSpeed) > noRotSpeed)
      vehicleYawRate = 0;
    if (stop) {
      vehicleSpeed = 0.0;
      vehicleYawRate = 0.0;
    }
    std::cout << "finalvehicleSpeed:" << vehicleSpeed<<std::endl;
std::cout << "finalvehicleYawRate:" << vehicleYawRate<<std::endl;
      if (fabs(vehicleSpeed) <= maxAccel / 100.0) {
        cmd_vel.twist.linear.x = 0;
        cmd_vel.twist.linear.y = 0;
      } else {
        cmd_vel.twist.linear.x = cos(dirDiff) * vehicleSpeed;
        cmd_vel.twist.linear.y = -sin(dirDiff) * vehicleSpeed;
      }
      cmd_vel.twist.angular.z = vehicleYawRate;

      if ((cmd_vel.twist.linear.x == 0 && cmd_vel.twist.linear.y == 0 &&
          cmd_vel.twist.angular.z == 0) || stop) {
            std::cout << "stop"<<std::endl;
        // sport_req.StopMove(req);
      } else {
        sport_req.Move(req, cmd_vel.twist.linear.x, cmd_vel.twist.linear.y,
                       cmd_vel.twist.angular.z);
      }
      std::cout << "cmd_vel.twist.linear:" << cmd_vel.twist.linear.x << " " << cmd_vel.twist.linear.y << " " << cmd_vel.twist.angular.z<<std::endl;
  }


private:
  double yawRateGain = 0.7;
  double lookAheadDis = 0.25;
  double maxAccel = 1.5;
  double stopYawRateGain = 0.7;
  double maxYawRate = 30.0;
  double stopDisThre = 0.2;
  double maxSpeed = 0.3;
  double slowDwnDisThre = 0.75;
  double slowTime1 = 2.0;
  double slowTime2 = 2.0;
  double slowRate2 = 0.5;
  double slowRate1 = 0.25;
  double omniDirDiffThre = 1.5;
  double goalCloseDis = 0.4;
  double vehicleSpeed = 0.3;
  double vehicleYawRate = 0.0;
  int pubSkipCount = 1;
  double noRotSpeed = 10.0;
  double dirDiffThre = 0.4;
  int pathPointID = 0;
  double joySpeed = 0.25;
  double slowInitTime = 0.0;
  geometry_msgs::msg::TwistStamped cmd_vel;
  SportClient sport_req;
};
