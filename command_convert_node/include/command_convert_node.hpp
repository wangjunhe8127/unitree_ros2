#include <string>
#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/dog_control_command.hpp"
#include "unitree_api/msg/request.hpp"
#include "ros2_sport_client.h"
class ConvertBase {
  public:
  ConvertBase() {}
   virtual ~ConvertBase() {}
   virtual void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) = 0;
    SportClient sport_req_;
};
class ConvertTrajectory : public ConvertBase{
  public:
   ConvertTrajectory() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
    std::vector<PathPoint> path;
    for (int i = 0; i < 30; i++)
    {
                PathPoint path_point;
                path_point.timeFromStart = data->path_point[i].t_from_start;
                path_point.x = data->path_point[i].x;
                path_point.y = data->path_point[i].y;
                path_point.yaw = data->path_point[i].yaw;
                path_point.vx = data->path_point[i].vx;
                path_point.vy = data->path_point[i].vy;
                path_point.vyaw = data->path_point[i].vyaw;
                path.push_back(path_point);
    }
    sport_req_.TrajectoryFollow(req, path);
  }
};
class ConvertPosition : public ConvertBase{
  public:
   ConvertPosition() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
    float vx = data->point.linear.x;
    float vy = data->point.linear.y;
    float vyaw = data->point.linear.z;
    sport_req_.Move(req, vx, vy, vyaw);
  }
};
class ConvertEuler : public ConvertBase{
  public:
   ConvertEuler() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
    float roll = data->point.angular.x;
    float pitch = data->point.angular.y;
    float yaw = data->point.angular.z;
    sport_req_.Euler(req, roll, pitch, yaw);
  }
};

class ConvertSpeedLevel : public ConvertBase{
  public:
   ConvertSpeedLevel() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
    int level = data->speed_level;
    sport_req_.SpeedLevel(req, level);
  }
};
class ConvertBodyHeight : public ConvertBase{
  public:
   ConvertBodyHeight() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
      float height = data->body_height;
      sport_req_.BodyHeight(req, height);
  }
};
class ConvertDamp : public ConvertBase{
  public:
   ConvertDamp() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
      sport_req_.Damp(req);
  }
};
class ConvertBalanceStand : public ConvertBase{
  public:
   ConvertBalanceStand() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
      sport_req_.BalanceStand(req);
  }
};
class ConvertStopMove :public ConvertBase{
  public:
   ConvertStopMove() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
      sport_req_.StopMove(req);
  }
};
class ConvertStandUp : public ConvertBase{
  public:
   ConvertStandUp() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
      sport_req_.StandUp(req);
  }
};
class ConvertRecoveryStand : public ConvertBase{
  public:
   ConvertRecoveryStand() : ConvertBase() {}
   void Process(unitree_go::msg::DogControlCommand::SharedPtr data,
    unitree_api::msg::Request &req) override {
      sport_req_.RecoveryStand(req);
  }
};
class CommandConvertNode : public rclcpp::Node{
 public:
  CommandConvertNode();
 private:
    std::shared_ptr<ConvertBase> convert_trajectory_ = std::make_shared<ConvertTrajectory>();
    std::shared_ptr<ConvertBase> convert_body_height_ = std::make_shared<ConvertBodyHeight>();
    std::shared_ptr<ConvertBase> convert_speed_level_ = std::make_shared<ConvertSpeedLevel>();
    std::shared_ptr<ConvertBase> convert_damp_ = std::make_shared<ConvertDamp>();
    std::shared_ptr<ConvertBase> convert_balance_stand_ = std::make_shared<ConvertBalanceStand>();
    std::shared_ptr<ConvertBase> convert_stop_move_ = std::make_shared<ConvertStopMove>();
    std::shared_ptr<ConvertBase> convert_stand_up_ = std::make_shared<ConvertStandUp>();
    std::shared_ptr<ConvertBase> convert_recovery_stand_ = std::make_shared<ConvertRecoveryStand>();
    std::shared_ptr<ConvertBase> convert_position_ = std::make_shared<ConvertPosition>();
    std::shared_ptr<ConvertBase> convert_euler_ = std::make_shared<ConvertEuler>();
  std::vector<std::shared_ptr<ConvertBase>> convert_list_;
  void control_callback(unitree_go::msg::DogControlCommand::SharedPtr data);
//   std::string low_topic_name_ = "lf/lowstate";
  std::string sub_topic_name_ = "/control/dog_control_command";
  std::string pub_topic_name_ = "/api/sport/request";
  rclcpp::Subscription<unitree_go::msg::DogControlCommand>::SharedPtr control_suber_;
  rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr control_puber_;
  SportClient sport_req_;
  unitree_api::msg::Request req_;
};