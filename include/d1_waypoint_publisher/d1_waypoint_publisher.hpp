#ifndef D1_WAYPOINT_PUBLISHER_CORE_HPP_
#define D1_WAYPOINT_PUBLISHER_CORE_HPP_

#include <cmath>
#include <stdexcept>
#include <fstream>
#include <string>
#include <sstream>
#include <vector>
#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "nav_msgs/msg/odometry.hpp"

#include "std_msgs/msg/string.hpp"

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"


#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2/transform_datatypes.h>
#include "tf2/LinearMath/Quaternion.h"

#include "tsukutsuku2_msgs/msg/waypoints.hpp"
#include "tsukutsuku2_msgs/msg/waypoint.hpp"
#include "tsukutsuku2_msgs/msg/state_and_feedback.hpp"
#include "tsukutsuku2_msgs/msg/d1.hpp"



#define FOLLOW_WAYPOINTS_MODE 1
#define THROUGH_POSES_MODE 0

#define STANBY 0
#define PROCCESSING 1
#define ABORTED 2
#define REJECTED 3
#define CANCELING 4


#define NO_SEND -1
#define APPROACH_WHITE_LINE 0
#define APPROACH_GREEN_BOX 1
#define APPROACH_AREA 2
#define APPROACH_GOAL 3
#define APPROACH_BLUE_BOX 4


using namespace std::chrono_literals;

using std::placeholders::_1;
using std::placeholders::_2;

class D1WaypointPublisher : public rclcpp::Node{
public:
  D1WaypointPublisher();

private:
  std::vector<std::string> getCSVLine(std::string& input, char delimiter);
  void ReadWaypointsFromCSV(std::string& csv_file, std::vector<tsukutsuku2_msgs::msg::Waypoint>& waypoints_);
  //void ReadWaypointsFromCSV(std::string& csv_file, std::vector<tsukutsuku2_msgs::msg::Waypoints>& waypoints_);
  //void PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void AreaCallback(const tsukutsuku2_msgs::msg::D1::SharedPtr msg);
  void FindBoxCallback(const tsukutsuku2_msgs::msg::D1::SharedPtr msg);
  void StateAndFeedbackCallback(const tsukutsuku2_msgs::msg::StateAndFeedback::SharedPtr msg);
  void FindCharacterCallback(const tsukutsuku2_msgs::msg::D1::SharedPtr msg);
  void BoxPoseCallback(const tsukutsuku2_msgs::msg::D1::SharedPtr msg); 
  void SendWaypointsTimerCallback();
  size_t SendWaypointsOnce(size_t sending_index);

private:
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr d1_waypoint_pub_;
  rclcpp::Publisher<tsukutsuku2_msgs::msg::Waypoints>::SharedPtr d1_waypoints_publisher_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr area_sub_;
  //rclcpp::Subscription<std_msgs::msg::String>::SharedPtr find_box_sub_;
  rclcpp::Subscription<tsukutsuku2_msgs::msg::D1>::SharedPtr area_sub_;
  rclcpp::Subscription<tsukutsuku2_msgs::msg::D1>::SharedPtr find_box_sub_;
  rclcpp::Subscription<tsukutsuku2_msgs::msg::D1>::SharedPtr find_character_sub_;
  rclcpp::Subscription<tsukutsuku2_msgs::msg::D1>::SharedPtr box_pose_sub_;
  rclcpp::Subscription<tsukutsuku2_msgs::msg::StateAndFeedback>::SharedPtr state_and_feedback_sub_;

  geometry_msgs::msg::Pose current_pose_;
  std_msgs::msg::String area_character_;


  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr shutdown_timer_;  // 終了タイマー
  int id_;
  //std::string csv_file_;
  std::string waypoints_file_1_;
  std::string waypoints_file_2_;
  std::string waypoints_file_3_;
  std::string waypoints_file_4_;
  std::string waypoints_file_5_;
  std::string waypoints_file_6_;
  
  std::vector<std::string> csv_file_;//自分で追加

  tsukutsuku2_msgs::msg::Waypoint waypoint;
  tsukutsuku2_msgs::msg::Waypoints waypoints_msg;
  tsukutsuku2_msgs::msg::StateAndFeedback state_and_feedback_msg;

  tsukutsuku2_msgs::msg::D1 d1_msg;
  

  std::vector<tsukutsuku2_msgs::msg::Waypoint> waypoints_; 
  //auto waypoints_msg = tsukutsuku2_msgs::msg::Waypoints();
  
  size_t start_index_;

  static int state_;

  int elapsed_time_;  // 経過時間を保持する変数

  int find_point_;//初期化してない
  int find_character_;
  int goal_point_;//初期化してない


  int follow_type_;
  int start_index_int_;
  bool is_action_server_ready_;
  bool is_goal_achieved_;
  bool is_aborted_;
  bool is_standby_;
  bool is_goal_accepted_;

  uint8_t next_state_;
  uint8_t now_feedback_;

  std::string box_character;
  bool find_box;
  bool find_character;

  int approach_green_box_flag;
  int approach_blue_box_flag;
  int approach_goal_flag;
  int approach_area_flag;

  double current_yaw_;

  double goal_x;
  double goal_y;

  int16_t number_of_poses_remaining_;
};
#endif