
#pragma once

#include <actionlib/server/simple_action_server.h>
#include <cartesian_trajectory_generator/OverlayMotion.h>
#include <cartesian_trajectory_generator/TrajectoryAction.h>
#include <cartesian_trajectory_generator/cartesian_trajectory_generator_base.h>
#include <cartesian_trajectory_generator/overlay_functions.h>
#include <cartesian_trajectory_generator/pose_paramConfig.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <interactive_markers/interactive_marker_server.h>
#include <interactive_markers/menu_handler.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <memory>
#include <string>

namespace cartesian_trajectory_generator
{
class cartesianTrajectoryGeneratorRos
{
public:
  cartesianTrajectoryGeneratorRos();

  void actionFeedback();

  void actionGoalCallback();

  void actionPreemptCallback();

  void applyOverlay(Eigen::Vector3d& pos, double t_o);

  void markerConfigCallback(cartesian_trajectory_generator::pose_paramConfig &config, uint32_t level);

  bool getCurrentPose(Eigen::Vector3d &startPosition, Eigen::Quaterniond &startOrientation, bool print_exception = true);

  bool goalCallback(const geometry_msgs::PoseStampedConstPtr &msg, bool get_initial_pose = true);

  void goalMsgCallback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    this->trans_goal_threshold_ = this->trans_goal_threshold_default_;
    this->rot_goal_threshold_ = this->rot_goal_threshold_default_;
    goalCallback(msg);
  }

  bool overlayCallback(cartesian_trajectory_generator::OverlayMotionRequest &req,
                       cartesian_trajectory_generator::OverlayMotionResponse &res);

  bool overlayFadeOut();

  void applyOverlayFadeOut(Eigen::Vector3d &pos);

  void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback);

  void publishRefMsg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot);

  void publishRefTf(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot);

  void run();

  bool updateGoal(bool get_start_pose = true);

  void updateMarkerPose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot);

  void updateMarkerPose(const geometry_msgs::Pose &pose)
  {
    server_->setPose("Goal Pose", pose);
  }

private:
  cartesian_trajectory_generator_base<cartesian_trajectory_generator::constant_acceleration,
                                      cartesian_trajectory_generator::constant_acceleration>
      ctg_;

  // ROS Communication
  ros::NodeHandle n_;
  ros::Publisher pub_pose_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_goal_;
  ros::ServiceServer srv_overlay_;
  std::unique_ptr<actionlib::SimpleActionServer<cartesian_trajectory_generator::TrajectoryAction>> as_;
  boost::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
  interactive_markers::MenuHandler menu_handler_;

  Eigen::Vector3d requested_position_;
  Eigen::Quaterniond requested_orientation_;
  geometry_msgs::PoseStamped requested_pose_;
  Eigen::Vector3d start_position_;
  Eigen::Quaterniond start_orientation_;
  Eigen::Vector3d current_position_;
  Eigen::Quaterniond current_orientation_;
  ros::Time traj_start_ = ros::Time::now();
  std::shared_ptr<cartesian_trajectory_generator::overlay_base> overlay_f_;
  ros::Time overlay_start_ = ros::Time::now();
  Eigen::Vector3d overlay_fade_{ Eigen::Vector3d::Zero() };
  std::string overlay_frame_id_;
  bool first_goal_{ false };
  bool publish_constantly_ { false };

  geometry_msgs::PoseStamped pose_msg_;
  cartesian_trajectory_generator::TrajectoryFeedback action_feedback_;
  cartesian_trajectory_generator::TrajectoryResult action_result_;
  std::string frame_name_;
  std::string ee_link_;
  double trans_goal_threshold_{0};
  double rot_goal_threshold_{0};
  double trans_goal_threshold_default_{0};
  double rot_goal_threshold_default_{0};
  double trans_v_max_{0};
  double trajectory_t_{ 0. };

  ros::Rate rate_ = 1;

  // tf frame
  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_br_;
  tf::Transform tf_br_transform_;
  ros::Time tf_last_time_ = ros::Time::now();
  tf::Vector3 tf_pos_;
  tf::Quaternion tf_rot_;

  // Dynamic reconfigure
  dynamic_reconfigure::Server<cartesian_trajectory_generator::pose_paramConfig> config_pose_server;
};

}  // namespace cartesian_trajectory_generator
