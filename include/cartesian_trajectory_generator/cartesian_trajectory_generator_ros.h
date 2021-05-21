
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

namespace cartesian_trajectory_generator
{
visualization_msgs::Marker makeBox(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::Marker marker;

  marker.type = visualization_msgs::Marker::SPHERE;
  marker.scale.x = msg.scale * 0.45;
  marker.scale.y = msg.scale * 0.45;
  marker.scale.z = msg.scale * 0.45;
  marker.color.r = 0.5;
  marker.color.g = 0.8;
  marker.color.b = 0.8;
  marker.color.a = 0.7;

  return marker;
}

visualization_msgs::InteractiveMarkerControl &makeBoxControl(visualization_msgs::InteractiveMarker &msg)
{
  visualization_msgs::InteractiveMarkerControl control;
  control.always_visible = true;
  control.markers.push_back(makeBox(msg));
  msg.controls.push_back(control);

  return msg.controls.back();
}

void addMarkerControls(visualization_msgs::InteractiveMarker &int_marker)
{
  visualization_msgs::InteractiveMarkerControl control;
  tf::Quaternion orien(1.0, 0.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "rotate_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_x";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf::Quaternion(0.0, 1.0, 0.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "rotate_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_z";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);

  orien = tf::Quaternion(0.0, 0.0, 1.0, 1.0);
  orien.normalize();
  tf::quaternionTFToMsg(orien, control.orientation);
  control.name = "rotate_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::ROTATE_AXIS;
  int_marker.controls.push_back(control);
  control.name = "move_y";
  control.interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_AXIS;
  int_marker.controls.push_back(control);
}

class cartesian_trajectory_generator_ros
{
public:
  cartesian_trajectory_generator_ros()
  {
    double publish_rate{ 0 };
    std::string pose_topic;
    std::string new_goal_topic;
    std::string current_goal_topic;
    double rot_v_max_{ 0 };
    double trans_a_{ 0 };
    double rot_a_{ 0 };
    double trans_d_{ 0 };
    double rot_d_{ 0 };
    bool synced{ false };
    if (!(n_.getParam("pose_topic", pose_topic) && n_.getParam("new_goal_topic", new_goal_topic) &&
          n_.getParam("current_goal_topic", current_goal_topic) && n_.getParam("frame_name", frame_name_) &&
          n_.getParam("ee_link", ee_link_) && n_.getParam("publish_rate", publish_rate) &&
          n_.getParam("trans_v_max", trans_v_max_) && n_.getParam("rot_v_max", rot_v_max_) &&
          n_.getParam("trans_a", trans_a_) && n_.getParam("rot_a", rot_a_)))
    {
      ROS_ERROR("Failed to load required parameters. Are they load to the parameter server?");
      ros::shutdown();
    }
    n_.param<double>("trans_d", trans_d_, trans_a_);
    n_.param<double>("rot_d", rot_d_, trans_a_);
    n_.param<bool>("sync", synced, false);

    rate_ = ros::Rate(publish_rate);
    pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
    pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>(current_goal_topic, 1);
    sub_goal_ = n_.subscribe(new_goal_topic, 1, &cartesian_trajectory_generator_ros::goal_msg_callback, this);
    srv_overlay_ = n_.advertiseService("overlay_motion", &cartesian_trajectory_generator_ros::overlay_callback, this);

    pose_msg_.header.frame_id = frame_name_;
    requested_pose_.header.frame_id = frame_name_;
    // Set base parameters
    auto t = ctg_.get_translation_obj();
    t->set_acceleration(trans_a_);
    t->set_deceleration(trans_d_);
    t->set_v_max(trans_v_max_);
    auto r = ctg_.get_rotation_obj();
    r->set_acceleration(rot_a_);
    r->set_deceleration(rot_d_);
    r->set_v_max(rot_v_max_);
    ctg_.set_synchronized(synced);

    config_pose_server.setCallback(boost::bind(&cartesian_trajectory_generator_ros::config_callback, this, _1, _2));
    traj_start_ = ros::Time::now();

    visualization_msgs::InteractiveMarker int_marker;
    server_.reset(new interactive_markers::InteractiveMarkerServer("cartesian_trajectory_generator", "", false));
    int_marker.header.frame_id = frame_name_;
    int_marker.scale = 0.5;
    int_marker.name = "Goal Pose";
    int_marker.description = "6-DOF Goal Pose";
    makeBoxControl(int_marker);
    int_marker.controls[0].interaction_mode = visualization_msgs::InteractiveMarkerControl::MOVE_ROTATE_3D;
    addMarkerControls(int_marker);
    server_->insert(int_marker);
    server_->setCallback(int_marker.name,
                         boost::bind(&cartesian_trajectory_generator_ros::processMarkerFeedback, this, _1));
    menu_handler_.insert("Send Pose",
                         boost::bind(&cartesian_trajectory_generator_ros::processMarkerFeedback, this, _1));
    menu_handler_.insert("Reset Marker Pose",
                         boost::bind(&cartesian_trajectory_generator_ros::processMarkerFeedback, this, _1));
    menu_handler_.apply(*server_, int_marker.name);

    // Action Server Setup
    as_ = std::unique_ptr<actionlib::SimpleActionServer<cartesian_trajectory_generator::TrajectoryAction>>(
        new actionlib::SimpleActionServer<cartesian_trajectory_generator::TrajectoryAction>(
            n_, std::string("goal_action"), false));
    as_->registerGoalCallback(boost::bind(&cartesian_trajectory_generator_ros::action_goal_callback, this));
    as_->registerPreemptCallback(boost::bind(&cartesian_trajectory_generator_ros::action_preempt_callback, this));
    as_->start();
  }

  bool getInitialPose(Eigen::Vector3d &startPosition, Eigen::Quaterniond &startOrientation)
  {
    tf::StampedTransform transform;
    try
    {
      tf_listener_.waitForTransform(frame_name_, ee_link_, ros::Time(0), ros::Duration(3.0));
      tf_listener_.lookupTransform(frame_name_, ee_link_, ros::Time(0), transform);
    }
    catch (tf::TransformException &ex)
    {
      ROS_ERROR("%s", ex.what());
      return false;
    }
    tf::vectorTFToEigen(transform.getOrigin(), startPosition);
    tf::quaternionTFToEigen(transform.getRotation(), startOrientation);
    return true;
  }

  void update_goal()
  {
    first_goal_ = true;
    requested_orientation_.normalize();
    // publishing latest request once
    requested_pose_.header.stamp = ros::Time::now();
    tf::pointEigenToMsg(requested_position_, requested_pose_.pose.position);
    tf::quaternionEigenToMsg(requested_orientation_, requested_pose_.pose.orientation);
    pub_goal_.publish(requested_pose_);
    update_marker_pose(requested_pose_.pose);
    // get initial pose
    Eigen::Vector3d startPosition;
    Eigen::Quaterniond startOrientation;
    getInitialPose(startPosition, startOrientation);

    ROS_INFO("Starting position:(x=%2lf,y=%2lf,z=%2lf) \t orientation:(x=%3lf,y=%3lf,z=%3lf, w=%3lf) ",
             startPosition[0], startPosition[1], startPosition[2], startOrientation.coeffs()[0],
             startOrientation.coeffs()[1], startOrientation.coeffs()[2], startOrientation.coeffs()[3]);
    ctg_.update_goal(startPosition, startOrientation, requested_position_, requested_orientation_);
    traj_start_ = ros::Time::now();
  }

  void update_marker_pose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
  {
    geometry_msgs::Pose pose;
    tf::pointEigenToMsg(pos, pose.position);
    tf::quaternionEigenToMsg(rot, pose.orientation);
    update_marker_pose(pose);
  }

  void update_marker_pose(const geometry_msgs::Pose &pose)
  {
    server_->setPose("Goal Pose", pose);
  }

  void goal_msg_callback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    goal_callback(msg);
  }

  bool goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (msg->header.frame_id == frame_name_)
    {
      tf::pointMsgToEigen(msg->pose.position, requested_position_);
      tf::quaternionMsgToEigen(msg->pose.orientation, requested_orientation_);
    }
    else
    {
      try
      {
        geometry_msgs::PoseStamped t_msg;
        tf_listener_.transformPose(frame_name_, ros::Time(0), *msg, msg->header.frame_id, t_msg);
        tf::pointMsgToEigen(t_msg.pose.position, requested_position_);
        tf::quaternionMsgToEigen(t_msg.pose.orientation, requested_orientation_);
      }
      catch (tf::TransformException ex)
      {
        ROS_ERROR("%s Did not update goal.", ex.what());
        return false;
      }
    }
    update_goal();
    return true;
  }

  void config_callback(cartesian_trajectory_generator::pose_paramConfig &config, uint32_t level)
  {
    if (config.ready_to_send)
    {
      config.ready_to_send = false;
      requested_position_[0] = config.position_x;
      requested_position_[1] = config.position_y;
      requested_position_[2] = config.position_z;
      requested_orientation_ = Eigen::AngleAxisd(config.roll, Eigen::Vector3d::UnitX()) *
                               Eigen::AngleAxisd(config.pitch, Eigen::Vector3d::UnitY()) *
                               Eigen::AngleAxisd(config.yaw, Eigen::Vector3d::UnitZ());

      update_goal();
      ROS_INFO("Request from dynamic reconfig-server recieved");
    }
  }

  bool overlay_callback(cartesian_trajectory_generator::OverlayMotionRequest &req,
                        cartesian_trajectory_generator::OverlayMotionResponse &res)
  {
    if (overlay_f_)
    {
      overlay_fade_ = overlay_f_->get_translation((ros::Time::now() - overlay_start_).toSec());
    }
    if (req.motion == "archimedes")
    {
      auto o = std::make_shared<cartesian_trajectory_generator::archimedes_spiral>();
      overlay_f_ = o;
      o->set_allow_decrease(req.allow_decrease);
      Eigen::Vector3d vec;
      tf::vectorMsgToEigen(req.dir, vec);
      o->set_direction(vec);
      o->set_max_radius(req.radius);
      o->set_path_velocity(req.path_velocity);
      o->set_path_distance(req.path_distance);
      overlay_start_ = ros::Time::now();
    }
    else
    {
      overlay_f_.reset();
    }
    if (!first_goal_)
    {
      getInitialPose(requested_position_, requested_orientation_);
      update_goal();
    }
    return true;
  }

  void action_goal_callback()
  {
    boost::shared_ptr<const cartesian_trajectory_generator::TrajectoryGoal> goal;
    goal = as_->acceptNewGoal();
    if (!goal_callback(boost::make_shared<const geometry_msgs::PoseStamped>(goal->goal)))
    {
      action_result_.error_code = action_result_.TF_FAILED;
      as_->setAborted(action_result_);
      return;
    }
    ROS_INFO("Accepted new goal");
  }

  void action_preempt_callback()
  {
    ROS_INFO("Actionserver got preempted.");
    as_->setPreempted();
  }

  void action_feedback()
  {
    if (as_->isActive())
    {
      if (ctg_.get_total_time() != 0.0)
      {
        action_feedback_.time_percentage = std::min(1.0, trajectory_t_ / ctg_.get_total_time());
      }
      else
      {
        action_feedback_.time_percentage = 0.0;
      }
      double d_trans, d_rot = 1.0;
      if (ctg_.get_trans_distance() != 0.0)
      {
        d_trans = ctg_.get_trans_distance(trajectory_t_) / ctg_.get_trans_distance();
      }
      if (ctg_.get_rot_distance() != 0.0)
      {
        d_rot = ctg_.get_rot_distance(trajectory_t_) / ctg_.get_rot_distance();
      }
      action_feedback_.path_percentage = std::min(d_trans, d_rot);
      as_->publishFeedback(action_feedback_);
      if (action_feedback_.time_percentage == 1.0)
      {
        action_result_.error_code = action_result_.SUCCESSFUL;
        as_->setSucceeded(action_result_);
      }
    }
  }

  void overlay_fade(Eigen::Vector3d &pos)
  {
    double norm = overlay_fade_.norm();
    if (norm > 0.0)
    {
      double diff = rate_.expectedCycleTime().toSec() * 0.25 * trans_v_max_;
      if (diff > norm)
      {
        overlay_fade_ = Eigen::Vector3d::Zero();
      }
      else
      {
        overlay_fade_ = overlay_fade_ * (norm - diff) / norm;
      }
      pos += overlay_fade_;
    }
  }

  void publish_msg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
  {
    pose_msg_.header.stamp = ros::Time::now();
    tf::pointEigenToMsg(pos, pose_msg_.pose.position);
    tf::quaternionEigenToMsg(rot, pose_msg_.pose.orientation);
    pub_pose_.publish(pose_msg_);
  }

  void publish_tf(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
  {
    tf::vectorEigenToTF(pos, tf_pos_);
    tf_br_transform_.setOrigin(tf_pos_);
    tf::quaternionEigenToTF(rot, tf_rot_);
    tf_br_transform_.setRotation(tf_rot_);
    tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), frame_name_, ee_link_ + "_ref_pose"));
  }

  void run()
  {
    while (!getInitialPose(requested_position_, requested_orientation_))
    {
      ROS_INFO_STREAM("Waiting for inital transform from " << frame_name_ << " to " << ee_link_);
      ros::Duration(1.0).sleep();
    }
    update_marker_pose(requested_position_, requested_orientation_);
    ctg_.update_goal(requested_position_, requested_orientation_, requested_position_, requested_orientation_);
    traj_start_ = ros::Time::now();

    ROS_INFO_STREAM("Setup complete.");
    double t_o{ 0. };
    Eigen::Vector3d pos{ requested_position_ };
    Eigen::Quaterniond rot{ requested_orientation_ };

    while (n_.ok())
    {
      trajectory_t_ = (ros::Time::now() - traj_start_).toSec();
      t_o = (ros::Time::now() - overlay_start_).toSec();
      pos = ctg_.get_position(trajectory_t_);
      rot = ctg_.get_orientation(trajectory_t_);
      if (overlay_f_)
      {
        pos += overlay_f_->get_translation(t_o);
      }
      overlay_fade(pos);
      if (first_goal_)
      {
        publish_msg(pos, rot);
        action_feedback();
      }
      publish_tf(pos, rot);
      server_->applyChanges();
      ros::spinOnce();
      rate_.sleep();
    }
  }

  void processMarkerFeedback(const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
  {
    if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT &&
        feedback->menu_entry_id == 1)
    {
      ROS_INFO_STREAM("New marker pose received");
      geometry_msgs::PoseStamped msg;
      msg.header.frame_id = frame_name_;
      msg.pose = feedback->pose;
      geometry_msgs::PoseStampedConstPtr msg_ptr(new geometry_msgs::PoseStamped(msg));
      goal_callback(msg_ptr);
    }
    else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT &&
             feedback->menu_entry_id == 2)
    {
      Eigen::Vector3d current_position{ Eigen::Vector3d::Zero() };
      Eigen::Quaterniond current_orientation{ Eigen::Quaterniond::Identity() };
      if (getInitialPose(current_position, current_orientation))
      {
        update_marker_pose(current_position, current_orientation);
      }
    }

    server_->applyChanges();
  }

private:
  ros::NodeHandle n_;
  cartesian_trajectory_generator_base<cartesian_trajectory_generator::constant_acceleration,
                                      cartesian_trajectory_generator::constant_acceleration>
      ctg_;
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
  ros::Time traj_start_ = ros::Time::now();
  std::shared_ptr<cartesian_trajectory_generator::overlay_base> overlay_f_;
  ros::Time overlay_start_ = ros::Time::now();
  Eigen::Vector3d overlay_fade_{ Eigen::Vector3d::Zero() };
  bool first_goal_{ false };

  geometry_msgs::PoseStamped pose_msg_;
  cartesian_trajectory_generator::TrajectoryFeedback action_feedback_;
  cartesian_trajectory_generator::TrajectoryResult action_result_;
  std::string frame_name_;
  std::string ee_link_;
  double trans_v_max_{ 0 };
  double trajectory_t_{ 0. };

  ros::Rate rate_ = 1;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_br_;
  tf::Transform tf_br_transform_;
  tf::Vector3 tf_pos_;
  tf::Quaternion tf_rot_;

  dynamic_reconfigure::Server<cartesian_trajectory_generator::pose_paramConfig> config_pose_server;
};

}  // namespace cartesian_trajectory_generator