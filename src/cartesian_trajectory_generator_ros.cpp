
#include <cartesian_trajectory_generator/cartesian_trajectory_generator_ros.h>

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

cartesianTrajectoryGeneratorRos::cartesianTrajectoryGeneratorRos()
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
    ROS_ERROR_STREAM("Failed to load required parameters. Are they load to the parameter server in namespace '" << n_.getNamespace() << "'?");
    ros::shutdown();
  }
  n_.param<double>("trans_d", trans_d_, trans_a_);
  n_.param<double>("rot_d", rot_d_, trans_a_);
  n_.param<bool>("sync", synced, false);
  n_.param<bool>("publish_constantly", publish_constantly_, false);

  rate_ = ros::Rate(publish_rate);
  pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
  pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>(current_goal_topic, 1);
  sub_goal_ = n_.subscribe(new_goal_topic, 1, &cartesianTrajectoryGeneratorRos::goalMsgCallback, this);
  srv_overlay_ = n_.advertiseService("overlay_motion", &cartesianTrajectoryGeneratorRos::overlayCallback, this);

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

  config_pose_server.setCallback(boost::bind(&cartesianTrajectoryGeneratorRos::markerConfigCallback, this, _1, _2));
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
  server_->setCallback(int_marker.name, boost::bind(&cartesianTrajectoryGeneratorRos::processMarkerFeedback, this, _1));
  menu_handler_.insert("Send Pose", boost::bind(&cartesianTrajectoryGeneratorRos::processMarkerFeedback, this, _1));
  menu_handler_.insert("Reset Marker Pose",
                       boost::bind(&cartesianTrajectoryGeneratorRos::processMarkerFeedback, this, _1));
  menu_handler_.apply(*server_, int_marker.name);

  // Action Server Setup
  as_ = std::unique_ptr<actionlib::SimpleActionServer<cartesian_trajectory_generator::TrajectoryAction>>(
      new actionlib::SimpleActionServer<cartesian_trajectory_generator::TrajectoryAction>(
          n_, std::string("goal_action"), false));
  as_->registerGoalCallback(boost::bind(&cartesianTrajectoryGeneratorRos::actionGoalCallback, this));
  as_->registerPreemptCallback(boost::bind(&cartesianTrajectoryGeneratorRos::actionPreemptCallback, this));
  as_->start();
}

void cartesianTrajectoryGeneratorRos::actionFeedback()
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
      ROS_INFO("Set goal succeeded.");
      action_result_.error_code = action_result_.SUCCESSFUL;
      as_->setSucceeded(action_result_);
    }
  }
}

void cartesianTrajectoryGeneratorRos::actionGoalCallback()
{
  boost::shared_ptr<const cartesian_trajectory_generator::TrajectoryGoal> action;
  action = as_->acceptNewGoal();
  bool get_initial_pose = true;
  if (action->start.pose != geometry_msgs::Pose())
  {
    get_initial_pose = false;
    geometry_msgs::PoseStamped start_pose = action->start;
    try
    {
      tf_listener_.transformPose(this->frame_name_, action->start, start_pose);
    }
    catch(tf::TransformException ex)
    {
      ROS_ERROR("%s", ex.what());
    }

    tf::pointMsgToEigen(start_pose.pose.position, this->start_position_);
    tf::quaternionMsgToEigen(start_pose.pose.orientation, this->start_orientation_);
  }
  if (!goalCallback(boost::make_shared<const geometry_msgs::PoseStamped>(action->goal), get_initial_pose))
  {
    action_result_.error_code = action_result_.TF_FAILED;
    as_->setAborted(action_result_);
    return;
  }
  ROS_INFO("Accepted new goal.");
}

void cartesianTrajectoryGeneratorRos::actionPreemptCallback()
{
  ROS_INFO("Actionserver got preempted.");
  // Abort by setting a new goal to the current position
  if (getCurrentPose(this->requested_position_, this->requested_orientation_, true)) {
    updateGoal(true);
  } else {
    ROS_WARN("Could not update goal pose. Emergency abort by setting early start time.");
    this->traj_start_ = ros::Time::now() - ros::Duration(60 * 60);
  }
  as_->setPreempted();
}

void cartesianTrajectoryGeneratorRos::markerConfigCallback(cartesian_trajectory_generator::pose_paramConfig &config,
                                                           uint32_t level)
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

    updateGoal();
    ROS_INFO("Request from dynamic reconfig-server recieved");
  }
}

void cartesianTrajectoryGeneratorRos::applyOverlay(Eigen::Vector3d &pos, double t_o)
{
  if (!overlay_f_)
  {
    return;
  }

  Eigen::Quaterniond rot = Eigen::Quaterniond::Identity();
  if (overlay_frame_id_ != frame_name_)
  {
    try
    {
      tf::StampedTransform transform;
      tf_listener_.lookupTransform(frame_name_, overlay_frame_id_, ros::Time(0), transform);
      tf::quaternionTFToEigen(transform.getRotation(), rot);
    }
    catch (tf::TransformException ex)
    {
      ROS_ERROR_THROTTLE(1, "%s", ex.what());
      return;
    }
  }
  pos += overlay_f_->get_translation_rotated(t_o, rot);
}

bool cartesianTrajectoryGeneratorRos::getCurrentPose(Eigen::Vector3d &startPosition,
                                                     Eigen::Quaterniond &startOrientation, bool print_exception)
{
  tf::StampedTransform transform;
  try
  {
    tf_listener_.waitForTransform(frame_name_, ee_link_, ros::Time(0), ros::Duration(3.0));
    tf_listener_.lookupTransform(frame_name_, ee_link_, ros::Time(0), transform);
  }
  catch (tf::TransformException &ex)
  {
    if (print_exception)
    {
      ROS_ERROR("%s", ex.what());
    }
    return false;
  }
  tf::vectorTFToEigen(transform.getOrigin(), startPosition);
  tf::quaternionTFToEigen(transform.getRotation(), startOrientation);
  return true;
}

bool cartesianTrajectoryGeneratorRos::goalCallback(const geometry_msgs::PoseStampedConstPtr &msg, bool get_initial_pose)
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
  updateGoal(get_initial_pose);
  return true;
}

bool cartesianTrajectoryGeneratorRos::overlayCallback(cartesian_trajectory_generator::OverlayMotionRequest &req,
                                                      cartesian_trajectory_generator::OverlayMotionResponse &res)
{
  if (!first_goal_)
  {
    if (getCurrentPose(requested_position_, requested_orientation_)) {
      updateGoal();
    } else {
      ROS_WARN("Could not update goal pose. Will not apply overlay.");
      return false;
    }
  }
  if (overlay_f_)
  {
    overlay_fade_ = overlay_f_->get_translation((ros::Time::now() - overlay_start_).toSec());
  }
  if (req.motion == cartesian_trajectory_generator::OverlayMotionRequest::ARCHIMEDES)
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
    if (!req.header.frame_id.empty())
    {
      overlay_frame_id_ = req.header.frame_id;
    }
    else
    {
      overlay_frame_id_ = ee_link_;
    }
  }
  else
  {
    overlay_f_.reset();
  }
  return true;
}

bool cartesianTrajectoryGeneratorRos::overlayFadeOut()
{
  double norm = overlay_fade_.norm();
  return norm > 0.0;
}

void cartesianTrajectoryGeneratorRos::applyOverlayFadeOut(Eigen::Vector3d &pos)
{
  if (overlayFadeOut())
  {
    double norm = overlay_fade_.norm();
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

void cartesianTrajectoryGeneratorRos::processMarkerFeedback(
    const visualization_msgs::InteractiveMarkerFeedbackConstPtr &feedback)
{
  if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT &&
      feedback->menu_entry_id == 1)
  {
    ROS_INFO_STREAM("New marker pose received");
    geometry_msgs::PoseStamped msg;
    msg.header.frame_id = frame_name_;
    msg.pose = feedback->pose;
    geometry_msgs::PoseStampedConstPtr msg_ptr(new geometry_msgs::PoseStamped(msg));
    goalCallback(msg_ptr);
  }
  else if (feedback->event_type == visualization_msgs::InteractiveMarkerFeedback::MENU_SELECT &&
           feedback->menu_entry_id == 2)
  {
    Eigen::Vector3d current_position{ Eigen::Vector3d::Zero() };
    Eigen::Quaterniond current_orientation{ Eigen::Quaterniond::Identity() };
    if (getCurrentPose(current_position, current_orientation))
    {
      updateMarkerPose(this->current_position_, this->current_orientation_);
    }
  }

  server_->applyChanges();
}

void cartesianTrajectoryGeneratorRos::publishRefTf(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
{
  if (ros::Time::now() > this->tf_last_time_)
  {
    tf::vectorEigenToTF(pos, tf_pos_);
    tf_br_transform_.setOrigin(tf_pos_);
    tf::quaternionEigenToTF(rot, tf_rot_);
    tf_br_transform_.setRotation(tf_rot_);
    tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), frame_name_, ee_link_ + "_ref_pose"));
    this->tf_last_time_ = ros::Time::now();
  }
}

void cartesianTrajectoryGeneratorRos::publishRefMsg(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
{
  pose_msg_.header.stamp = ros::Time::now();
  tf::pointEigenToMsg(pos, pose_msg_.pose.position);
  tf::quaternionEigenToMsg(rot, pose_msg_.pose.orientation);
  pub_pose_.publish(pose_msg_);
}

void cartesianTrajectoryGeneratorRos::run()
{
  while (!getCurrentPose(requested_position_, requested_orientation_, false))
  {
    ROS_INFO_STREAM("Waiting for inital transform from " << frame_name_ << " to " << ee_link_);
    ros::Duration(1.0).sleep();
  }
  updateMarkerPose(requested_position_, requested_orientation_);
  ctg_.updateGoal(requested_position_, requested_orientation_, requested_position_, requested_orientation_);
  traj_start_ = ros::Time::now();

  ROS_INFO_STREAM("Setup complete.");
  double t_o{ 0. };
  Eigen::Vector3d pos{ requested_position_ };
  Eigen::Quaterniond rot{ requested_orientation_ };

  while (n_.ok())
  {
    trajectory_t_ = (ros::Time::now() - traj_start_).toSec();
    if (publish_constantly_ || trajectory_t_ < ctg_.get_total_time() || overlay_f_ || overlayFadeOut())
    {
      t_o = (ros::Time::now() - overlay_start_).toSec();
      pos = ctg_.get_position(trajectory_t_);
      rot = ctg_.get_orientation(trajectory_t_);

      applyOverlay(pos, t_o);
      applyOverlayFadeOut(pos);
      if (first_goal_)
      {
        publishRefMsg(pos, rot);
      }
    }
    actionFeedback();
    publishRefTf(pos, rot);
    server_->applyChanges();
    ros::spinOnce();
    rate_.sleep();
  }
}

bool cartesianTrajectoryGeneratorRos::updateGoal(bool get_start_pose)
{
  if (get_start_pose)
  {
    if (!getCurrentPose(start_position_, start_orientation_))
    {
      ROS_ERROR("Could not look up transform. Not setting new goal.");
      return false;
    }
  }
  first_goal_ = true;
  requested_orientation_.normalize();
  // publishing latest request once
  requested_pose_.header.stamp = ros::Time::now();
  
  tf::pointEigenToMsg(requested_position_, requested_pose_.pose.position);
  tf::quaternionEigenToMsg(requested_orientation_, requested_pose_.pose.orientation);


  pub_goal_.publish(requested_pose_);
  updateMarkerPose(requested_pose_.pose);

  ROS_INFO("Starting position:(x: %2lf,y: %2lf,z: %2lf) \t Orientation:(x: %3lf,y: %3lf,z: %3lf, w: %3lf)", start_position_[0],
           start_position_[1], start_position_[2], start_orientation_.coeffs()[0], start_orientation_.coeffs()[1],
           start_orientation_.coeffs()[2], start_orientation_.coeffs()[3]);
  ctg_.updateGoal(start_position_, start_orientation_, requested_position_, requested_orientation_);
  traj_start_ = ros::Time::now();
  return true;
}

void cartesianTrajectoryGeneratorRos::updateMarkerPose(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
{
  geometry_msgs::Pose pose;
  tf::pointEigenToMsg(pos, pose.position);
  tf::quaternionEigenToMsg(rot, pose.orientation);
  updateMarkerPose(pose);
}

}  // namespace cartesian_trajectory_generator
