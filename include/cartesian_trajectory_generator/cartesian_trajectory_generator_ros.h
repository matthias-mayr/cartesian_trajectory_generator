
#pragma once

#include <cartesian_trajectory_generator/cartesian_trajectory_generator_base.h>
#include <cartesian_trajectory_generator/pose_paramConfig.h>
#include <dynamic_reconfigure/server.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>

namespace cartesian_trajectory_generator
{
class cartesian_trajectory_generator_ros
{
public:
  cartesian_trajectory_generator_ros()
  {
    double publish_rate{ 0 };
    std::string pose_topic;
    std::string goal_topic;
    double trans_v_max_{ 0 };
    double rot_v_max_{ 0 };
    double trans_a_{ 0 };
    double rot_a_{ 0 };
    double trans_d_{ 0 };
    double rot_d_{ 0 };
    bool synced{ false };
    if (!(n_.getParam("cartesian_trajectory_generator/pose_topic", pose_topic) &&
          n_.getParam("cartesian_trajectory_generator/goal_topic", goal_topic) &&
          n_.getParam("cartesian_trajectory_generator/frame_name", frame_name_) &&
          n_.getParam("cartesian_trajectory_generator/ee_link", ee_link_) &&
          n_.getParam("cartesian_trajectory_generator/publish_rate", publish_rate) &&
          n_.getParam("cartesian_trajectory_generator/trans_v_max", trans_v_max_) &&
          n_.getParam("cartesian_trajectory_generator/rot_v_max", rot_v_max_) &&
          n_.getParam("cartesian_trajectory_generator/trans_a", trans_a_) &&
          n_.getParam("cartesian_trajectory_generator/rot_a", rot_a_)))
    {
      ROS_ERROR("Failed to load required parameters. Are they load to the parameter server?");
      ros::shutdown();
    }
    n_.param<double>("cartesian_trajectory_generator/trans_d", trans_d_, trans_a_);
    n_.param<double>("cartesian_trajectory_generator/rot_d", rot_d_, trans_a_);
    n_.param<bool>("cartesian_trajectory_generator/sync", synced, false);

    rate_ = ros::Rate(publish_rate);
    pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>(pose_topic, 1);
    pub_goal_ = n_.advertise<geometry_msgs::PoseStamped>("/test", 1);
    sub_goal_ = n_.subscribe(goal_topic, 1, &cartesian_trajectory_generator_ros::goal_callback, this);

    pose_msg_.header.frame_id = frame_name_;
    latest_poseStamped_request.header.frame_id = frame_name_;
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
  }

  void getInitialPose(Eigen::Vector3d &startPosition, Eigen::Quaterniond &startOrientation)
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
      return;
    }
    tf::vectorTFToEigen(transform.getOrigin(), startPosition);
    tf::quaternionTFToEigen(transform.getRotation(), startOrientation);
  }

  void update_goal()
  {
    // publishing latest request once
    tf::pointEigenToMsg(requested_position_, latest_poseStamped_request.pose.position);
    tf::quaternionEigenToMsg(requested_orientation_, latest_poseStamped_request.pose.orientation);
    pub_goal_.publish(latest_poseStamped_request);
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

  void goal_callback(const geometry_msgs::PoseStampedConstPtr &msg)
  {
    if (msg->header.frame_id == frame_name_)
    {
      tf::pointMsgToEigen(msg->pose.position, requested_position_);
      tf::quaternionMsgToEigen(msg->pose.orientation, requested_orientation_);
      update_goal();
    }
    else
    {
      ROS_ERROR_STREAM("Goal in wrong frame. Please use frame " << frame_name_);
    }
  }

  void config_callback(cartesian_trajectory_generator::pose_paramConfig &config, uint32_t level)
  {
    if (config.ready_to_send)
    {
      config.ready_to_send = false;
      requested_position_[0] = config.position_x;
      requested_position_[1] = config.position_y;
      requested_position_[2] = config.position_z;
      requested_orientation_.coeffs()[0] = config.orientation_x;
      requested_orientation_.coeffs()[1] = config.orientation_y;
      requested_orientation_.coeffs()[2] = config.orientation_z;
      requested_orientation_.coeffs()[3] = config.orientation_w;
      update_goal();
      ROS_INFO("Request from dynamic reconfig-server recieved");
    }
  }

  void publish(const Eigen::Vector3d &pos, const Eigen::Quaterniond &rot)
  {
    pose_msg_.header.stamp = ros::Time::now();
    tf::pointEigenToMsg(pos, pose_msg_.pose.position);
    tf::quaternionEigenToMsg(rot, pose_msg_.pose.orientation);
    pub_pose_.publish(pose_msg_);
    // Publish tf of the ref pose
    tf::vectorEigenToTF(pos, tf_pos_);
    tf_br_transform_.setOrigin(tf_pos_);
    tf::quaternionEigenToTF(rot, tf_rot_);
    tf_br_transform_.setRotation(tf_rot_);
    tf_br_.sendTransform(tf::StampedTransform(tf_br_transform_, ros::Time::now(), frame_name_, "ref_pose"));
  }

  void run()
  {
    double t{ 0. };
    Eigen::Vector3d pos{ Eigen::Vector3d::Zero() };
    Eigen::Quaterniond rot{ Eigen::Quaterniond() };
    while (n_.ok())
    {
      if (ros::Time::now() - traj_start_ < ros::Duration(1.1 * ctg_.get_total_time()))
      {
        t = (ros::Time::now() - traj_start_).toSec();
        pos = ctg_.get_position(t);
        rot = ctg_.get_orientation(t);
        publish(pos, rot);
      }
      ros::spinOnce();
      rate_.sleep();
    }
  }

private:
  ros::NodeHandle n_;
  cartesian_trajectory_generator_base<cartesian_trajectory_generator::constant_acceleration,
                                      cartesian_trajectory_generator::constant_acceleration>
      ctg_;
  ros::Publisher pub_pose_;
  ros::Subscriber sub_goal_;
  ros::Publisher pub_goal_;

  Eigen::Vector3d requested_position_;
  Eigen::Quaterniond requested_orientation_;
  ros::Time traj_start_ = ros::Time::now();

  geometry_msgs::PoseStamped pose_msg_;
  geometry_msgs::PoseStamped latest_poseStamped_request;
  std::string frame_name_;
  std::string ee_link_;

  ros::Rate rate_ = 1;

  tf::TransformListener tf_listener_;
  tf::TransformBroadcaster tf_br_;
  tf::Transform tf_br_transform_;
  tf::Vector3 tf_pos_;
  tf::Quaternion tf_rot_;

  dynamic_reconfigure::Server<cartesian_trajectory_generator::pose_paramConfig> config_pose_server;
};

}  // namespace cartesian_trajectory_generator