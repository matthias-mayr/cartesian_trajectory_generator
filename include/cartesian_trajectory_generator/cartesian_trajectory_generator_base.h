#pragma once

#include <Eigen/Geometry>
#include <exception>
#include <memory>

#include <cartesian_trajectory_generator/velocity_functions.h>

namespace cartesian_trajectory_generator
{
template <typename v_trans_t, typename v_rot_t>
class cartesian_trajectory_generator_base
{
public:
  cartesian_trajectory_generator_base() = default;
  ~cartesian_trajectory_generator_base() = default;

  Eigen::Vector3d get_position(double time)
  {
    return start_position_ + trans->get_distance(trans_t_ * time) * trans_vec_;
  }

  Eigen::Quaterniond get_orientation(double time)
  {
    if (d_rot_ == 0.)
    {
      return end_orientation_;
    }
    return start_orientation_.slerp((rot->get_distance(rot_t_ * time) / d_rot_), end_orientation_);
  }

  double get_trans_distance(double time)
  {
    return trans->get_distance(trans_t_ * time);
  }

  double get_rot_distance(double time)
  {
    return rot->get_distance(rot_t_ * time);
  }

  double get_trans_vel(double time)
  {
    return trans->get_velocity(trans_t_ * time);
  }

  double get_rot_vel(double time)
  {
    return rot->get_velocity(rot_t_ * time);
  }

  std::shared_ptr<v_trans_t> get_translation_obj()
  {
    return trans;
  }

  std::shared_ptr<v_rot_t> get_rotation_obj()
  {
    return rot;
  }

  double get_total_time()
  {
    return time_;
  }

  bool update_goal(const Eigen::Vector3d& startPosition, const Eigen::Quaterniond& startOrientation,
                   const Eigen::Vector3d& endPosition, const Eigen::Quaterniond& endOrientation)
  {
    trans_t_ = rot_t_ = 1.0;
    // Translation vector and distance
    start_position_ = startPosition;
    trans_vec_ = endPosition - startPosition;
    double d_trans = (endPosition - startPosition).norm();
    trans_vec_.normalize();
    // Rotation distance
    start_orientation_ = startOrientation.normalized();
    end_orientation_ = endOrientation.normalized();
    d_rot_ = start_orientation_.angularDistance(end_orientation_);
    // Parameterize velocity functions
    trans->set_distance(d_trans);
    rot->set_distance(d_rot_);
    // Synchronize them if needed
    if (synchronized_)
    {
      if (trans->get_time() > rot->get_time())
      {
        rot_t_ = rot->get_time() / trans->get_time();
      }
      else if (rot->get_time() > trans->get_time())
      {
        trans_t_ = trans->get_time() / rot->get_time();
      }
    }
    time_ = std::max(trans->get_time(), rot->get_time());
  }

  void set_synchronized(bool sync)
  {
    synchronized_ = sync;
  }

private:
  std::shared_ptr<v_trans_t> trans{ new v_trans_t };
  std::shared_ptr<v_rot_t> rot{ new v_rot_t };

  bool synchronized_{ false };
  double trans_t_{ 1.0 };
  double rot_t_{ 1.0 };
  Eigen::Vector3d start_position_;
  Eigen::Vector3d trans_vec_;
  Eigen::Quaterniond start_orientation_;
  Eigen::Quaterniond end_orientation_;
  double d_rot_;
  double time_{ 0 };
};

}  // namespace cartesian_trajectory_generator