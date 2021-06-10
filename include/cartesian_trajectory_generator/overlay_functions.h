#pragma once

#include <Eigen/Geometry>
#include <cmath>
#include <exception>

namespace cartesian_trajectory_generator
{
//* Base class for overlay functions.
/**
 * Derived classes need to implement
 * - get_velocity(time)
 * - get_distance(time)
 */
class overlay_base
{
public:
  overlay_base() = default;

  ~overlay_base() = default;

  /*! \brief Get the last time value. */
  double get_last_time()
  {
    return time_;
  }

  /*! \brief */
  virtual Eigen::Vector3d get_translation(double time) = 0;

  /*! \brief */
  virtual Eigen::Quaterniond get_rotation(double time) = 0;

  Eigen::Vector3d get_translation_rotated(double time, const Eigen::Quaterniond& rot)
  {
    return rot * get_translation(time);
  }

  Eigen::Quaterniond get_rotation_rotated(double time, const Eigen::Quaterniond& rot)
  {
    return rot * get_rotation(time);
  }

protected:
  double time_{ 0 }; /*!< Last time. */
};

class archimedes_spiral : public overlay_base
{
public:
  Eigen::Vector3d get_translation(double time)
  {
    if (time > time_)
    {
      update(time);
    }
    else if (time < time_)
    {
      throw std::invalid_argument("Time might not decrease.");
    }
    add_[0] = r_ * std::cos(theta_);
    add_[1] = r_ * std::sin(theta_);
    add_[2] = 0.0;
    // Rotate vector
    return rot_ * add_;
  }

  Eigen::Quaterniond get_rotation(double time)
  {
    return Eigen::Quaterniond::Identity();
  }

  void set_direction(const Eigen::Vector3d& dir)
  {
    rot_ = Eigen::Quaterniond::FromTwoVectors(Eigen::Vector3d::UnitZ(), dir.normalized());
  }

  void set_max_radius(double r)
  {
    if (r >= 0)
    {
      max_r_ = r;
    }
    else
    {
      throw std::invalid_argument("Maximum radius must be non-negative.");
    }
  }

  void set_path_distance(double d)
  {
    if (d >= 0)
    {
      d_ = d;
    }
    else
    {
      throw std::invalid_argument("Distance must be non-negative.");
    }
  }

  void set_path_velocity(double v)
  {
    v_ = v;
  }

  void set_allow_decrease(bool decrease)
  {
    allow_decrease_ = decrease;
  }

private:
  void update(double t)
  {
    double theta_d = v_ * (t - time_) / (r_ + 0.001);
    double r_d = theta_d * d_ / (2 * M_PI);
    if (v_ < 0)
    {
      r_d *= -1;
    }
    theta_ += theta_d;
    increase_r_ ? r_ += r_d : r_ -= r_d;

    if (allow_decrease_ && (r_ > max_r_))
    {
      increase_r_ = false;
    }
    if (r_ > max_r_)
    {
      r_ = max_r_;
    }
    if (r_ < 0)
    {
      increase_r_ = true;
      r_ = 0.0;
    }
    time_ = t;
  }

  double r_{ 0 };
  double theta_{ 0 };
  double d_{ 0 };
  double v_{ 0 };
  double max_r_{ 0 };
  bool allow_decrease_{ false };
  bool increase_r_{ true };
  Eigen::Quaterniond rot_{ Eigen::Quaterniond::Identity() };
  Eigen::Vector3d add_{ Eigen::Vector3d::Zero() };
};

}  // namespace cartesian_trajectory_generator