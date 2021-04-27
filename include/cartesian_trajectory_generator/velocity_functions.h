#pragma once

#include <cmath>
#include <exception>

namespace cartesian_trajectory_generator
{
//* Base class for velocity functions.
/**
 * Assumes that there is some maximum velocity.
 * Derived classes need to implement
 * - parameterize()
 * - get_velocity(time)
 * - get_distance(time)
 * Parameterize is called every time a new distance is set.
 */
class velocity_base
{
public:
  velocity_base() = default;

  velocity_base(double v_max)
  {
    set_v_max(v_max);
  }

  ~velocity_base() = default;

  virtual void parameterize() = 0;

  virtual double get_velocity(double time) = 0;

  /*! \brief Get total time needed to cover the distance. */
  double get_time()
  {
    return time_;
  }

  /*! \brief Get the requested distance. */
  double get_distance()
  {
    return dist_;
  }

  /*! \brief Get the covered distance at a given time. */
  virtual double get_distance(double time) = 0;

  /*! \brief Get the maximum velocity. */
  double get_v_max()
  {
    return v_max_;
  }

  /*! \brief Set a new distance. */
  void set_distance(double dist)
  {
    if (dist > 0)
    {
      dist_ = dist;
      parameterize();
    }
    else
    {
      throw std::invalid_argument("Distances must be positive.");
    }
  }

  /*! \brief Set a maximum velocity. */
  void set_v_max(double v_max)
  {
    if (v_max > 0)
    {
      v_max_ = v_max;
    }
    else
    {
      throw std::invalid_argument("Maximum velocity must be positive.");
    }
  }

protected:
  double time_{ 0 };  /*!< Time needed for the defined distance. */
  double dist_{ 0 };  /*!< Stores the specified distance. */
  double v_max_{ 0 }; /*!< Stores the maximum veloicty. */
};

//* Velocity function with constant acceleration.
/**
 * It is parameterized with positive values for acceleration and deceleration.
 */
class constant_acceleration : public velocity_base
{
public:
  constant_acceleration() = default;

  constant_acceleration(double acc, double dec, double v_max)
  {
    set_acceleration(acc);
    set_deceleration(dec);
    set_v_max(v_max);
  }

  ~constant_acceleration() = default;

  /*! \brief Get the total distance needed to accelerate. */
  double get_acc_dist()
  {
    return get_acc_dist(get_acc_time());
  }

  /*! \brief Get the distance needed to accelerate until a given time. */
  double get_acc_dist(double time)
  {
    return 0.5 * acc_ * std::pow(time, 2.0);
  }

  /*! \brief Get the total distance needed to decelerate. */
  double get_dec_dist()
  {
    return get_dec_dist(get_dec_time());
  }

  /*! \brief Get the distance needed to accelerate until a given time. */
  double get_dec_dist(double time)
  {
    return 0.5 * dec_ * std::pow(time, 2.0);
  }

  /*! \brief Get the time needed to accelerate. */
  double get_acc_time()
  {
    return v_max_ / acc_;
  }

  /*! \brief Get the time needed to accelerate. */
  double get_dec_time()
  {
    return v_max_ / dec_;
  }

  /*! \brief Get the total time needed to accelerate and decelerate. */
  double get_total_acc_times()
  {
    return get_acc_time() + get_dec_time();
  }

  /*! \brief Set acceleration. Value must be positive.*/
  void set_acceleration(double acc)
  {
    if (acc > 0)
    {
      acc_ = acc;
    }
    else
    {
      throw std::invalid_argument("Acceleration must be positive.");
    }
  }

  /*! \brief Set deceleration. Value must be positive.*/
  void set_deceleration(double dec)
  {
    if (dec > 0)
    {
      dec_ = dec;
    }
    else
    {
      throw std::invalid_argument("Deceleration must be positive.");
    }
  }

  /*! \brief Get the covered distance up to a specified point in time.*/
  double get_distance(double time) override
  {
    // The trajectory has ended
    if (time > time_)
    {
      return dist_;
    }
    // Deceleration phase
    else if (time > (t_acc_ + t_const_))
    {
      return get_acc_dist(t_acc_) + v_max_ * t_const_ + get_dec_dist(time - t_acc_ - t_const_);
    }

    else if (time > t_acc_)
    {
      return get_acc_dist(t_acc_) + v_max_ * (time - t_acc_);
    }
    // Acceleration phase
    else if (time >= 0)
    {
      return get_acc_dist(time);
    }
    // Negative time means that the trajectory has not started
    else
    {
      return 0;
    }
  }

  /*! \brief Get the velocity at a specified point in time.*/
  double get_velocity(double time) override
  {
    // The trajectory has ended
    if (time > time_)
    {
      return 0;
    }
    // Deceleration phase
    else if (time > (t_acc_ + t_const_))
    {
      double vel = -dec_ * (time - t_const_) + (acc_ + dec_) * t_acc_;
      // Never return negative values.
      return (vel >= 0) ? vel : 0;
    }
    // Constant velocity phase
    else if (time > t_acc_)
    {
      return v_max_;
    }
    // Acceleration phase
    else if (time >= 0)
    {
      return acc_ * time;
    }
    // Negative time means that the trajectory has not started
    else
    {
      return 0;
    }
  }

private:
  /*! \brief Internal function to parameterize when a new distance is given.*/
  void parameterize() override
  {
    if (acc_ <= 0)
    {
      throw std::runtime_error("Acceleration must be positive.");
    }
    if (dec_ <= 0)
    {
      throw std::runtime_error("Deceleration must be positive.");
    }
    if (v_max_ <= 0)
    {
      throw std::runtime_error("Maximum velocity must be positive.");
    }
    // Case 1: The trajectory has a part with constant velocity
    if (dist_ > (get_acc_dist() + get_dec_dist()))
    {
      t_acc_ = get_acc_time();
      t_const_ = (dist_ - get_acc_dist() - get_dec_dist()) / v_max_;
      t_dec_ = get_dec_time();
    }
    // Case 2: No constant velocity and the trajectory only accelerates and decelerates
    else
    {
      t_acc_ = std::sqrt(-2 * dist_ / ((acc_ + dec_) * (1 - (acc_ + dec_) / dec_)));
      t_const_ = 0;
      t_dec_ = t_acc_ * ((acc_ + dec_) / dec_ - 1);
    }
    time_ = t_acc_ + t_const_ + t_dec_;
  }

  double acc_{ 0 };     /*!< Acceleration. Must be positive. */
  double dec_{ 0 };     /*!< Deceleration. Must be positive. */
  double t_acc_{ 0 };   /*!< Time needed to accelerate for the defined distance. */
  double t_const_{ 0 }; /*!< Time of constant velocity for the defined distance. */
  double t_dec_{ 0 };   /*!< Time needed to decelerate for the defined distance. */
};

}  // namespace cartesian_trajectory_generator