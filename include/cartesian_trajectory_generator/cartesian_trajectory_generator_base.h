
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
#include <ros/ros.h>
class cartesian_trajectory_generator_base
{
public:
    std::vector<Eigen::Vector3d> pop_position(std::vector<Eigen::Vector3d> &position);

    std::vector<Eigen::Quaterniond> pop_orientation();

    std::vector<double> pop_velocity(std::vector<double> &velocity);

    std::vector<double> pop_time(std::vector<double> &time);
    double get_total_time();
    bool makePlan(Eigen::Vector3d startPosition, Eigen::Quaterniond startOrientation, Eigen::Vector3d endPosition, Eigen::Quaterniond endOrientation, double v_max, double a_max, double publish_rate);

private:
    //samples down the vector v to datapoints+1 elements
    bool down_sample(std::vector<double> &v, int data_points);
    bool down_sample(std::vector<Eigen::Vector3d> &v, int data_points);

    Eigen::Vector3d currentPosition;
    Eigen::Quaterniond currentOrientation;
    double v = 0;
    std::vector<Eigen::Vector3d> position_array;
    std::vector<Eigen::Quaterniond> orientation_array;
    std::vector<double> velocity_array;
    std::vector<double> time_array;
    double totTime;
};
