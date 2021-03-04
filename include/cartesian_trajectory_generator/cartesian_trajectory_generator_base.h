
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
class cartesian_trajectory_generator_base
{
public:
    std::vector<Eigen::Vector3d> get_position()
    {
        return position_array;
    }

    std::vector<double> get_velocity()
    {
        return velocity_array;
    }

    bool makePlan(Eigen::Vector3d startPosition, Eigen::Quaterniond startOrientation, Eigen::Vector3d endPosition, Eigen::Quaterniond endOrientation, double v_max, double a_max, double publish_rate)
    {
        double distance = (endPosition - startPosition).norm();
        Eigen::Vector3d direction = (endPosition - startPosition) / distance; //normalized direction
        double accelerationDistance = v_max * v_max / (2 * a_max);
        double accelerationTime = v_max / a_max;
        double distanceAcceleration = v_max * accelerationTime / 2;
        double distanceConstantVel = distance - 2 * distanceAcceleration;
        double timeConstantVel = distanceConstantVel / v_max;
        double totTime = timeConstantVel + accelerationTime * 2;
        int i = 0;
        double tol = 0.001;
        currentPosition = startPosition;
        double time;
        //to check if it starts going backwards

        while ((currentPosition - endPosition).norm() > tol)

        {
            time = i * 1 / publish_rate;

            if (accelerationDistance * 2 < distance) //first case: we will reach maximum velocity and be able to deaccelerate before reaching endpose
            {
                if (time < accelerationTime) // acceleration phase
                {
                    v = a_max * time;
                    currentPosition = startPosition + 0.5 * a_max * time * time * direction;
                }
                else
                {
                    v = v_max; //max velocity reached
                    currentPosition = startPosition + 0.5 * a_max * accelerationTime * accelerationTime * direction + v * (time - accelerationTime) * direction;
                    double currentDistance = (currentPosition - startPosition).norm();

                    if (distanceAcceleration >= distance - currentDistance)
                    { //if its time to slow down
                        double newtime = time - timeConstantVel - accelerationTime;
                        v = v_max - a_max * newtime;
                        currentPosition = startPosition + 0.5 * a_max * accelerationTime * accelerationTime * direction + v_max * (timeConstantVel)*direction - 0.5 * a_max * newtime * newtime * direction + v_max * newtime * direction;
                    }
                }
            }
            else
            {
                v = a_max * time;
                currentPosition = startPosition + 0.5 * a_max * time * time * direction;
                double currentDistance = (currentPosition - startPosition).norm();
                if (currentDistance >= distance / 2)
                {
                    double v_peak = sqrt(2 * a_max * distance / 2);
                    double time_peak = v_peak / a_max;
                    if (time <= time_peak * 2)
                    {
                        v = v_peak - a_max * (time - time_peak);

                        currentPosition = startPosition + 0.5 * a_max * time_peak * time_peak * direction - 0.5 * a_max * (time - time_peak) * (time - time_peak) * direction + v_peak * (time - time_peak) * direction;
                    }
                }
            }

            velocity_array.push_back(v);
            position_array.push_back(currentPosition);

            i++;
        }
        velocity_array.push_back(0);
        position_array.push_back(endPosition);
        return true;
    }

private:
    Eigen::Vector3d currentPosition;
    double v = 0;
    std::vector<Eigen::Vector3d> position_array;
    std::vector<double> velocity_array;
};
