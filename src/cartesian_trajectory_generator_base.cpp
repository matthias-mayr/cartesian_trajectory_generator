#include <cartesian_trajectory_generator/cartesian_trajectory_generator_base.h>

std::vector<Eigen::Vector3d> cartesian_trajectory_generator_base::pop_position(std::vector<Eigen::Vector3d> &position)
{
    position = position_array;
    position_array.clear();
    return position;
}

std::vector<Eigen::Quaterniond> cartesian_trajectory_generator_base::pop_orientation()
{
    std::vector<Eigen::Quaterniond> temp = orientation_array;
    orientation_array.clear();
}

std::vector<double> cartesian_trajectory_generator_base::pop_velocity(std::vector<double> &velocity)
{
    velocity = velocity_array;
    velocity_array.clear();
    return velocity;
}

std::vector<double> cartesian_trajectory_generator_base::pop_time(std::vector<double> &time)
{
    time = time_array;
    time_array.clear();
    return time;
}

double cartesian_trajectory_generator_base::get_total_time()
{
    return totTime;
}

bool cartesian_trajectory_generator_base::makePlan(Eigen::Vector3d startPosition, Eigen::Quaterniond startOrientation, Eigen::Vector3d endPosition, Eigen::Quaterniond endOrientation, double v_max, double a_max, double publish_rate)
{

    double distance = (endPosition - startPosition).norm();
    Eigen::Vector3d direction = (endPosition - startPosition) / distance; //normalized direction
    double accelerationDistance = v_max * v_max / (2 * a_max);
    double accelerationTime = v_max / a_max;
    double distanceAcceleration = v_max * accelerationTime / 2;
    double distanceConstantVel = distance - 2 * distanceAcceleration;
    double timeConstantVel = distanceConstantVel / v_max;

    int i = 0;
    double tol = 0.0001;
    currentPosition = startPosition;
    currentOrientation = startOrientation;
    double time;

    //estimate the total time
    if (accelerationDistance * 2 < distance)
    {
        totTime = timeConstantVel + accelerationTime * 2;
    }
    else
    {
        totTime = 2 * sqrt(distance / a_max);
    }

    double calibration_frequency=20000; //higher=more accurate but slower , lower= viceversa

    while ((currentPosition - endPosition).norm() > tol)

    {

        time = i * 1 / calibration_frequency;
        time_array.push_back(time);

        if (time > totTime * 2)
        { //multiplited with 2 just to be sure
            return false;
        }
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
        //orientation_array.push_back(currentOrientation);
        i++;
    }
    time_array.push_back(time);
    velocity_array.push_back(0);
    position_array.push_back(endPosition);

    //downsampling 
    bool check=true; 
    int data_points=publish_rate/calibration_frequency*time_array.size();
    check = down_sample(time_array, data_points) && down_sample(velocity_array, data_points) && down_sample(position_array, data_points);
    
    if (check)
    {
        return true;
    }
    else
    {
        return false;
    }
}

//samples down the vector v to datapoints+1 elements
bool cartesian_trajectory_generator_base::down_sample(std::vector<double> &v, int data_points)
{
    if (data_points >= v.size() || data_points <= 2)
    {
        return false;
    }
    std::vector<double> v_s;
    int jump = v.size() / data_points;
    int i = 0;
    while (i < data_points)
    {
        v_s.push_back(v[jump * i]);
        i++;
    }
    v_s.push_back(v[v.size() - 1]);
    v.clear();
    v = v_s;
    return true;
}

bool cartesian_trajectory_generator_base::down_sample(std::vector<Eigen::Vector3d> &v, int data_points)
{

    if (data_points >= v.size() || data_points <= 2)
    {
        return false;
    }
    std::vector<Eigen::Vector3d> v_s;
    int jump = v.size() / data_points;
    int i = 0;
    while (i < data_points)
    {
        v_s.push_back(v[jump * i]);
        i++;
    }
    v_s.push_back(v[v.size() - 1]);
    v.clear();
    v = v_s;
    return true;
}