
//<<#include <eigen_conversions/eigen_msg.h>
//#include <eigen3/Eigen/Core>
#include <Eigen/Geometry>
#include <iostream>
#include <vector>
class cartesian_trajectory_generator_base
{
public:

        std::vector<Eigen::Vector3d> get_trajectory()
        {
            return position_array;
        }

        std::vector<double> getVelocities(){
            return velocity_array;
        }
    bool almostEqual(Eigen::Vector3d v1, Eigen::Vector3d v2) //not done
    {
        //bool cond1=abs(v1[0]-v2[0])<;
        return true;
    }
    void makePlan(Eigen::Vector3d startPosition, Eigen::Quaterniond startOrientation, Eigen::Vector3d endPosition, Eigen::Quaterniond endOrientation, double v_max, double a_max, double publish_rate)
    {
        

        Eigen::Vector3d currentPosition;
        Eigen::Vector3d origin(0.0, 0.0, 0.0);
        double dx = endPosition[0] - startPosition[0];
        double dy = endPosition[1] - startPosition[1];
        double dz = endPosition[2] - startPosition[2];
        Eigen::Vector3d direction = endPosition - startPosition;

        //acceleration phase (assuming linear maximum acceleration is applied)
        double v = 0;
        double a = a_max;
        currentPosition = startPosition;
        position_array.push_back(currentPosition);
        double distance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
        double distanceStartRetardation = v_max * v_max / (2 * a_max);
        //while (!almostEqual(currentPosition, endPosition))
        for(int i=0;i<50;i++)
        {

            double dx = currentPosition[0] - startPosition[0];
            double dy = currentPosition[1] - startPosition[1];
            double dz = currentPosition[2] - startPosition[2];
            double currentDistance = sqrt(pow(dx, 2) + pow(dy, 2) + pow(dz, 2));
            velocity_array.push_back(v);
            if (abs(distance - currentDistance) >= distanceStartRetardation) //if its time to start retardation
            {
                a = -a_max;
                //v = v + a * 1 / publish_rate;
                currentPosition = currentPosition + v * direction + 0.5 * a * direction * 1 / (publish_rate * publish_rate); //x=x0+v0*t+0.5*a*t², v=v0+a*t
            }

            else
            {

                if (v < v_max) //keep accelerating until maximum velocity is reached
                {
                    v = v + a_max * 1 / publish_rate;
                    currentPosition = currentPosition + 0.5 * a * direction * 1 / (publish_rate * publish_rate); //x=x0+v0*t+0.5*a*t², v=v0+a*t
                }
                else
                {
                    v = v_max;
                    a = 0;
                    currentPosition=currentPosition+v*1/publish_rate*direction;
                }
            }

            position_array.push_back(currentPosition);
        }
    }

private:
    std::vector<Eigen::Vector3d> position_array;
    std::vector<double> velocity_array;
};
