
#include <cartesian_trajectory_generator/cartesian_trajectory_generator_ros.h>
#include <cartesian_trajectory_generator/pose.h>
int main(int argc, char **argv)
{
    //rosrun cartesian_trajectory_generator cartesian_trajectory_generator  /bh/CartesianImpedance_trajectory_controller/target_pose bh_link_ee 100 5.0 1.0

    ros::init(argc, argv, "cartesian_trajectory_generator");
    
    int nrInputs=1;
    if (argc != nrInputs)
    {
        ROS_INFO_STREAM("Invalid number of inputs: " << argc << "/" << nrInputs);
        return 1;
    }
    ros::NodeHandle n;
    cartesian_trajectory_generator_ros gen;
    gen.run();
    return 0;
}
