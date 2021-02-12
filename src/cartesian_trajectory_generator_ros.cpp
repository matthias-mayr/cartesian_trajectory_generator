
#include <cartesian_trajectory_generator/cartesian_trajectory_generator_ros.h>

int main(int argc, char **argv)
{
    //terminal e.g. : rosrun cartesian_trajectory_generator cartesian_trajectory_generator 1 1 1 0 0 0 1 publish_pose frame_name 100 0.2 0.05  
    ros::init(argc, argv, "cartesian_trajectory_generator");
    int nrInputs = 13;
    if (argc != nrInputs)
    {
        ROS_INFO_STREAM("Invalid number of inputs: " << argc << "/" << nrInputs);
        return 1;
    }
    cartesian_trajectory_generator_ros gen;
    Eigen::Vector3d endPosition(atof(argv[1]), atof(argv[2]), atof(argv[3]));
    Eigen::Quaterniond endOrientation(atof(argv[4]), atof(argv[5]), atof(argv[6]), atof(argv[7]));
    std::string topic_name = argv[8];
    std::string frame_name=argv[9];
    double publish_rate = atof(argv[10]);
    double v_max = atof(argv[11]);
    double a_max = atof(argv[12]);
    gen.setParameters(topic_name, publish_rate, v_max, a_max,frame_name);
    gen.requestingPose(endPosition, endOrientation);

    gen.run();

    return 0;
}
