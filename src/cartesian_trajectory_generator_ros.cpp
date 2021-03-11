
#include <cartesian_trajectory_generator/cartesian_trajectory_generator_ros.h>
#include <cartesian_trajectory_generator/pose.h>
int main(int argc, char **argv)
{
    //rosrun cartesian_trajectory_generator cartesian_trajectory_generator  /bh/CartesianImpedance_trajectory_controller/target_pose bh_link_ee 100 5.0 1.0

    ros::init(argc, argv, "cartesian_trajectory_generator");
    
    int nrInputs=6;
    if (argc != nrInputs)
    {
        ROS_INFO_STREAM("Invalid number of inputs: " << argc << "/" << nrInputs);
        return 1;
    }
    ros::NodeHandle n;
    
    cartesian_trajectory_generator_ros gen;
  
   std::string topic_name = argv[1];
    std::string frame_name=argv[2];
    double publish_rate = atof(argv[3]);
    double v_max = atof(argv[4]);
    double a_max = atof(argv[5]);
    gen.setParameters(topic_name, publish_rate, v_max, a_max,frame_name);


    gen.run();

    return 0;
}
