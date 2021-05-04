
#include <cartesian_trajectory_generator/cartesian_trajectory_generator_ros.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "cartesian_trajectory_generator");

  cartesian_trajectory_generator::cartesian_trajectory_generator_ros gen;
  gen.run();
  return 0;
}
