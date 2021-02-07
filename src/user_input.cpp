#include <ros/ros.h>

#include "cartesian_trajectory_generator/generate_trajectory.h"
#include "geometry_msgs/PoseStamped.h"

int main(int argc, char **argv)
{

    ros::init(argc, argv, "user_input");

    if (argc != 8)
    {
        ROS_INFO("Invalid number of inputs!");
        return 1;
    }

    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<cartesian_trajectory_generator::generate_trajectory>("trajectory_plan");
    cartesian_trajectory_generator::generate_trajectory srv;
    srv.request.positionX = atof(argv[1]);
    srv.request.positionY = atof(argv[2]);
    srv.request.positionZ = atof(argv[3]);
    srv.request.orientationX = atof(argv[4]);
    srv.request.orientationY = atof(argv[5]);
    srv.request.orientationZ = atof(argv[6]);
    srv.request.orientationW = atof(argv[7]);
    // argv[8]="export ROSCONSOLE_FORMAT='$[message]";

    if (client.call(srv))
    {
        ROS_INFO("Trajectory plan:\n Position: x=%3lf, y=%3lf, z=%3lf \n Orientation: x=%3lf, y=%3lf, z=%3lf, w=%3lf \n", srv.request.positionX, srv.request.positionY, srv.request.positionZ,srv.request.orientationX, srv.request.orientationY, srv.request.orientationZ, srv.request.orientationW);
       // ros::Subscriber sub = n.subscribe("channel1", 1000, poseCallback);
    }
    else
    {
        ROS_ERROR("Failed to call service!");
        return 1;
    }

    
    // ros::spin();
    return 0;
};
