#include <ros/ros.h>

#include "cartesian_trajectory_generator_base.h"


class cartesian_trajectory_generator_ros {
    public:
        cartesian_trajectory_generator_ros() {
            // Initialize here
        }
        void run() {
            while (_n.ok())
            {
                // Publish the message.
                // node_example->publishMessage(&pub_message);

                ros::spinOnce();
                _r.sleep();
            }
        }
    private:
        ros::NodeHandle _n;
        ros::Rate _r{1};


};