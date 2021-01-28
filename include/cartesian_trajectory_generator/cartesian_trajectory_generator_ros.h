#include <ros/ros.h>

#include "cartesian_trajectory_generator_base.h"

#include "geometry_msgs/PoseStamped.h"

class cartesian_trajectory_generator_ros {
    public:
    
        cartesian_trajectory_generator_ros() {
            // Initialize here
    

    pub = _n.advertise<geometry_msgs::PoseStamped>("channel1", 1000);
    poseStamped.header.stamp = ros::Time::now();
    poseStamped.header.frame_id = "world";
    poseStamped.pose.position.x=0;
    poseStamped.pose.position.y=0;
    poseStamped.pose.position.z=0;
    poseStamped.pose.orientation.x=0;
    poseStamped.pose.orientation.y=0;
    poseStamped.pose.orientation.z=0;
    poseStamped.pose.orientation.w=0;
    
// ros::ServiceServer service = _n.advertiseService<geometry_msgs::PoseStamped>(topic_name, 1000);

    




        }
        void run() {
            while (_n.ok())
            {
    
    pub.publish(poseStamped);    
        // Publish the message.
                // node_example->publishMessage(&pub_message);

                ros::spinOnce();
                _r.sleep();
            }

        }
    private:
    geometry_msgs::PoseStamped poseStamped;
    ros::Publisher pub;
        ros::NodeHandle _n;
        ros::Rate _r{1};


};

