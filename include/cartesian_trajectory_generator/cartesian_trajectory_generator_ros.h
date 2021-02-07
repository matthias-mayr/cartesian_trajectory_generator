
#include <ros/ros.h>

#include "cartesian_trajectory_generator_base.h"
#include "cartesian_trajectory_generator/generate_trajectory.h"
#include "geometry_msgs/PoseStamped.h"

#include <tf2_ros/static_transform_broadcaster.h>
#include "tf/tf.h"
#include "tf/transform_listener.h"
#include "eigen3/Eigen/Core"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

class cartesian_trajectory_generator_ros
{
public:
    cartesian_trajectory_generator_ros()
    {

        //initialization
       
        topic_name = "pose";
        publish_rate = 10;
        v_max = 100; //mm/s
        a_max = 5;   //mm/s²
        poseStamped.header.frame_id = "world";
        _n.setParam("topic_name", topic_name);
        _n.setParam("publish_rate", publish_rate);
        _n.setParam("v_max", v_max);
        _n.setParam("a_max", a_max);

     
        //initial start pose
        //static tf2_ros::StaticTransformBroadcaster transform;

        //ros::Publisher publishPose=_n.advertise<geometry_msgs::PoseStamped>(topic_name,1000);
    }

    ros::Rate getClock()
    {
        ros::Rate rate(publish_rate);
        return rate;
    }

    bool getInitialPose(Eigen::Vector3d &startPosition, Eigen::Quaterniond &startOrientation)
    {
        startPosition[0] = 0.0;
        startPosition[1] = 0.0;
        startPosition[2] = 0.0;
        Eigen::Quaterniond temp(0.0, 0.0, 0.0, 0.0);
        startOrientation = temp;
        return true;
    }



    bool plan(cartesian_trajectory_generator::generate_trajectory::Request &req, cartesian_trajectory_generator::generate_trajectory::Response &resp)
    {
        //Declare publisher subscriber
        publisher = _n.advertise<geometry_msgs::PoseStamped>(topic_name, 1000);
        subscriber = _n.subscribe(topic_name, 1000, &cartesian_trajectory_generator_ros::callback, this);
        //get trajectory (stored in vector)
        Eigen::Vector3d startPosition;
        Eigen::Quaterniond startOrientation;
        getInitialPose(startPosition, startOrientation); //startpos always zero for now
        Eigen::Vector3d endPosition(req.positionX, req.positionY, req.positionZ);
        Eigen::Quaterniond endOrientation(req.orientationX, req.orientationY, req.orientationZ, req.orientationW);
        cartesian_trajectory_generator_base ctg;
        ctg.makePlan(startPosition, startOrientation, endPosition, endOrientation, v_max, a_max, publish_rate);
        the_trajectory = ctg.get_trajectory();
        velocities_trajectory=ctg.getVelocities();
        //publish with publish_rate;
            int i=0;
            ROS_INFO_STREAM("Size of trajectory: "<<the_trajectory.size());
            ROS_INFO("Publishing pose:");

            while(i<the_trajectory.size()){ 
                Eigen::Vector3d pos = the_trajectory[i];
                double vel= velocities_trajectory[i];
                poseStamped.pose.position.x = pos[0];
                poseStamped.pose.position.y = pos[1];
                poseStamped.pose.position.z = pos[2];     
                poseStamped.pose.orientation.x=vel; //TEMPORARY 
                poseStamped.header.stamp=ros::Time::now();
                publisher.publish(poseStamped);
                i++;
                ros::spinOnce();
                getClock().sleep();
            }
            ROS_INFO("Done");
        return true;
    }

    void callback(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        ROS_INFO_STREAM(*msg);
        
    }
    void run()
    {
        const std::string service_name = "trajectory_plan";
        bool notImportant;

        while (_n.ok())
        {
              service = _n.advertiseService("trajectory_plan", &cartesian_trajectory_generator_ros::plan, this);
            ROS_INFO("Initial message");
            

           ros::spin();
           getClock().sleep();
           
        }
    }

private:
    geometry_msgs::PoseStamped poseStamped;
    ros::NodeHandle _n;
    ros::ServiceServer service;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    std::string topic_name;
    double publish_rate;
    double v_max;
    double a_max;
    std::vector<Eigen::Vector3d> the_trajectory;
    std::vector<double> velocities_trajectory;
 
};
