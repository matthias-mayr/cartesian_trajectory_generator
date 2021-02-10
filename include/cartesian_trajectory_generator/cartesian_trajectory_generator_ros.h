
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
        publish_rate = 2;
        v_max = 0.2; //m/s
        a_max = 0.05;   //m/s²
        poseStamped.header.frame_id = "world";
        publisher = _n.advertise<geometry_msgs::PoseStamped>(topic_name, 1000);
        subscriber = _n.subscribe(topic_name, 1000, &cartesian_trajectory_generator_ros::callback, this);
        _n.setParam("topic_name", topic_name);
        _n.setParam("publish_rate", publish_rate);
        _n.setParam("v_max", v_max);
        _n.setParam("a_max", a_max);
        //initial start pose
        //static tf2_ros::StaticTransformBroadcaster transform;
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
        //get trajectory (stored in vector)
        Eigen::Vector3d startPosition;
        Eigen::Quaterniond startOrientation;
        getInitialPose(startPosition, startOrientation); //startpos always zero for now
        Eigen::Vector3d endPosition(req.positionX, req.positionY, req.positionZ);
       
    
        Eigen::Quaterniond endOrientation(req.orientationX, req.orientationY, req.orientationZ, req.orientationW);
       
        
            ROS_INFO("Publishing pose:");
            int i=0;
            ros::Time startTime=ros::Time::now();
            double startTime_d=startTime.toSec();
            double tol=0.001;
            while((ctg.get_position()-endPosition).norm()>tol){//while pose not reached 
                ros::Time time=ros::Time::now();
                double time_d=time.toSec()-startTime_d;
                ctg.makePlan(startPosition, startOrientation, endPosition, endOrientation, v_max, a_max, time_d);
               
                Eigen::Vector3d pos = ctg.get_position();
                poseStamped.pose.position.x = pos[0];
                poseStamped.pose.position.y = pos[1];
                poseStamped.pose.position.z = pos[2]; 

                double vel=ctg.get_velocity();    
                poseStamped.pose.orientation.w=vel; //TEMPORARY FOR DEBUGGING
                poseStamped.pose.orientation.y=time_d; //TEMP
                publisher.publish(poseStamped);
                i++;
                ros::spinOnce();
                getClock().sleep();
            }
            
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
            ROS_INFO("This is a trajectory generator. input pose through the file user_input.cpp ");
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
    cartesian_trajectory_generator_base ctg;
};
