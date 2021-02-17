
#include <ros/ros.h>

#include "cartesian_trajectory_generator_base.h"
#include "cartesian_trajectory_generator/generate_pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>

class cartesian_trajectory_generator_ros
{
public:
    cartesian_trajectory_generator_ros()
    {
        //initialization
        topicOfThisPublisher = "publish_pose";
        poseStamped.header.frame_id = "world";
        publisher = _n.advertise<geometry_msgs::PoseStamped>(topicOfThisPublisher, 1);
    }

    void getInitialPose(Eigen::Vector3d &startPosition, Eigen::Quaterniond &startOrientation)
    {
        //in terminal type: rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world ee_frame 100
        try
        {
            listenPose.waitForTransform("world",frame_name, ros::Time(0), ros::Duration(3.0));
            listenPose.lookupTransform( "world", frame_name, ros::Time(0), transform); 
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
            ros::shutdown();
        }
        tf::vectorTFToEigen(transform.getOrigin(),startPosition);
         tf::quaternionTFToEigen(transform.getRotation(),startOrientation);
    
    }

    void publishPose()
    {
        Eigen::Vector3d startPosition;
        Eigen::Quaterniond startOrientation;
        getInitialPose(startPosition, startOrientation);
        bool makePlan = ctg.makePlan(startPosition, startOrientation, endPosition, endOrientation, v_max, a_max, publish_rate);
        if (!makePlan)
        {
            ROS_ERROR("Failed to generate a trajectory plan, shutting down");
            ros::shutdown();
        }
        else
        {
            ROS_INFO("Successfully generated a trajectory plan.");
        }
        position_array = ctg.get_position();
        velocity_array = ctg.get_velocity();
        int i = 0;
        double tol = 0.001;
        ROS_INFO("plan:\n (x=%2lf,y=%2lf,z=%2lf)->(x=%3lf,y=%3lf,z=%3lf)\n", startPosition[0], startPosition[1], startPosition[2], endPosition[0], endPosition[1], endPosition[2]);
        ros::Duration(3.0).sleep();
        while (i < position_array.size())
        {
            poseStamped.header.stamp = ros::Time::now();
            Eigen::Vector3d pos = position_array[i];
            poseStamped.pose.position.x = pos[0];
            poseStamped.pose.position.y = pos[1];
            poseStamped.pose.position.z = pos[2];

            double vel = velocity_array[i];
            poseStamped.pose.orientation.w = vel; //TEMPORARY FOR DEBUGGING
            publisher.publish(poseStamped);
            i++;
            ros::spinOnce();
            rate.sleep();
        }
    }
    void printParameters()
    {
        ROS_INFO_STREAM("Subscribing to "
                        << "\"" << topic_name << "\"");
        ROS_INFO_STREAM("Frequency: " << publish_rate);
        ROS_INFO_STREAM("v_max= " << v_max);
        ROS_INFO_STREAM("a_max= " << a_max);
    }
    void setParameters(std::string &topic_name, double &publish_rate, double &v_max, double &a_max,std::string frame_name)
    {
        this->topic_name = topic_name;

        if (topic_name.compare(topicOfThisPublisher) != 0)
        {
            ROS_ERROR_STREAM("Topic \"" << topic_name << "\" does not exist, shutting down");
            ros::shutdown();
        }

        subscriber = _n.subscribe(topic_name, 1, &cartesian_trajectory_generator_ros::callBackSubscriber, this);
        this->publish_rate = publish_rate;
        this->v_max = v_max;
        this->a_max = a_max;
        this->frame_name=frame_name;
        rate = ros::Rate(publish_rate);
        _n.setParam("topic_name", topic_name);
        _n.setParam("publish_rate", publish_rate);
        _n.setParam("v_max", v_max);
        _n.setParam("a_max", a_max);
        printParameters();
    }

    void requestingPose(Eigen::Vector3d &endPosition, Eigen::Quaterniond &endOrientation)
    {
        this->endPosition = endPosition;
        this->endOrientation = endOrientation;
    }

    void callBackSubscriber(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        ROS_INFO_STREAM(*msg);
    }

    void run()
    {
        while (_n.ok())
        {
            publishPose();
            ros::spin();
            rate.sleep();
        }
    }

private:
    geometry_msgs::PoseStamped poseStamped;
    ros::NodeHandle _n;
    ros::Publisher publisher;
    ros::Subscriber subscriber;
    std::string topic_name;
    double publish_rate;
    double v_max;
    double a_max;
    cartesian_trajectory_generator_base ctg;
    Eigen::Vector3d endPosition;
    Eigen::Quaterniond endOrientation;
    ros::Rate rate = 1; //default val
    
    //the plan 
    std::vector<Eigen::Vector3d> position_array;
    std::vector<double> velocity_array;
    
    //temporary i guess
    std::string topicOfThisPublisher;
    std::string frame_name;
    //for initial pose
    tf::TransformListener listenPose;
    tf::StampedTransform transform;
};
