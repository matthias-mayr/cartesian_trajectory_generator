
#include <ros/ros.h>
#include <vector>
#include "cartesian_trajectory_generator_base.h"
#include "cartesian_trajectory_generator/pose.h"
#include "ros_logger/ros_logger.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/transform_listener.h"
#include <eigen_conversions/eigen_msg.h>
#include <tf_conversions/tf_eigen.h>
#include <dynamic_reconfigure/server.h>
#include <cartesian_trajectory_generator/pose_paramConfig.h>
class cartesian_trajectory_generator_ros
{
public:
    cartesian_trajectory_generator_ros()
    {
        //initialization
        topicOfThisPublisher = "/bh/CartesianImpedance_trajectory_controller/target_pose";
        poseStamped.header.frame_id = "world";
        publish_command = _n.advertise<geometry_msgs::PoseStamped>(topicOfThisPublisher, 1); // commanded poseStamped
        current_sequence = {false, false};                                                   //for dynamic reconfig

        //For logging
        latest_request_publisher = _n.advertise<geometry_msgs::PoseStamped>("/bh/CartesianImpedance_trajectory_controller/latest_request", 1);
    }

    void getInitialPose(Eigen::Vector3d &startPosition, Eigen::Quaterniond &startOrientation)
    {
        //in terminal type: rosrun tf static_transform_publisher 0.0 0.0 0.0 0.0 0.0 0.0 1.0 world ee_frame 100
        try
        {
            listenPose.waitForTransform("world", frame_name, ros::Time(0), ros::Duration(3.0));
            listenPose.lookupTransform("world", frame_name, ros::Time(0), transform);
        }
        catch (tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            return;
        }
        tf::vectorTFToEigen(transform.getOrigin(), startPosition);
        tf::quaternionTFToEigen(transform.getRotation(), startOrientation);
    }

    void publishPose()
    {
        endPosition = requested_position;
        endOrientation.coeffs() = requested_orientation.coeffs();

        //publishing latest request once
        latest_poseStamped_request.pose.position.x = requested_position[0];
        latest_poseStamped_request.pose.position.y = requested_position[1];
        latest_poseStamped_request.pose.position.z = requested_position[2];
        latest_poseStamped_request.pose.orientation.x = requested_orientation.coeffs()[0];
        latest_poseStamped_request.pose.orientation.y = requested_orientation.coeffs()[1];
        latest_poseStamped_request.pose.orientation.z = requested_orientation.coeffs()[2];
        latest_poseStamped_request.pose.orientation.w = requested_orientation.coeffs()[3];
        latest_request_publisher.publish(latest_poseStamped_request);
        //get initial pose
        Eigen::Vector3d startPosition;
        Eigen::Quaterniond startOrientation;
        getInitialPose(startPosition, startOrientation);

        ROS_INFO("Starting position:(x=%2lf,y=%2lf,z=%2lf) \t orientation:(x=%3lf,y=%3lf,z=%3lf, w=%3lf) ", startPosition[0], startPosition[1], startPosition[2], startOrientation.coeffs()[0], startOrientation.coeffs()[1], startOrientation.coeffs()[2], startOrientation.coeffs()[3]);
        bool makePlan = ctg.makePlan(startPosition, startOrientation, endPosition, endOrientation, v_max, a_max, publish_rate);
        if (!makePlan)
        {
            ROS_ERROR("Failed to generate a trajectory plan!");
            return;
        }
        else
        {
            ROS_INFO("Successfully generated a trajectory plan.");
            ROS_INFO("Estimated total time: %f", ctg.get_total_time());

            //the plan
            std::vector<Eigen::Vector3d> position_array;
            std::vector<Eigen::Vector4d> orientation_array;
            std::vector<double> time_array;
            ctg.pop_position(position_array);
            ctg.pop_time(time_array);

            //temporarly fill all values with the same orientaiton
            orientation_array.resize(position_array.size());
            std::fill(orientation_array.begin(), orientation_array.end(), startOrientation.coeffs());

            //(position_array.size(), startOrientation.coeffs());
            poseStamped.pose.orientation.x = startOrientation.coeffs()[0];
            poseStamped.pose.orientation.y = startOrientation.coeffs()[1];
            poseStamped.pose.orientation.z = startOrientation.coeffs()[2];
            poseStamped.pose.orientation.w = startOrientation.coeffs()[3];
            ROS_INFO("%i, %i, %i", position_array.size(), orientation_array.size(), time_array.size());
            if(logger.log_push_all(time_array, position_array, orientation_array)){
                ROS_INFO("Trajectory saved");
            }else{
                ROS_ERROR("Failed to save trajectory");
            }

            int i = 0;
            double tol = 0.001;
            ROS_INFO("plan:\n (x=%2lf,y=%2lf,z=%2lf)->(x=%3lf,y=%3lf,z=%3lf)\n", startPosition[0], startPosition[1], startPosition[2], endPosition[0], endPosition[1], endPosition[2]);

            while (i < position_array.size())
            {
                poseStamped.header.stamp = ros::Time::now();
                Eigen::Vector3d pos = position_array[i];
                poseStamped.pose.position.x = pos[0];
                poseStamped.pose.position.y = pos[1];
                poseStamped.pose.position.z = pos[2];
                publish_command.publish(poseStamped);
                ros::spinOnce();
                rate.sleep();
                i++;
            }

            ROS_INFO("Plan published.");
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
    void setParameters(std::string &topic_name, double &publish_rate, double &v_max, double &a_max, std::string frame_name)
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
        this->frame_name = frame_name;
        rate = ros::Rate(publish_rate);
        _n.setParam("topic_name", topic_name);
        _n.setParam("publish_rate", publish_rate);
        _n.setParam("v_max", v_max);
        _n.setParam("a_max", a_max);
        printParameters();
        Eigen::Vector3d v;
    }

    void callBackSubscriber(const geometry_msgs::PoseStampedConstPtr &msg)
    {
        ROS_INFO_STREAM(*msg);
    }

    void callbackConfig(cartesian_trajectory_generator::pose_paramConfig &config, uint32_t level)
    {
        bool temp = current_sequence[0];
        current_sequence[0] = config.ready_to_send;
        current_sequence[1] = temp;

        if (config.ready_to_send)
        {
            config.ready_to_send = false;
            requested_position[0] = config.posX;
            requested_position[1] = config.posY;
            requested_position[2] = config.posZ;
            requested_orientation.coeffs()[0] = config.orX;
            requested_orientation.coeffs()[1] = config.orY;
            requested_orientation.coeffs()[2] = config.orZ;
            requested_orientation.coeffs()[3] = config.orW;
            ROS_INFO("Request from dynamic reconfig-server recieved");
        }
    }

    void run()
    {
        logger.set_preferences(",", 1,1); //separator, print first line, overwrite

        logger.log_to(path, "commanded_trajectory.txt");
        bool reset_info = false; //needed so we don't get spammed

        //dynamic reconfiguration
        dynamic_reconfigure::Server<cartesian_trajectory_generator::pose_paramConfig> config_pose_server;
        dynamic_reconfigure::Server<cartesian_trajectory_generator::pose_paramConfig>::CallbackType f;
        f = boost::bind(&cartesian_trajectory_generator_ros::callbackConfig, this, _1, _2);
        config_pose_server.setCallback(f);
        while (_n.ok()) /*&&ros::ok())*/
        {
            if (current_sequence[0] == true)
            {
                publishPose();
                reset_info = false;
                current_sequence[0] = false;
            }
            else
            {
                if (!reset_info)
                {

                    ROS_INFO("Waiting for request");

                    reset_info = true;
                }
            }
            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    ros::NodeHandle _n;

    // Dynamic reconfigure
    Eigen::Vector3d requested_position;
    Eigen::Quaterniond requested_orientation;
    std::vector<bool> current_sequence;
    //--------------------------

    geometry_msgs::PoseStamped poseStamped;
    geometry_msgs::PoseStamped latest_poseStamped_request;
    ros::Publisher publish_command;
    ros::Subscriber subscriber;
    std::string topic_name;
    double publish_rate;
    double v_max;
    double a_max;
    cartesian_trajectory_generator_base ctg;
    Eigen::Vector3d endPosition;
    Eigen::Quaterniond endOrientation;
    ros::Rate rate = 1; //default val

    //temporary i guess
    std::string topicOfThisPublisher;
    std::string frame_name;

    //for initial pose
    tf::TransformListener listenPose;
    tf::StampedTransform transform;

    //to allow exporting data
    Logger logger;
    ros::Publisher latest_request_publisher;
    //path needs to be changed on your local machine. Choose your directory path where you want to save the logs.
    const char *path{"/home/oussama/catkin_ws/src/cartesian_trajectory_generator/generated_logs"};
};
