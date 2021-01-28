#include "ros/ros.h"

#include "geometry_msgs/PoseStamped.h"

void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
   {
    msg->pose.position.x,msg->pose.position.y,msg->pose.position.z;
msg->pose.orientation.x,msg->pose.orientation.y,msg->pose.orientation.z,msg->pose.orientation.w;
  
ROS_INFO_STREAM("Recieved: \n" << msg);

   }

int main(int argc, char **argv) {
    
	ros::init(argc, argv, "listener");
	ros::NodeHandle _n;
	ros::Subscriber sub = _n.subscribe("channel1", 1000, poseCallback);

ros::spin();
 return 0;
}

