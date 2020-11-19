#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

void mycallback(const sensor_msgs::PointCloud2 PCmsg)
{
    ROS_INFO_STREAM(PCmsg.header);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"fuse_pc");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("saved_pcs",1000,mycallback);
    ros::spin();
    return 0;
}