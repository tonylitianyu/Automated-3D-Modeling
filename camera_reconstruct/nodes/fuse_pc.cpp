#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"

void mycallback(const sensor_msgs::PointCloud2 PCmsg)
{
    ROS_INFO("frame: %s", PCmsg.header.frame_id->data.c_str());
    // ROS_INFO("frame: %s", "im a frame");
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"fuse_pc");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("saved_pcs",1000,mycallback);
    ros::spin();
    return 0;
}