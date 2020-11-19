#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/String.h"

class FusePC
{
    private:
    ros::Subscriber pcsub;

    public:
    pcCollect(ros::NodeHandle *nh)
    {
        pcsub = nh->subscribe("saved_pcs",1000,FusePC::mycallback,this);
    }
    void mycallback(const sensor_msgs::PointCloud2 PCmsg)
    {
        ROS_INFO_STREAM(PCmsg.header);
    }
};


int main(int argc, char **argv)
{
    ros::init(argc,argv,"fuse_pc");
    ros::NodeHandle nh;
    FusePC fpc = FusePC(&nh);
    ros::spin();
    return 0;
}