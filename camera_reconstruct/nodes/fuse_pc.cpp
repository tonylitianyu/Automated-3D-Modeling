#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
// #include "std_msgs/String.h"
#include "pcl_conversions/pcl_conversions.h"

class FusePC
{
    private:
    ros::Subscriber pcsub;
    ros::Publisher pcpub;
    sensor_msgs::PointCloud2 pcfused;

    public:
    FusePC(ros::NodeHandle *nh)
    {
        pcsub = nh->subscribe("saved_pcs",1000,&FusePC::fusion,this);
        pcpub = nh->advertise<sensor_msgs::PointCloud2>("/fused_pc", 10, true); 
    }
    void fusion(const sensor_msgs::PointCloud2 PCmsg)
    {
        pcl::concatenatePointCloud(pcfused, PCmsg, pcfused);
        pcpub.publish(pcfused);
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