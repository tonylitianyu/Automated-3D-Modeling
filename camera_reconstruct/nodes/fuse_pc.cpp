#include "ros/ros.h"
#include <ros/console.h>
#include "sensor_msgs/PointCloud2.h"
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/filters/crop_box.h>


class FusePC
/*
gets saved point clouds, fuses point clouds, crops fused point cloud publishes fused point cloud
Subscribers:    /saved_pcs - most recent saved PointCloud2 message
Publisher:      /fused_pc - concatenated PointCloud2 message
*/
{
    private:
        ros::Subscriber pcsub;
        ros::Publisher pcpub;
        ros::Publisher pcpub2;
        sensor_msgs::PointCloud2 pcfused;
        float minX = -0.25;
        float minY = -0.25;
        float minZ = -0.25;
        float maxX = 0.25;
        float maxY = 0.25;
        float maxZ = 0.25;
        int count = 0;

        pcl::PCLPointCloud2 *front; 

    public:
        FusePC(ros::NodeHandle *nh)
        /*
        constructor of FusePC class
        initializes subscriber and publisher
        */
        {
            pcsub = nh->subscribe("/saved_pcs",1000,&FusePC::fusion,this);
            pcpub = nh->advertise<sensor_msgs::PointCloud2>("/fused_pc", 10, true);
            pcpub2 = nh->advertise<sensor_msgs::PointCloud2>("/final_pc", 10, true); 
        }

        void fusion(const sensor_msgs::PointCloud2 PCmsg)
        /*
        subsciber callback function
        fuses PointCloud2 messages published to saved_pcs topic
        crops fused PointCloud2 message
        publishes fused PointCloud2 message to /fused_pc
        */
        {

            pcl::concatenatePointCloud(pcfused, PCmsg, pcfused);

            pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
            pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
            pcl::PCLPointCloud2 cloud_filtered;

            pcl_conversions::toPCL(pcfused, *cloud);
            pcl::CropBox<pcl::PCLPointCloud2> cropFilter;
            cropFilter.setInputCloud(cloudPtr);
            cropFilter.setMin(Eigen::Vector4f(minX,minY,minZ,1.0));
            cropFilter.setMax(Eigen::Vector4f(maxX,maxY,maxZ,1.0));

            cropFilter.filter(cloud_filtered);
            sensor_msgs::PointCloud2 output;
            pcl_conversions::fromPCL(cloud_filtered,output);

            count += 1;

            pcpub.publish(output);

            if(count == 5)
            {
                pcpub2.publish(output);

                count = 0;
                pcfused = sensor_msgs::PointCloud2();
            }
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