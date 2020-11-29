// ROS core
#include <ros/ros.h>

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_eigen/tf2_eigen.h>

// PCL includes
#include <pcl/io/pcd_io.h>

#include <pcl_conversions/pcl_conversions.h>

// #blocked includes
#include <ros/console.h>
#include "sensor_msgs/PointCloud2.h"
// #include <pcl/filters/crop_box.h>
#include <cstdlib>
#include <std_srvs/Empty.h>

using namespace std;

int main(string argc, char **argv)
{
    ros::init(argc, argv, "export_pc_server", ros::init_options::AnonymousName);
    // if (argc != 3)
    // {
    //     ROS_INFO("usage: add_two_ints_client X Y");
    //     return 1;
    // }

    ros::NodeHandle nh_;
    ros::ServiceClient client = nh_.serviceClient<camera_reconstruct::export_pc>("export_pc");
    camera_reconstruct::export_pc srv;
    srv.request.pc = atoi(argv);
    if (client.call(srv))
    {
        ROS_INFO("Saving to blocked.ply...");
    }
    else
    {
        ROS_ERROR("Failed to call service export_pc");
        return 1;
    }

    return 0;
}