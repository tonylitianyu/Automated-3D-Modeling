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

// test
#include "camera_reconstruct/export_pc.h"

using namespace std;

// void xpc_callback(camera_reconstruct::export_pc::Request &req)
// {
//     // put shit here
//     pcl::PCDWriter writer;
//     if(binary_)
//     {
//         if(compressed_)
//         {
//             writer.writeBinaryCompressed (ss.str (), *cloud, v, q);
//         }
//         else
//         {
//             writer.writeBinary (ss.str (), *cloud, v, q);
//         }
//     }
//     else
//     {
//         writer.writeASCII (ss.str (), *cloud, v, q, 8);
//     }
// }

void xpc_callback(camera_reconstruct::export_pc::Request &req)
{
    // put shit here
    Eigen::Vector4f v = Eigen::Vector4f::Zero ();
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();

    pcl::PCDWriter writer;
    // writer.writeASCII (ss.str (), req.pc, v, q, 8);
    // writer.writeASCII ("xpc.pcd", req.pc, v, q, 8);
    writer.writeBinary ("xpc.pcd", req.pc, v, q);
}

int main(string argc, char **argv)
{
    ros::init(argc, argv, "export_pc_server");
    ros::NodeHandle nh_;

    ros::ServiceServer service = nh_.advertiseService("export_pc", xpc_callback);

    ros::spin();

    return 0;
}