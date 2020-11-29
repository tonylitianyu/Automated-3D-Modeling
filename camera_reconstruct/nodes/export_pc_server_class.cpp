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

class ExportPC
{
    private:
        std::string prefix_;
        std::string filename_;
        bool binary_;
        bool compressed_;
        std::string fixed_frame_;
        tf2_ros::Buffer tf_buffer_;
        tf2_ros::TransformListener tf_listener_;
    
    protected:
        // ros::NodeHandle nh_;

    public:
        ros::NodeHandle nh_;
        string cloud_topic_;
        ros::Subscriber sub_;

        // subscriber callback
        void cloud_cb(const boost::shared_ptr<const pcl::PCLPointCloud2>& cloud)
        {
            if ((cloud->width * cloud->height) == 0)
            {
                return;
            }

            ROS_INFO("Received %d data points in frame %s with the following fields: %s",
                (int)cloud->width * cloud->height,
                cloud->header.frame_id.c_str (),
                pcl::getFieldsList (*cloud).c_str ());
            
            Eigen::Vector4f v = Eigen::Vector4f::Zero ();
            Eigen::Quaternionf q = Eigen::Quaternionf::Identity ();

            if (!fixed_frame_.empty ())
            {
                if (!tf_buffer_.canTransform (fixed_frame_, cloud->header.frame_id, pcl_conversions::fromPCL (cloud->header.stamp), ros::Duration (3.0)))
                {
                    ROS_WARN("Could not get transform :(");
                    return;
                }

                Eigen::Affine3d transform;
                transform = tf2::transformToEigen (tf_buffer_.lookupTransform (fixed_frame_, cloud->header.frame_id,  pcl_conversions::fromPCL (cloud->header.stamp)));
                v = Eigen::Vector4f::Zero ();
                v.head<3> () = transform.translation ().cast<float> ();
                q = transform.rotation ().cast<float> ();
            }

            std::stringstream ss;
            if(filename_ != "")
            {
                ss << filename_ << ".pcd";
            }
            else
            {
                ss << prefix_ << cloud->header.stamp << ".pcd";
            }

            ROS_INFO ("Data saved to %s", ss.str ().c_str ());

            // pcl::PCDWriter writer;
            // if(binary_)
            // {
            //     if(compressed_)
            //     {
            //         writer.writeBinaryCompressed (ss.str (), *cloud, v, q);
            //     }
            //     else
            //     {
            //         writer.writeBinary (ss.str (), *cloud, v, q);
            //     }
            // }
            // else
            // {
            //     writer.writeASCII (ss.str (), *cloud, v, q, 8);
            // }

        }

        // service callback
        void xpc_callback(camera_reconstruct::export_pc::Request& req);
        //, camera_reconstruct::export_pc::Response& res);

        // constructor
        ExportPC(): binary_(false), compressed_(false), tf_listener_(tf_buffer_)
        {
            // Check if a prefix parameter is defined for output file names.
            ros::NodeHandle priv_nh("~");
            if (priv_nh.getParam ("prefix", prefix_))
                {
                ROS_INFO_STREAM ("PCD file prefix is: " << prefix_);
                }
            else if (nh_.getParam ("prefix", prefix_))
                {
                ROS_WARN_STREAM ("Non-private PCD prefix parameter is DEPRECATED: "
                                << prefix_);
                }

            priv_nh.getParam ("fixed_frame", fixed_frame_);
            priv_nh.getParam ("binary", binary_);
            priv_nh.getParam ("compressed", compressed_);
            priv_nh.getParam ("filename", filename_);
            if(binary_)
            {
                if(compressed_)
                {
                ROS_INFO_STREAM ("Saving as binary compressed PCD");
                }
                else
                {
                ROS_INFO_STREAM ("Saving as binary uncompressed PCD");
                }
            }
            else
            {
                ROS_INFO_STREAM ("Saving as ASCII PCD");
            }

            if (filename_ != "")
            {
                ROS_INFO_STREAM ("Saving to fixed filename: " << filename_);
            }

            cloud_topic_ = "input";

            sub_ = nh_.subscribe (cloud_topic_, 1,  &PointCloudToPCD::cloud_cb, this);
            ROS_INFO ("Listening for incoming data on topic %s",
                        nh_.resolveName (cloud_topic_).c_str ());
        }
};

// void ExportPC::xpc_callback(camera_reconstruct::export_pc::Request& req, camera_reconstruct::export_pc& res)
// {
//     //service stuff here
//     //  see export_pc_server
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

int main(string argc, char** argv)
{
    ros::init(argc, argv, "export_pc_server", ros::init_options::AnonymousName);
    // ros::NodeHandle nh_;

    ExportPC pc;
    ros::ServiceServer ss = nh_.advertiseService("export_pc", &ExportPC::xpc_callback, &pc);

    ros::spin();

    return 0;
}