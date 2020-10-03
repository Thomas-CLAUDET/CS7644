#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#define OPENCV_TRAITS_ENABLE_DEPRECATED
#include <opencv2/opencv.hpp>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#include <Eigen/Core>


class FloorPlaneMapping {
    protected:
        ros::Subscriber scan_sub_;
        tf::TransformListener listener_;


        ros::NodeHandle nh_;

        ros::Publisher mapping_pub = nh_.advertise<cv::Mat_<uint32_t>>("map_topic", 1000);

        std::string base_frame_;
        double max_range_;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

        int dims[2] = {101,101};
        mapping = cv::Mat_<uint32_t>(2,dims);    

    protected: // ROS Callbacks

        void pc_callback(const sensor_msgs::PointCloud2ConstPtr msg) {
            pcl::PointCloud<pcl::PointXYZ> temp;
            pcl::fromROSMsg(*msg, temp);
            // Make sure the point cloud is in the base-frame
            listener_.waitForTransform(base_frame_,msg->header.frame_id,msg->header.stamp,ros::Duration(1.0));
            pcl_ros::transformPointCloud(base_frame_,msg->header.stamp, temp, msg->header.frame_id, lastpc_, listener_);

            //
            unsigned int n = temp.size();
            std::vector<size_t> pidx;
            // First count the useful points
            for (unsigned int i=0;i<n;i++) {
                float x = temp[i].x;
                float y = temp[i].y;
                float d = hypot(x,y);
                if (d < 1e-2) {
                    // Bogus point, ignore
                    continue;
                }
                x = lastpc_[i].x;
                y = lastpc_[i].y;
                d = hypot(x,y);
                if (d > max_range_) {
                    // too far, ignore
                    continue;
                }
                pidx.push_back(i);
            }
            
            //
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c using the hough transform
            // Remember to use the a_min,a_max,n_a variables (resp. b, c).
            n = pidx.size();
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());
            // fill the accumulator with zeros
            mapping = -1;


            for (unsigned int i=0;i<n;i++) {
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;
                double z = lastpc_[pidx[i]].z;
                // Update the accumulator based on current point here
                // individual cells in the accumulator can be accessed as follows
                if (z != 0) {
                    mapping(50 - round(y*10), 50 + round(x*10)) = 0;
                }
                else{
                    mapping(50 - round(y*10), 50 + round(x*10)) = 1;
                }
            }
            
            mapping_pub.publish(mapping);
        }     

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneMapping fp;
    ros::spin();
    return 0;
}


