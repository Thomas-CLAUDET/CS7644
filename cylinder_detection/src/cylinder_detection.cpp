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


class CylinderDectection {
    protected:
        ros::Subscriber scan_sub_;
        ros::Publisher marker_pub_;
        tf::TransformListener listener_;

        ros::NodeHandle nh_;
        std::string base_frame_;
        double max_range_;

        typedef std::vector<pcl::PointXYZ> PointList;
        PointList L;

        typedef std::vector<Eigen::Vector3f> CircleParamList;
        CircleParamList C;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;
        cv::Mat_<uint32_t> accumulator;

        int n_a, n_b, n_r;
        double a_min, a_max, b_min, b_max, r_min, r_max;
        double eps = 0.3;

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

                if (temp[i].z > eps) {
                    L.push_back(temp[i]);
                }
                //watch out here, may write for(if temp(i) not in L)
            }
            

            //
            // BEGIN TODO
            // Finding planes: z = a*x + b*y + c using the hough transform
            // Remember to use the a_min,a_max,n_a variables (resp. b, c).
            n = L.size();
            // fill the accumulator with zeros
            accumulator = 0;

            float a = 0.0;
            float b = 0.0;
            float r = 0.0;
            
            int l = 0;

            for (unsigned int i=0;i<n;i++) {
                double x = L[i].x;
                double y = L[i].y;
                double z = L[i].z;
                // Update the accumulator based on current point here
                // individual cells in the accumulator can be accessed as follows
                for (int j=0;j<=n_a-1;j++){
                    a = a_min + j*((a_max-a_min)/(n_a-1));

                    for (int k=0; k<=n_b-1 ;k++){
                        
                        b = b_min + k*((b_max-b_min)/(n_b-1));

                        r = sqrt(pow(x-a,2) + pow(y-b,2));

                        if (r_min <= r && r <= r_max){

                            l = int(n_r*(r-r_min)/(r_max-r_min));
                            accumulator(j,k,l)+=1;
                        }
                    }
                }
            }


            unsigned int T = 0;


            for (int j=0;j<n_a;j++){
                for (int k=0;k<n_b;k++){
                    for (int l=0;l<n_r;l++){
                        if (accumulator(j,k,l)>T){
                            T = accumulator(j,k,l);
                        }
                    }
                }
            }
            
            

            for (int j=0;j<n_a;j++){
                for (int k=0;k<n_b;k++){
                    for (int l=0;l<n_r;l++){
                        if (accumulator(j,k,l)>T/2){
                            a = a_min + j*((a_max-a_min)/(n_a-1));
                            b = b_min + k*((b_max-b_min)/(n_b-1));
                            r = r_min + l*((r_max-r_min)/(n_r-1));
                            Eigen::Vector3f circle;
                            circle << a,b,r; 
                            C.push_back(circle);
                        }
                    }
                }
            }
            
            // Use the accumulator to find the best plane parameters and store
            // them in X (this will be used for display later)
            // X = {a,b,c}

            // END OF TODO
            ROS_INFO("Extracted circle: Center = (%.2f, %.2f), Radius = %.2f",C[0][0], C[0][1], C[0][2]);

            Eigen::Vector3f O,u,v,w;
            w << X[0], X[1], -1.0;
            w /= w.norm();
            O << 1.0, 0.0, 1.0*X[0]+0.0*X[1]+X[2];
            u << 2.0, 0.0, 2.0*X[0]+0.0*X[1]+X[2];
            u -= O;
            u /= u.norm();
            v = w.cross(u);

            tf::Matrix3x3 R(u(0),v(0),w(0),
                    u(1),v(1),w(1),
                    u(2),v(2),w(2));
            tf::Quaternion Q;
            R.getRotation(Q);
            

            // Finally publish the marker
            for (int i = 0; i < C.size() ; i++)    
            {
                // Documentation on visualization markers can be found on:
                // http://www.ros.org/wiki/rviz/DisplayTypes/Marker
                visualization_msgs::Marker m;
                m.header.stamp = msg->header.stamp;
                m.header.frame_id = base_frame_;
                m.ns = "floor_plane";
                m.id = 1;
                m.type = visualization_msgs::Marker::CYLINDER;
                m.action = visualization_msgs::Marker::ADD;
                m.pose.position.x = C[i][0];
                m.pose.position.y = C[i][1];
                m.pose.position.z = 0;
                tf::quaternionTFToMsg(Q,m.pose.orientation);
                m.scale.x = C[i][2];
                m.scale.y = C[i][2];
                m.scale.z = 5;
                m.color.a = 0.5;
                m.color.r = 1.0;
                m.color.g = 0.0;
                m.color.b = 1.0;
                marker_pub_.publish(m);
            }

            
        }

    public:
        CylinderDectection() : nh_("~") {
            nh_.param("base_frame",base_frame_,std::string("/body"));
            nh_.param("max_range",max_range_,5.0);
            nh_.param("n_a",n_a,100);
            nh_.param("a_min",a_min,-5.0);
            nh_.param("a_max",a_max,5.0);
            nh_.param("n_b",n_b,100);
            nh_.param("b_min",b_min,-5.0);
            nh_.param("b_max",b_max,+5.0);
            nh_.param("n_r",n_r,3);
            nh_.param("r_min",r_min,10.0);
            nh_.param("r_max",r_max,15.0);

            assert(n_a > 0);
            assert(n_b > 0);
            assert(n_r > 0);

            // the accumulator is created here as a 3D matrix of size n_a x n_b x n_c
            int dims[3] = {n_a,n_b,n_r};
            accumulator = cv::Mat_<uint32_t>(3,dims);


            // Make sure TF is ready
            ros::Duration(0.5).sleep();

            scan_sub_ = nh_.subscribe("scans",1,&CylinderDectection::pc_callback,this);
            //marker_pub_ = nh_.advertise<visualization_msgs::Marker>("cylinder",1);

        }

};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"cylinder_detection");
    CylinderDectection fp;

    ros::spin();
    return 0;
}


