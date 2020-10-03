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


#include <geometry_msgs/Twist.h>
#include <tf2_msgs/TFMessage.h>
#include <map>

//OBj2 : Détection de cercle sur le pointcloud de la cellule

class FloorPlaneMapping {
    protected:
        ros::Subscriber scan_sub_;
        ros::Subscriber pos_sub_;
        ros::Subscriber speed_sub_;
        ros::Publisher map_pub_;
        tf::TransformListener listener_;


        ros::NodeHandle nh_;

        std::string base_frame_;

        typedef std::list<pcl::PointXYZ> PointList;
        typedef std::map<cv::Point,PointList> ListMatrix;
        ListMatrix M;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;


        double max_range_;
        double tolerance;
        int n_samples; 
        double eps;
        double last_node[2];

        tf2_msgs::TFMessage posRobot;
        geometry_msgs::Twist vRobot;

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
                pidx.push_back(i);
            }
        
            n = pidx.size();
            ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());


            for (unsigned int i=0;i<n;i++) {
                double x = lastpc_[pidx[i]].x;
                double y = lastpc_[pidx[i]].y;

                cv::Point coord(floor((x+5)*10),floor((y+5)*10));
                pth_back(lastpc_[pidx[i]]);
            }

            eps = 0.5;
            last_node[0] = -2.9250;
            last_node[1] = 2.2000; //found in vrep
            
            cv::Mat_<uint8_t> traversability_map(100,100);
            traversability_map = -1;
            if ((hypot(posRobot.transforms.translation.x - last_node[0], posRobot.transforms.translation.y - last_node[1]) < eps) && (hypot(vRobot.linear.x, vRobot.linear.y) < eps))
            {
                for (ListMatrix::const_iterator it = M.begin(); it!=M.end(); it++) 
                {
                    float outlierRatio;
                    int traversable;
                    cv::Point coord = it -> first;
                    const PointList &L = it -> second;
                        
                    // process Ransac
                    unsigned int nCell = pte();
                    size_t best = 0;
                    double X[3] = {1,1,1};
                    ROS_INFO("%d useful points out of %d",(int)n,(int)temp.size());

                    for (unsigned int i=0;i<(unsigned)n_samples;i++) 
                    {        
                        // Implement RANSAC here. Useful commands:
                        // Select a random number in in [0,i-1]

                        //Find different points
                        size_t n1 = 0;
                        size_t n2 = 0;
                        size_t n3 = 0;
                        unsigned S = 0;


                        // Create a 3D point:
                        // Eigen::Vector3f P; P << x,y,z;

                        int idx = 0;
                        n1 = std::min((rand() / (double)RAND_MAX) * nCell,(double)nCell-1);
                        n2 = std::min((rand() / (double)RAND_MAX) * nCell,(double)nCell-1);
                        n3 = std::min((rand() / (double)RAND_MAX) * nCell,(double)nCell-1);
                        
                        for (ListMatrix::const_iterator pt = L.begin(); pt!=L.end(); pt++) 
                		{	
                			if(idx==n1){
                				Eigen::Vector3f P1; P1 << pt.x, pt.y, pt.z; 
                			}
                        	if(idx==n2){
                				Eigen::Vector3f P2; P2 << pt.x, pt.y, pt.z; 
                			}
                			if(idx==n13){
                				Eigen::Vector3f P3; P3 << pt.x, pt.y, pt.z; 
                			}
                        	
                        }

                        // Plane equation
                        float a1 = P2[0] - P1[0]; 
                        float b1 = P2[1] - P1[1]; 
                        float c1 = P2[2] - P1[2]; 
                        float a2 = P3[0] - P1[0]; 
                        float b2 = P3[1] - P1[1]; 
                        float c2 = P3[2] - P1[2]; 
                        float a = b1 * c2 - b2 * c1; 
                        float b = a2 * c1 - a1 * c2; 
                        float c = a1 * b2 - b1 * a2; 
                        float d = (- a * P1[0] - b * P1[1] - c * P1[2]); 
                        Eigen::Vector3f N; N << a,b,c;


                        for (ListMatrix::const_iterator pt = L.begin(); pt!=L.end(); pt++)
                        {   
                            Eigen::Vector3f A; A << pt.x, pt.y, pt.z; 
                            Eigen::Vector3f M; M << P3[0],P3[1],P3[2] ;
                            Eigen::Vector3f MA; MA << A[0]-M[0], A[1]-M[1], A[2]-M[2];
                            double distance =   abs(MA.dot(N))/N.norm();
                            if (distance < tolerance)
                            {
                                S++;
                            }
                        }
                        if (S>best){
                        best = S;
                        std::cout<<S<<std::endl;
                        X[0] = -a;
                        X[1] = -b;
                        X[2] = -d;

                        float outlierRatio = (nCell - S)/nCell;
                    }

                    if (outlierRatio < 0.3)
                    {
                        traversable = 0;
                    }         
                    else 
                    {
                        traversable = 1;
                    }
                    
                    traversability_map(coord.y, coord.x) = traversable;
                }
            }
            map_pub_.publish(traversability_map);
        }   

        void pos_callback(const tf2_msgs::TFMessageConstPtr msg)
        {
            tf2_msgs::TFMessage temp;
            pcl::fromROSMsg(*msg, posRobot); 
        }

        void speed_callback(const geometry_msgs::TwistConstPtr msg)
        {
            geometry_msgs::vector3 vitesse;
            pcl::fromROSMsg(*msg, vRobot); 
        }

    public:

        FloorPlaneMapping() : nh_("~")
        {
            nh_.param("base_frame",base_frame_,std::string("/bubbleRob"));
            nh_.param("max_range",max_range_,2.0);
            nh_.param("n_samples",n_samples,100);
            nh_.param("tolerance",tolerance,0.5);

            ROS_INFO("Searching for Plane parameter z = a x + b y + c");
            ROS_INFO("RANSAC: %d iteration with %f tolerance",n_samples,tolerance);
            assert(n_samples > 0);

            // Make sure TF is ready
            ros::Duration(0.5).sleep();
            scan_sub_ = nh_.subscribe("scans",1,&FloorPlaneMapping::pc_callback,this);  
            pos_sub_ = nh_.subscribe("/tf",1,&FloorPlaneMapping::pos_callback,this);  
            speed_sub_ = nh_.subscribe("/vrep/twistStatus",1,&FloorPlaneMapping::speed_callback,this); 
            map_pub_ = nh_.advertise<cv::Mat_<uint8_t>>("map",1); 
        }


};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneMapping fp;
    
    ros::spin();
    return 0;
}


