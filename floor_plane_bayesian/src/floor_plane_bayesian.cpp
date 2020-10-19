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
#include <cv_bridge/cv_bridge.h>

#include <Eigen/Core>

#include <map>


/* Bayesian estimation of the floor plane 
P(Txyk|Zxy1,...,Zxyk) = PROPTO P(Zxyk|Txyk)P(Txyk-1|Zxy1...Zxyk-1) with Txy a boolean variable on the traversability of cell xy
Zxyi boolean variables on the deduced traversability of cell xy at measurement i

Proportionnal constant = 1/(P(Zxyk|Zxy1...Zxyk-1)), it changes at every iteration

P(Zxyk|Txyk): 
                                 Zxyk:

                              F            T

                       F    alpha       1-alpha
              Txyk:                                    with alpha = 0.8 to begin with (pretty confident in the model)
                       T   1-alpha       alpha 


                                        P(Z | T = True) P(T = True)
 P(T = True|Z) =   -------------------------------------------------------------
                        P(Z | T = True)P(T = True) + P(Z | T = False)P(T = False)


P(Txyk-1 = True): At time 0, we have no clue about the prior, so P(Txyk-1 = True)=0.5
At time t=k>0: P(Txyk-1 = True) = P(Txyk|Zxy1,...,Zxyk), the previous belief
P(Txyk-1 = False) = 1-P(Txyk-1 = True) for any time t



*/


struct PointCmp 
{
	template<typename T>
	bool operator()(const T &a, const T &b) const
	{
		if (a.x == b.x)
			return a.y > b.y;

		return a.x < b.x;
	}
};

	

class FloorPlaneMapping {
    protected:
        ros::Subscriber scan_sub_;
        ros::Subscriber pos_sub_;
        ros::Subscriber speed_sub_;
        ros::Publisher map_pub_;
        tf::TransformListener listener_;


        ros::NodeHandle nh_;

        std::string base_frame_;

        typedef std::vector<pcl::PointXYZ> PointList;
        typedef std::map<cv::Point,PointList, PointCmp> ListMatrix;

        ListMatrix M;


        typedef std::vector<cv::Point> CoordList;
        CoordList Cl;

        pcl::PointCloud<pcl::PointXYZ> lastpc_;

        double alpha;
        double max_range_;
        double tolerance;
        int n_samples; 

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
                bool check true;
                for (int j = 0; j < Cl.size(); j++)
                {
                    
                    if (Cl[i] == coord)
                    {
                        check = false;
                    }
                }
                if (check == true) 
                {
                    Cl.push_back(coord);
                }

                M[coord].push_back(lastpc_[pidx[i]]);
            }

            
            cv::Mat_<uint8_t> traversability_map(100,100);
            
            //initializing traversability map
            traversability_map = 0.5;
			alpha = 0.8;
            
            for (int i = 0; i< Cl.size(); i++) 
            {
                //Ransac outputting 0 or 1

                // processing of all the cells to predict their traversability
                        float outlierRatio;
                            
                        // process Ransac
                        unsigned int nCell = M[Cl[i]].size();
                        size_t best = 0;
                        double X[3] = {1,1,1};

                        for (unsigned int i=0;i<(unsigned)n_samples;i++) 
                        {        
                            // Implement RANSAC here. Useful commands:
                            // Select a random number in in [0,i-1]

                            //Find different points
                            size_t n1 = 0;
                            size_t n2 = 0;
                            size_t n3 = 0;
                            unsigned S = 0;
                            float theta = 0;
                            float theta_max = std::atan(1); //theta max is 45 degrees (arctan(1))


                            // Create a 3D point:
                            // Eigen::Vector3f P; P << x,y,z;

                            int idx = 0;
                            while (n1 == n2 || n1 == n3 || n2 == n3)
                            
                            {
                                n1 = std::min((rand() / (double)RAND_MAX) * nCell,(double)nCell-1);
                                n2 = std::min((rand() / (double)RAND_MAX) * nCell,(double)nCell-1);
                                n3 = std::min((rand() / (double)RAND_MAX) * nCell,(double)nCell-1);
                            }


                            Eigen::Vector3f P1; P1 << L[n1].x ,L[n1].y ,L[n1].z; 
                            Eigen::Vector3f P2; P2 << L[n2].x ,L[n2].y ,L[n2].z; 
                            Eigen::Vector3f P3; P3 << L[n3].x ,L[n3].y ,L[n3].z; 

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
                            
                            Eigen::Vector3f N; N << a,b,c;  //vector normal to our predicted plane
                            theta = std::acos(c/sqrt(c*c+b*b+a*a)); //Theta is measured between the normal to predicted plane and normal to z=0 plane (0 0 1)



                            for (int i = 0; i<L.size(); i++)
                            {   
                                Eigen::Vector3f A; A << L[i].x, L[i].y, L[i].z; 
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

                        if (outlierRatio < 0.3 and theta < theta_max)
                        {
                            traversability_map[Cl[i]] = (1-alpha)*traversability_map[Cl[i]]/((1-alpha)*traversability_map[Cl[i]] + alpha*(1-traversability_map[Cl[i]]));

                        }         
                        else 
                        {
                            traversability_map[Cl[i]] = alpha*traversability_map[Cl[i]]/(alpha*traversability_map[Cl[i]] + (1-alpha)*(1-traversability_map[Cl[i]]));

                        }
                    }
        }   

        cv_bridge::CvImage br(msg->header, "mono8", traversability_map);   
        map_pub_.publish(br.toImageMsg());

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
            map_pub_ = nh_.advertise<sensor_msgs::Image>("/traversability_map", 1);
        }

		      


};

int main(int argc, char * argv[]) 
{
    ros::init(argc,argv,"floor_plane_mapping");
    FloorPlaneMapping fp;
    
    ros::spin();
    return 0;
}


