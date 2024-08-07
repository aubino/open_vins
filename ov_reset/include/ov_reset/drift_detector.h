#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl-1.10/pcl/common/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <deque>

class OvDriftDetector 
{
    public : 
        OvDriftDetector(ros::NodeHandle nh ,  std::string track_points_topic , std::string slam_points_topic 
                        , std::string odom_topic, std::string restart_cfg , bool do_restart = false , 
                         bool publish_conf = false ,  uint16_t min_track_points = 40, uint16_t min_slam_points = 8 , 
                        float track_timeout=3.0, float slam_timeout=3.0 , float cov_to_dist_max = 50.0 ) ;
        void process_track_points(const sensor_msgs::PointCloud2::ConstPtr& msg) ;
        void process_slam_points(const sensor_msgs::PointCloud2::ConstPtr& msg) ; 
        void process_odom(const nav_msgs::Odometry::ConstPtr& msg) ; 
    
    private : 
        std::deque<pcl::PointCloud<pcl::PointXYZ>> slam_points_buffer ; 
        std::deque<pcl::PointCloud<pcl::PointXYZ>> track_points_buffer ;
        std::deque<double> positio_cov_buffer ; 
        std::deque<double> orientation_cov_buffer ; 
        std::deque<double> speed_cov_buffer ;
        ros::Subscriber tracker_sub , slam_sub, odom_sub ; 
        ros::Publisher conf_pub ;  
        bool publish_conf ; 
        float track_timeout , slam_timeout , cov_to_dist_max ; 
        uint16_t min_track_points , min_track_points ;           
} ; 