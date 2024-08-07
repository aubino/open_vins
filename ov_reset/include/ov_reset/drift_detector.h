#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl-1.10/pcl/common/common.h>
#include <nav_msgs/Odometry.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Core>
#include <ov_reset/RestartOv.h>
#include <std_msgs/Float64.h>
#include <cmath>
#include <deque>

/** Cofidece is split into two parts . One part is deduced from the number of slam points and the other from the number of slam points.
 * Slam points are responsible for 60 percent of the score while tracing points are responsible for 40 percent of it.
 * The score is computed by s = b + tanh(ax) for each of these components
 */

#define SLAM_A  0.08  // The lower the pickier the slam score to go to it's maximum. Meaning the number of slam points needed to reach SLAM_B percent is higher
#define SLAM_B  70 // Weight of the slam score in the total score
#define TRACK_A 0.05
#define TRACK_B 30


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
        ros::ServiceClient reset_client ;   
        bool publish_conf , do_restart ; 
        float track_timeout , slam_timeout , cov_to_dist_max ; 
        uint16_t min_track_points , min_track_points ; 
        std::atomic<double> slam_conf  , track_conf , cov_conf , conf;
        std::string restart_cfg ;             
} ; 

// Function to extract standard deviations from odometry covariance
std::pair<double, double> extract_std_dev(const nav_msgs::Odometry& odom) {
    // Convert the covariance array to an Eigen matrix
    Eigen::Matrix<double, 6, 6> covariance_matrix;
    for (int i = 0; i < 6; ++i) {
        for (int j = 0; j < 6; ++j) {
            covariance_matrix(i, j) = odom.pose.covariance[i * 6 + j];
        }
    }

    // Extract the 3x3 position covariance matrix
    Eigen::Matrix3d position_covariance = covariance_matrix.block<3,3>(0, 0);

    // Extract the 3x3 orientation covariance matrix
    Eigen::Matrix3d orientation_covariance = covariance_matrix.block<3,3>(3, 3);

    // Compute the eigenvalues for position covariance
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> position_solver(position_covariance);
    Eigen::Vector3d position_eigenvalues = position_solver.eigenvalues();

    // Compute the eigenvalues for orientation covariance
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> orientation_solver(orientation_covariance);
    Eigen::Vector3d orientation_eigenvalues = orientation_solver.eigenvalues();

    // Compute the standard deviations as the maximum square root of eigenvalues
    double position_std_dev = position_eigenvalues.array().sqrt().maxCoeff();
    double orientation_std_dev = orientation_eigenvalues.array().sqrt().maxCoeff();

    // Return the standard deviations as a pair
    return std::make_pair(position_std_dev, orientation_std_dev);
}

// Function to compute the hyperbolic tangent of x
double tanh(double x) {
    // Using the definition of tanh: tanh(x) = (e^x - e^-x) / (e^x + e^-x)
    double e_pos = std::exp(x);  // e^x
    double e_neg = std::exp(-x); // e^-x
    return (e_pos - e_neg) / (e_pos + e_neg);
}
