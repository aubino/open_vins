#include "ov_reset/drift_detector.h"

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


void OvDriftDetector::process_track_points(const sensor_msgs::PointCloud::ConstPtr& cloud_msg)
{
    // Create a PCL point cloud object
    pcl::PointCloud<pcl::PointXYZ> pcl_cloud;

    // Iterate over each point in the sensor_msgs::PointCloud
    for (size_t i = 0; i < cloud_msg->points.size(); ++i) {
        pcl::PointXYZ point;
        point.x = cloud_msg->points[i].x;
        point.y = cloud_msg->points[i].y;
        point.z = cloud_msg->points[i].z;
        pcl_cloud.points.push_back(point);
    }

    // Set the width and height of the PCL cloud
    pcl_cloud.width = pcl_cloud.points.size();
    pcl_cloud.height = 1; // Unordered point cloud
    pcl_cloud.is_dense = false;
    pcl_cloud.header.stamp = ros::Time::now().toNSec() ; //cloud_msg->header.stamp.toNSec() ; 
    track_points_buffer.push_back(pcl_cloud) ;
    ROS_INFO("Tracking points size %d", pcl_cloud.size()) ;  
    while (!track_points_buffer.empty() && (ros::Time::now().toNSec() - track_points_buffer.front().header.stamp) > track_timeout * 1e9) {
            track_points_buffer.pop_front();
        }
    int res = 0  ; 
    for(const auto cloud :  track_points_buffer)
        if(cloud.size() > min_track_points) 
            res ++ ;  
    if(res == 0)
    {
        // Tracking point has been low for the past track_timeout senconds. Trigger reset
        ROS_INFO("Tracking points Lower than %d for the last %f seconds. ", min_track_points , track_timeout) ; 
        if(do_restart)
        {
            ROS_INFO("Triggering reset") ;
            ov_reset::RestartOv req_data ; 
            req_data.request.estimator_config_path = restart_cfg ; 
            req_data.request.timeout = -1 ; 
            if (reset_client.call(req_data)) {
                ROS_INFO("Successfully called OvReset service");
            } else {
                ROS_ERROR("Failed to call OvReset service");
            } 
        }
        track_conf = 0.0 ; 
    }
    else track_conf = TRACK_B * tanh(TRACK_A * pcl_cloud.size()) ;
    ROS_INFO("Tracking confidance : %f",track_conf.load(std::memory_order::memory_order_acquire)) ;  
}

void OvDriftDetector::process_slam_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> output ; 
    pcl::fromPCLPointCloud2(pcl_pc2, output);
    ROS_INFO("Slam points size %d", output.size()) ;
    output.header.stamp = msg->header.stamp.toNSec() ; 
    slam_points_buffer.push_back(output) ;

    while (!slam_points_buffer.empty() && (ros::Time::now().toNSec() - slam_points_buffer.front().header.stamp) > slam_timeout * 1e9) {
            slam_points_buffer.pop_front();
        }
    int res = 0  ; 
    for(const auto cloud :  slam_points_buffer)
        if(cloud.size() > min_slam_points) 
            res ++ ;  
    if(res == 0)
    {
        // Tracking point has been low for the past track_timeout senconds. Trigger reset
        ROS_INFO("Slam points Lower than %d for the last %f seconds. ", min_track_points , track_timeout) ; 
        if(do_restart)
        {
            ROS_INFO("Triggering reset") ;
            ov_reset::RestartOv req_data ; 
            req_data.request.estimator_config_path = restart_cfg ; 
            req_data.request.timeout = -1 ; 
            if (reset_client.call(req_data)) {
                ROS_INFO("Successfully called OvReset service");
            } else {
                ROS_ERROR("Failed to call OvReset service");
            }
             
        }
        slam_conf = 0.0 ; 
    }
    else slam_conf = SLAM_B * tanh(SLAM_A * output.size()) ;
    ROS_INFO("Slam confidance : %f",slam_conf.load(std::memory_order::memory_order_acquire)) ; 
}

void OvDriftDetector::process_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    auto std_dev = extract_std_dev(*msg) ; 
    double dist = std::sqrt( msg->pose.pose.position.x * msg->pose.pose.position.x + 
                             msg->pose.pose.position.y * msg->pose.pose.position.y + 
                             msg->pose.pose.position.z * msg->pose.pose.position.z) ; 
    if((std_dev.first/dist) > cov_to_dist_max) 
    {
        ROS_INFO("Normalized covariance greater than %f . Decreasin confidance slightly if needed",cov_to_dist_max) ; 
        cov_conf = 0.0 ; 
    }
    if(publish_conf)
    {
        std_msgs::Float64 conf_to_publish ; 
        conf_to_publish.data = slam_conf + track_conf ; 
        conf_pub.publish(conf_to_publish) ; 
    }
}

OvDriftDetector::OvDriftDetector(
    ros::NodeHandle nh ,  
    std::string track_points_topic , 
    std::string slam_points_topic , 
    std::string odom_topic, 
    std::string restart_cfg , 
    bool do_restart  , 
    bool publish_conf ,  
    uint16_t min_track_points , 
    uint16_t min_slam_points  , 
    float track_timeout, 
    float slam_timeout , 
    float cov_to_dist_max ) : 
    publish_conf(publish_conf) , 
    do_restart(do_restart) , 
    track_timeout(track_timeout) ,  
    slam_timeout(slam_timeout) , 
    cov_to_dist_max(cov_to_dist_max) , 
    min_track_points(min_track_points) , 
    min_slam_points(min_slam_points) , 
    restart_cfg(restart_cfg) , 
    slam_conf(0) , 
    track_conf(0),
    cov_conf(0) , 
    conf(0)
{
    tracker_sub = nh.subscribe(track_points_topic,1,&OvDriftDetector::process_track_points,this) ; 
    slam_sub = nh.subscribe(slam_points_topic,1,&OvDriftDetector::process_slam_points,this) ;
    odom_sub = nh.subscribe(odom_topic,1,&OvDriftDetector::process_odom,this) ;
    if(publish_conf)
        conf_pub = nh.advertise<std_msgs::Float64>("tracker_confidence",10) ; 
    if(do_restart)
        reset_client = nh.serviceClient<ov_reset::RestartOv>("/ov_msckf/reset_ov") ; 
    
}