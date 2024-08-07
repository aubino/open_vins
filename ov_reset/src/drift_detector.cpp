#include "drift_detector.h"


void OvDriftDetector::process_track_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> output ; 
    pcl::fromPCLPointCloud2(pcl_pc2, output);
    track_points_buffer.push_back(output) ; 
    while (!track_points_buffer.empty() && (ros::Time::now().toNSec() - track_points_buffer.front().header.stamp) > track_timeout * 1e9) {
            track_points_buffer.pop_front();
        }
    uint res = 0  ; 
    for(const auto cloud :  track_points_buffer)
        res += cloud.size() > min_track_points ; 
    if(res == 0)
    {
        // Tracking point has been low for the past track_timeout senconds. Trigger reset
        ROS_INFO("Tracking points Lower than %d for the last %d seconds. ", min_track_points , track_timeout) ; 
        if(do_restart)
        {
            ROS_INFO("Triggering reset") ; 
        }
        track_conf = 0.0 ; 
    }
    else track_conf = TRACK_B * tanh(TRACK_A * output.size()) ; 
}

void OvDriftDetector::process_slam_points(const sensor_msgs::PointCloud2::ConstPtr& msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::PointCloud<pcl::PointXYZ> output ; 
    pcl::fromPCLPointCloud2(pcl_pc2, output);
    slam_points_buffer.push_back(output) ; 
    while (!slam_points_buffer.empty() && (ros::Time::now().toNSec() - slam_points_buffer.front().header.stamp) > track_timeout * 1e9) {
            slam_points_buffer.pop_front();
        }
    uint res = 0  ; 
    for(const auto cloud :  slam_points_buffer)
        res += cloud.size() > min_slam_points ; 
    if(res == 0)
    {
        // Tracking point has been low for the past track_timeout senconds. Trigger reset
        ROS_INFO("Slam points Lower than %d for the last %d seconds. ", min_track_points , track_timeout) ; 
        if(do_restart)
        {
            ROS_INFO("Triggering reset") ; 
        }
        slam_conf = 0.0 ; 
    }
    else slam_conf = SLAM_B * tanh(SLAM_A * output.size()) ; 
}

void OvDriftDetector::process_odom(const nav_msgs::Odometry::ConstPtr& msg)
{
    auto std_dev = extract_std_dev(*msg) ; 
    double dist = std::sqrt( msg->pose.pose.position.x * msg->pose.pose.position.x + 
                             msg->pose.pose.position.y * msg->pose.pose.position.y + 
                             msg->pose.pose.position.z * msg->pose.pose.position.z) ; 
    if((std_dev.first/dist) > cov_to_dist_max) 
    {
        ROS_INFO("Normalized covariance greater than %f . Decreasin confidance sligntly if needed",cov_to_dist_max) ; 
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
    bool do_restart = false , 
    bool publish_conf = false ,  
    uint16_t min_track_points = 40, 
    uint16_t min_slam_points = 8 , 
    float track_timeout=3.0, 
    float slam_timeout=3.0 , 
    float cov_to_dist_max = 50.0) : 
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
        conf_pub = nh.advertise("tracker_confidence",10) ; 
    if(do_restart)
        reset_client = nh.serviceClient<ov_reset::RestartOv>("/ov_msckf/reset_ov") ; 
    
}