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
        
    }
}