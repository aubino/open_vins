#include "ov_reset/drift_detector.h"
#include "ros/ros.h"
const std::string default_track_points_topic = "/ov_msckf/track_points" ; 
const std::string default_slam_points_topic  = "/ov_msckf/slam_points"; 
const std::string default_odom_topic =  "/ov_msckf/odomimu"; 
const std::string default_restart_cfg = "/home/aubin/estimator_config.yaml" ;
bool default_do_restart  = true ; 
bool default_publish_conf  = true ;  
uint16_t default_min_track_points = 20 ; 
uint16_t default_min_slam_points = 8 ; 
float default_track_timeout = 1.0 ; 
float default_slam_timeout = 3.0 ; 
float default_cov_to_dist_max = 0.025 ; 


int main(int argc , char** argv)
{
    std::string track_points_topic ; 
    std::string slam_points_topic ; 
    std::string odom_topic; 
    std::string restart_cfg ;
    bool do_restart  ; 
    bool publish_conf ;  
    int min_track_points ; 
    int min_slam_points  ; 
    float track_timeout ; 
    float slam_timeout ;
    float cov_to_dist_max ;
    ros::init( argc ,  argv , "drift_detector_node") ; 
    ros::NodeHandle nh("~") ; 
    if(!nh.getParam("track_points_topic",track_points_topic)) { track_points_topic = default_track_points_topic ; }
    if(!nh.getParam("slam_points_topic",slam_points_topic)) { slam_points_topic = default_slam_points_topic ; }
    if(!nh.getParam("odom_topic",odom_topic)) { odom_topic = default_odom_topic ; }
    if(!nh.getParam("restart_cfg",restart_cfg)) { restart_cfg = default_restart_cfg ; }
    if(!nh.getParam("do_restart",do_restart)) { do_restart = default_do_restart ; }  
    if(!nh.getParam("publish_conf" , publish_conf)) { publish_conf = default_publish_conf ; }
    if(!nh.getParam("min_track_points",min_track_points)) { min_track_points = default_min_track_points ; }
    if(!nh.getParam("min_slam_points" , min_slam_points)) { min_slam_points = default_min_slam_points ; } 
    if(!nh.getParam("track_timeout",track_timeout)) { track_timeout = default_track_timeout ; }
    if(!nh.getParam("slam_timeout",slam_timeout)) { slam_timeout = default_slam_timeout ; }
    if(!nh.getParam("cov_to_dist_max",cov_to_dist_max)) { cov_to_dist_max = default_cov_to_dist_max ; }
    OvDriftDetector detector(nh,
                            track_points_topic,
                            slam_points_topic,
                            odom_topic,
                            restart_cfg,
                            do_restart,
                            publish_conf,
                            min_track_points,
                            min_slam_points,
                            track_timeout,
                            slam_timeout,
                            cov_to_dist_max) ; 
    ros::spin() ; 
}