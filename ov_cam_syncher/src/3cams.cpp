#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <string.h>
#include <ov_dai/syncCameras.h>
#include <memory>

typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, sensor_msgs::Image> sync_pol;
std::vector<std::shared_ptr<message_filters::Synchronizer<sync_pol>>> sync_cam;
std::vector<std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>>> sync_subs_cam;
std::vector<ros::Subscriber> subs_cam;
ros::Publisher sync_publisher ; 

void triple_cam_callback(const sensor_msgs::ImageConstPtr &msg0, const sensor_msgs::ImageConstPtr &msg1 , const sensor_msgs::ImageConstPtr& msg2)
{
    ov_dai::syncCameras sync_msg ; 
    std_msgs::UInt8 ind ;  
    sync_msg.header.frame_id = "synced" ; 
    sync_msg.header.stamp = msg0->header.stamp ;
    sync_msg.images.push_back(*msg0) ; 
    ind.data = 0 ; 
    sync_msg.indexes.push_back(ind) ; 
    sync_msg.images.push_back(*msg1) ; 
    ind.data = 1 ; 
    sync_msg.indexes.push_back(ind) ; 
    sync_msg.images.push_back(*msg2) ; 
    ind.data = 2 ; 
    sync_msg.indexes.push_back(ind) ; 
    sync_publisher.publish(sync_msg) ;
}

int main(int argc , char** argv)
{
    std::string topic1_name , topic2_name , topic3_name , output_topic ;
    ros::init(argc,argv,"3cams_synch") ; 
    ros::NodeHandle nh("~") ;
    if(! nh.getParam("cam0topic",topic1_name)) 
    {
        ROS_INFO("Could not get the name of the cam0topic. Defaulting to %s","/cam0") ; 
        topic1_name = "/cam0" ; 
    }

    if(! nh.getParam("cam1topic",topic2_name)) 
    {
        ROS_INFO("Could not get the name of the cam1topic. Defaulting to %s","/cam1") ; 
        topic2_name = "/cam1" ; 
    }

    if(! nh.getParam("cam2topic",topic3_name)) 
    {
        ROS_INFO("Could not get the name of the cam2topic. Defaulting to %s","/cam2") ; 
        topic3_name = "/cam2" ; 
    }

    if(! nh.getParam("syncher_out",output_topic)) 
    {
        ROS_INFO("Could not get the name of the cam1topic. Defaulting to %s","/2cams_out") ; 
        output_topic = "/2cams_out" ; 
    }
    auto image_sub0 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, topic1_name, 1);
    auto image_sub1 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, topic2_name, 1);
    auto image_sub2 = std::make_shared<message_filters::Subscriber<sensor_msgs::Image>>(nh, topic3_name, 1);
    auto sync = std::make_shared<message_filters::Synchronizer<sync_pol>>(sync_pol(5), *image_sub0, *image_sub1,*image_sub2);
    sync_cam.push_back(sync);
    sync_subs_cam.push_back(image_sub0);
    sync_subs_cam.push_back(image_sub1);
    sync_subs_cam.push_back(image_sub2);
    ROS_INFO("subscribing to cam (triple): %s", topic1_name.c_str());
    ROS_INFO("subscribing to cam (triple): %s", topic2_name.c_str());
    ROS_INFO("subscribing to cam (triple): %s", topic3_name.c_str());
    sync_publisher = nh.advertise<ov_dai::syncCameras>(output_topic,2); 
    sync->registerCallback(&triple_cam_callback) ; 
    ros::spin() ; 
}