#include <ros/ros.h>
#include <boost/filesystem.hpp>
#include <sensor_msgs/Image.h>
#include <depthai/depthai.hpp>
#include <depthai_ros_msgs/TrackedFeatures.h>
#include <depthai_bridge/depthaiUtility.hpp>
#include <depthai_bridge/BridgePublisher.hpp>
#include <depthai_bridge/ImageConverter.hpp>
#include <depthai_bridge/TrackedFeaturesConverter.hpp>
#include <depthai_bridge/ImuConverter.hpp>
#include <cv_bridge/cv_bridge.h>
#include <ov_dai/TrackerManager.h>
#include <ov_dai/opencv_yaml_parse.h>
#include <memory>
#include <thread>
#include <mutex>

class trackerBuilder
{
    public:
        trackerBuilder(std::string config_path,std::shared_ptr<ros::NodeHandle> pnh) : 
        _config_path(config_path) , 
        _nh(pnh)
        {

        };
        void setupRos() 
        {
            auto r = build_pipeline_links(_pipeline) ; 
            for(size_t i =0 ; i<r.size() ; i++ )
            {
                auto q_pair = r[i] ; 
                dai::rosBridge::TrackedFeaturesConverter featConverter("cam" + std::to_string(i) + "_camera_optical_frame", true);
                auto featuresPub = std::make_shared<dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures>>(
                     q_pair.second,
                     *_nh,
                     f.tracker_option_list[i].topic + std::string("/features") ,
                     std::bind(&dai::rosBridge::TrackedFeaturesConverter::toRosMsg, &featConverter, std::placeholders::_1, std::placeholders::_2),
                     2);
                 featuresPub->addPublisherCallback() ; 
                features_publishers.push_back(featuresPub) ; 
                dai::rosBridge::ImageConverter img_converter("cam" + std::to_string(i) + "_camera_optical_frame", true) ; 
                auto imgPub = std::make_shared<dai::rosBridge::BridgePublisher<sensor_msgs::Image,dai::ImgFrame>>(q_pair.first,
                     *_nh,
                     f.tracker_option_list[i].topic,
                     std::bind(&dai::rosBridge::ImageConverter::toRosMsg, &img_converter, std::placeholders::_1, std::placeholders::_2),
                     10) ; 
                imgPub->addPublisherCallback() ; 
                image_publishers.push_back(imgPub) ; 
            }
        }
        std::shared_ptr<dai::Pipeline> _pipeline ;
        std::shared_ptr<dai::Device> device ; 
        std::shared_ptr<ros::NodeHandle> _nh ; 
        std::string _config_path ;
        std::vector<sensor_msgs::Image> image_list ; 
        std::shared_ptr<ov_dai::YamlParser> parser ;
        ov_dai::CamerasTrackerFactory f ; 
        std::vector<std::shared_ptr<dai::rosBridge::BridgePublisher<sensor_msgs::Image,dai::ImgFrame>>> image_publishers ; 
        std::vector<std::shared_ptr<dai::rosBridge::BridgePublisher<depthai_ros_msgs::TrackedFeatures, dai::TrackedFeatures>>> features_publishers ;   
        std::vector<ros::Publisher> dataPublishers ; 
    private : 
        std::vector<std::pair<std::shared_ptr<dai::DataOutputQueue>,std::shared_ptr<dai::DataOutputQueue>>> build_pipeline_links(std::shared_ptr<dai::Pipeline> pipeline)
        {
            // check if config file exists
            if(! boost::filesystem::exists(_config_path))
            {
                std::cout<<"Config file "<< _config_path << "does not exist \n" ; 
                exit(EXIT_FAILURE) ; 
            }
            parser = std::make_shared<ov_dai::YamlParser>(_config_path) ; 
            f.print_and_load_trackers(parser); 
            std::vector<ros::Subscriber> sub_list ; 
            std::vector<std::shared_ptr<dai::node::XLinkOut>> passthrough_output_nodes ;
            std::vector<std::shared_ptr<dai::node::XLinkOut>> features_output_nodes ;
            std::vector<std::shared_ptr<dai::DataOutputQueue>> device_input_queue ;
            auto trackers = f.createTrackers(pipeline) ; 
            for(size_t i =0 ; i<trackers.size() ; i++)
            {
                // Create cameras 
                auto cam = pipeline->create<dai::node::MonoCamera>() ; 
                auto res = ov_dai::sensor_resolution_map.at(f.tracker_option_list[i].resolution) ; 
                auto socket = ov_dai::socket_map_list.at(f.tracker_option_list[i].socket)  ;
                cam->setResolution(res) ; 
                cam->setBoardSocket(socket) ; 
                cam->setFps(f.tracker_option_list[i].framerate) ; 
                auto p_out = pipeline->create<dai::node::XLinkOut>() ;
                p_out->setStreamName("passthrough"+ std::to_string(i)) ; 
                auto f_out = pipeline->create<dai::node::XLinkOut>() ;
                f_out->setStreamName("features"+ std::to_string(i)) ; 
                passthrough_output_nodes.push_back(p_out) ;
                features_output_nodes.push_back(f_out) ; 
                // Linking 
                cam->out.link(trackers[i]->inputImage) ;
                trackers[i]->passthroughInputImage.link(p_out->input) ;
                trackers[i]->outputFeatures.link(f_out->input) ;  
            }
            device = std::make_shared<dai::Device>(*pipeline); 
            std::vector<std::pair<std::shared_ptr<dai::DataOutputQueue>,std::shared_ptr<dai::DataOutputQueue>>> result ; 
            for(size_t i =0 ; i<trackers.size() ; i++)
                result.push_back(std::make_pair(device->getOutputQueue("passthrough"+ std::to_string(i), 2, false),device->getOutputQueue("features"+ std::to_string(i), 2, false))) ; 
            return result ; 
        } ; 
} ; 

class externalTrackerBuilder
{
    private : 
        std::shared_ptr<dai::Pipeline> _pipeline ; 
        std::shared_ptr<ros::NodeHandle> _nh ; 
        std::string _config_path ;
        std::shared_ptr<ov_dai::YamlParser> parser ;
        ov_dai::CamerasTrackerFactory f ;
        std::shared_ptr<dai::Device> device ;  
        std::vector<ros::Subscriber> dataSubscriber ; 
        std::vector<std::shared_ptr<dai::DataInputQueue>> device_input_queues ; 
        std::vector<std::shared_ptr<dai::node::XLinkIn>> device_input_nodes ;
        std::vector<std::shared_ptr<dai::node::XLinkOut>> device_output_nodes ;  
        
    public :

        externalTrackerBuilder(std::string config_path,std::shared_ptr<ros::NodeHandle> pnh) : 
        _config_path(config_path) , 
        _nh(pnh)
        {

        }

        std::vector<std::shared_ptr<dai::DataOutputQueue>> build_pipeline(std::shared_ptr<dai::Pipeline> pipeline)
        {
            if(! boost::filesystem::exists(_config_path))
            {
                std::cout<<"Config file "<< _config_path << "does not exist \n" ; 
                exit(EXIT_FAILURE) ; 
            }
            parser = std::make_shared<ov_dai::YamlParser>(_config_path) ; 
            f.print_and_load_trackers(parser);
            auto trackers = f.createTrackers(pipeline) ;  
            // Now we only need to connect the links in to the trackers and that's it. First create the links in
            for(size_t i =0 ; i <trackers.size() ; i++)
            {
                auto link_in = pipeline->create<dai::node::XLinkIn>() ; 
                auto link_out = pipeline->create<dai::node::XLinkOut>() ;
                link_out->setStreamName("features"+ std::to_string(i))  ; 
                link_in->setStreamName("image" + std::to_string(i)) ; 
                // Linking
                link_in->out.link(trackers[i]->inputImage) ; 
                trackers[i]->outputFeatures.link(link_out->input) ;
                device_input_nodes.push_back(link_in) ; 
                device_output_nodes.push_back(link_out) ; 
            }
            device = std::make_shared<dai::Device>(*pipeline);
            std::vector<std::shared_ptr<dai::DataOutputQueue>> result ; 
            for(size_t i =0 ; i<trackers.size() ; i++)
                result.push_back(device->getOutputQueue("features"+ std::to_string(i), 2, false)) ; 
            return result ; 
        }

         void setupRos() 
        {
            auto r = build_pipeline(_pipeline) ;
            for(size_t i =0 ; i<r.size() ; i++ )
            {
                auto q_in = device->getInputQueue("image" + std::to_string(i)) ;
                device_input_queues.push_back(q_in) ;  
                auto s = _nh->subscribe<sensor_msgs::Image>(f.tracker_option_list[i].topic_in,2,
                        [&](const sensor_msgs::Image::ConstPtr& img)
                        {
                            // convert sensor_msg::Image to dai::ImageFrame
                            auto cv_img = cv_bridge::toCvCopy(*img,"mono8") ; 
                            dai::ImgFrame dai_img ; 
                            // dai_img.setFrame(cv_img->image) ; 
                            // Upload to the queue
                            q_in->send(dai_img) ; 
                        }) ; 
                dataSubscriber.push_back(s) ; 
            } 
        }
} ; 

int main(int argc, char**  argv)
{
    ros::init(argc,argv,"device_manager_node") ; 
    auto pnh = std::make_shared<ros::NodeHandle>("~") ; 
    bool upload = false ; 
    std::string config_path = "" ; 
    pnh->getParam("upload",upload) ; 
    pnh->getParam("config_path",config_path) ; 
    ov_dai::YamlParser parser(config_path) ; 
    if(upload)
    {
        auto etb = externalTrackerBuilder(config_path,pnh) ; 
        ros::spin() ; 
    }
    else
    {
        auto tb = trackerBuilder(config_path,pnh) ; 
        ros::spin() ; 
    }

}

