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
#include <ov_dai/TrackerManager.h>
#include <ov_dai/opencv_yaml_parse.h>
#include <memory>
#include <thread>
#include <mutex>

class trackerBuilder
{
    public:
        trackerBuilder(std::string config_path,std::shared_ptr<ros::NodeHandle> pnh,ov_dai::YamlParser parser) : 
        _config_path(config_path) , 
        _nh(pnh) , 
        parser(parser)
        {

        };
        std::shared_ptr<dai::Pipeline> _pipeline ; 
        std::shared_ptr<ros::NodeHandle> _nh ; 
        std::string _config_path ;
        std::vector<std::atomic_bool> frame_consuming_list ;
        ov_dai::YamlParser parser ;
        ov_dai::CamerasTrackerFactory f ;   
    private : 
        std::vector<std::pair<std::shared_ptr<dai::DataOutputQueue>,std::shared_ptr<dai::DataOutputQueue>>> build_pipeline_links(std::shared_ptr<dai::Pipeline> pipeline)
        {
            // check if config file exists
            if(! boost::filesystem::exists(_config_path))
                throw boost::filesystem::filesystem_error(
                    "File does not exist", 
                    _config_path, 
                    std::make_error_code(std::errc::no_such_file_or_directory)
                );
            parser = ov_dai::YamlParser(_config_path) ; 
            f.print_and_load_trackers(parser); 
            std::vector<ros::Subscriber> sub_list ; 
            std::vector<dai::node::XLinkOut> passthrough_output_nodes ;
            std::vector<dai::node::XLinkOut> features_output_nodes ;
            std::vector<dai::DataOutputQueue> device_input_queue ;
            auto trackers = f.createTrackers(pipeline) ; 
            for(size_t i =0 ; i<trackers.size() ; i++)
            {
                // Create cameras 
                auto cam = pipeline->create<dai::node::MonoCamera>() ; 
                cam->setResolution(ov_dai::CamerasTrackerFactory::sensor_resolution_map[f.tracker_option_list[i].resolution]) ; 
                cam->setBoardSocket(ov_dai::CamerasTrackerFactory::socket_map_list[f.tracker_option_list[i].socket]) ; 
                cam->setFps(f.tracker_option_list[i].framerate) ; 
                auto p_out = pipeline->create<dai::node::XLinkOut>() ;
                p_out->setStreamName("passthrough"+ std::to_string(i))
                auto f_out = pipeline->create<dai::node::XLinkOut>() ;
                f_out->setStreamName("features"+ std::to_string(i))
                passthrough_output_nodes.push_back(p_out) ;
                features_output_nodes.push_back(f_out) ; 
                // Linking 
                cam->out.link(trackers[t]->inputImage) ;
                trackers[t]->passthroughInputImage.link(p_out->input) ;
                trackers[t]->outputFeatures.link(f_out->input) ;  
            }
            dai::Device device(pipeline);
            std::vector<std::pair<std::shared_ptr<dai::DataOutputQueue>,std::shared_ptr<dai::DataOutputQueue>>> result ; 
            for(size_t i =0 ; i<trackers.size() ; i++)
                result.push_back(std::make_pair(device.getOutputQueue("passthrough"+ std::to_string(i), 2, False),device.getOutputQueue("features"+ std::to_string(i), 2, False))) ; 
            return result ; 
        } ; 
} ; 

