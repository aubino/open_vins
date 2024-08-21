/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_DAI_TRACKEROPTIONS_H
#define OV_DAI_TRACKEROPTIONS_H

#include <Eigen/Eigen>
#include <iostream>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include <depthai/depthai.hpp>
#include "opencv_yaml_parse.h"

namespace ov_dai {
  struct TrackerOptions
  {
    std::string socket ; 
    std::string topic ;
    std::string topic_in ; 
    bool upload ;  
    double framerate ; 
    std::string resolution ;
    size_t num_points ;
    double fast_threshold ; 
    size_t grid_x , grid_y ; 
    bool use_klt ; 
    double knn_ratio ; 
    size_t min_px_dist ;  
  }

/**
 * @brief Struct which stores all options needed for state estimation.
 *
 * This is broken into a few different parts: estimator, trackers, and simulation.
 * If you are going to add a parameter here you will need to add it to the parsers.
 * You will also need to add it to the print statement at the bottom of each.
 */
struct CamerasTrackerFactory {

  /**
   * @brief This function will load the non-simulation parameters of the system and print.
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load(const std::shared_ptr<ov_dai::YamlParser> &parser = nullptr) {
    print_and_load_trackers(parser);
  }

  // ESTIMATOR ===============================
  size_t max_cameras
  const std::map<std::string,dai::CameraBoardSocket> socket_map_list {{"a" : dai::CameraBoardSocket::CAM_A},
                                                                      {"b" : dai::CameraBoardSocket::CAM_B},
                                                                      {"c" : dai::CameraBoardSocket::CAM_C},
                                                                      {"d" : dai::CameraBoardSocket::CAM_D},
                                                                      {"e" : dai::CameraBoardSocket::CAM_E},
                                                                      {"f" : dai::CameraBoardSocket::CAM_F},
                                                                      {"g" : dai::CameraBoardSocket::CAM_G},
                                                                      {"h" : dai::CameraBoardSocket::CAM_H},
                                                                      {"i" : dai::CameraBoardSocket::CAM_I},
                                                                      {"j" : dai::CameraBoardSocket::CAM_J},
                                                                      {"A" : dai::CameraBoardSocket::CAM_A},
                                                                      {"B" : dai::CameraBoardSocket::CAM_B},
                                                                      {"C" : dai::CameraBoardSocket::CAM_C},
                                                                      {"D" : dai::CameraBoardSocket::CAM_D},
                                                                      {"E" : dai::CameraBoardSocket::CAM_E},
                                                                      {"F" : dai::CameraBoardSocket::CAM_F},
                                                                      {"G" : dai::CameraBoardSocket::CAM_G},
                                                                      {"H" : dai::CameraBoardSocket::CAM_H},
                                                                      {"I" : dai::CameraBoardSocket::CAM_I},
                                                                      {"J" : dai::CameraBoardSocket::CAM_J}} ; 
  

  const std::map <std::string, dai::MonoCameraProperties::SensorResolution> sensor_resolution_map {{"400p" : dai::MonoCameraProperties::SensorResolution::THE_400_P},
                                                                                                    {"720p" : dai::MonoCameraProperties::SensorResolution::THE_720_P},
                                                                                                    {"800p" : dai::MonoCameraProperties::SensorResolution::THE_800_P},
                                                                                                    {"480p" : dai::MonoCameraProperties::SensorResolution::THE_480_P},
                                                                                                    {"1200p" : dai::MonoCameraProperties::SensorResolution::THE_1200_P}} ; 
  /// Depthai sockets to use on each cameras
  std::vector<TrackerOptions> tracker_option_list ; 

  /**
   * @brief Function to convert tracker config files into node 
   */
  std::vector<dai::node::FeatureTracker> createTrackers( std::shared_ptr<dai::Pipeline> pipeline )
  {
    
    auto numShaves = 2;
    auto numMemorySlices = 2;
    std::vector<dai::node::FeatureTracker> trackers ; 
    for(size_t i=0 ; i < max_cameras ;  i++ )
    {
      auto featureTracker = pipeline->create<dai::node::FeatureTracker>();
      featureTracker->setHardwareResources(numShaves,numMemorySlices) ;
      featureTracker->setNumTargetFeatures( tracker_option_list[i].num_points) ;
      featureTracker->setMotionEstimator(dai::FeatureTrackerConfig::MotionEstimator::Type::LUCAS_KANADE_OPTICAL_FLOW) ;
      featureTracker->setFeatureMaintainer(true) ;
      dai::RawFeatureTrackerConfig::CornerDetector corner_detector ; 
      corner_detector.cellGridDimension = std::min{std::max({tracker_option_list[i].grid_x,tracker_option_list[i].grid_y}),4} ; 
      corner_detector.type = dai::RawFeatureTrackerConfig::CornerDetector::Type::SHI_THOMASI  ; 
      //corner_detector.thresholds.initialValue
      featureTracker->initialConfig.setCornerDetector(corner_detector) ;
      trackers.push_back(featureTracker) ;
    }
    return trackers ; 
  }



  /**
   * @brief This function will load print out all parameters related to visual tracking
   * This allows for visual checking that everything was loaded properly from ROS/CMD parsers.
   *
   * @param parser If not null, this parser will be used to load our parameters
   */
  void print_and_load_trackers(const std::shared_ptr<ov_dai::YamlParser> &parser = nullptr) {
    if (parser != nullptr) {
      parser->parse_config("max_cameras",max_cameras) ;  
      for(size_t i = 0 ; i<max_cameras ; i++)
      {
        //std::string socket , rostopic , rostopic_in , resolution ;
        TrackerOptions t ; 
        parser->parse("socket","cam" + std::to_string(size_t),t.socket) ; 
        parser->parse("topic","cam" + std::to_string(size_t), t.topic) ;
        parser->parse("topic_in","cam" + std::to_string(size_t),t.topic_in) ;
        parser->parse("upload","cam" + std::to_string(size_t),t.upload) ;
        parser->parse("framerate","cam" + std::to_string(size_t),t.framerate) ;
        parser->parse("resolution","cam" + std::to_string(size_t),t.resolution) ;
        parser->parse("num_points","cam" + std::to_string(size_t),t.num_points) ;
        parser->parse("fast_threshold","cam" + std::to_string(size_t),t.fast_threshold) ;
        parser->parse("grid_x","cam" + std::to_string(size_t),t.grid_x) ;
        parser->parse("grid_y","cam" + std::to_string(size_t),t.grid_y) ;
        parser->parse("use_klt","cam" + std::to_string(size_t),t.use_klt) ;
        parser->parse("knn_ratio","cam" + std::to_string(size_t),t.knn_ratio) ;
        parser->parse("min_px_dist","cam" + std::to_string(size_t),t.min_px_dist) ;
        tracker_option_list.push_back(t) ; 
      }
    }
  }
};

} // namespace ov_dai

#endif // OV_DAI_TRACKEROPTIONS_H