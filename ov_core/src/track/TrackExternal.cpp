#include "TrackExternal.h"
#include "Grider_FAST.h"
#include "Grider_GRID.h"
#include "cam/CamBase.h"
#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"

using namespace ov_core;

void TrackExternal::feed_monocular(const CameraData &message, size_t msg_id)
{
    PRINT_DEBUG(BOLDYELLOW "TrackExternal::feed_monocular recieved %zu features",message.features[msg_id].size()) ; 
    size_t cam_id = message.sensor_ids.at(msg_id);
    std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));
    cv::Mat mask = message.masks.at(msg_id);
    cv::Mat img ; 
    try
    {
        img= img_curr.at(cam_id); 
    }
    catch (const std::out_of_range& e)
    {
        img = cv::Mat(camera_calib.at(cam_id)->w(),camera_calib.at(cam_id)->h(),CV_8UC1 , cv::Scalar(255)) ; 
    }
    rT2 = boost::posix_time::microsec_clock::local_time();
    
    // If we didn't have any successful tracks last time, just extract this time
    // This also handles, the tracking initalization on the first call to this extractor
    if (pts_last[cam_id].empty()) {
        // Detect new features
        std::vector<cv::KeyPoint> good_left;
        std::vector<size_t> good_ids_left;
        for(const auto pt : message.features[cam_id])
        {
            good_left.emplace_back(pt.position,1) ; 
            good_ids_left.emplace_back(pt.id) ; 
        }
        // perform_detection_monocular(imgpyr, mask, good_left, good_ids_left);
        // Save the current image and pyramid
        std::lock_guard<std::mutex> lckv(mtx_last_vars);
        img_last[cam_id] = img;
        img_mask_last[cam_id] = mask;
        pts_last[cam_id] = good_left;
        ids_last[cam_id] = good_ids_left;
        features_last[cam_id] = message.features[cam_id] ; 
        return;
    }
    
    std::vector<TrackedFeature> screened_features ; 
    perform_screening(features_last[cam_id],message.features[cam_id],screened_features,cam_id) ; 
    // Additionall screening here with mask 
    std::vector<TrackedFeature> masked_features ; 
    for(const auto f : screened_features)
    {
        if ((int)message.masks.at(msg_id).at<uint8_t>((int)f.position.y, (int)f.position.x) > 127)
            masked_features.push_back(f) ; 
    }
    // Update our feature database, with the new trackig datas
    for(const auto f : screened_features)
    {
        cv::Point2f npt_l = camera_calib.at(cam_id)->undistort_cv(f.position);
        database->update_feature(f.id,message.timestamp,cam_id,f.position.x,f.position.y,npt_l.x, npt_l.y) ; 
    }
    std::vector<cv::KeyPoint> good_left;
    std::vector<size_t> good_ids_left;
    for(const auto pt : message.features[cam_id])
    {
        good_left.emplace_back(pt.position,1) ; 
        good_ids_left.emplace_back(pt.id) ; 
    }
    // Move forward in time 
    {
        std::lock_guard<std::mutex> lckv(mtx_last_vars);
        img_last[cam_id] = img;
        img_mask_last[cam_id] = mask;
        pts_last[cam_id] = good_left;
        ids_last[cam_id] = good_ids_left;
    }
  rT5 = boost::posix_time::microsec_clock::local_time();
}

void TrackExternal::perform_screening(const std::vector<TrackedFeature> &features0, const std::vector<TrackedFeature> &features1, 
            std::vector<TrackedFeature> &featuresOut ,size_t id)
{
  /// 1 - Make sure we don't have any points with negative coordinates  
  /// 2 - Verify the age of the feature. We want at leat 2 frames for consistancy
  for(const auto f : features1) 
  {
    if(f.position.x <0 || f.position.y<0 || (int)f.position.x >= camera_calib.at(id)->w() || (int)f.position.y>= camera_calib.at(id)->h())
        continue;
    if(f.age<2) 
        continue; 
    featuresOut.push_back(f) ; 
    
  }
}

void TrackExternal::feed_new_camera(const CameraData &message)
{
    // Error check that we have all the data
    if (message.sensor_ids.empty() || message.sensor_ids.size() != message.features.size() || message.features.size() != message.masks.size()) {
        PRINT_ERROR(RED "[ERROR]: MESSAGE DATA SIZES DO NOT MATCH OR EMPTY!!!\n" RESET);
        PRINT_ERROR(RED "[ERROR]:   - message.sensor_ids.size() = %zu\n" RESET, message.sensor_ids.size());
        PRINT_ERROR(RED "[ERROR]:   - message.images.size() = %zu\n" RESET, message.images.size());
        PRINT_ERROR(RED "[ERROR]:   - message.masks.size() = %zu\n" RESET, message.masks.size());
        std::exit(EXIT_FAILURE);
    }
    rT1 = boost::posix_time::microsec_clock::local_time();
    size_t num_images = message.images.size();
    for (size_t msg_id = 0; msg_id < num_images; msg_id++) {
        // Lock this data feed for this camera
        size_t cam_id = message.sensor_ids.at(msg_id);
        std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));

        // Histogram equalize
        cv::Mat img =  message.images.at(msg_id);
        // Save!
        features_curr[cam_id] = message.features.at(msg_id) ; 
        img_curr[cam_id] = img;
    }
    // Either call our stereo or monocular version
    // If we are doing binocular tracking, then we should parallize our tracking
    if (num_images == 1) {
        feed_monocular(message, 0);
    } 
    else if (!use_stereo) {
        parallel_for_(cv::Range(0, (int)num_images), LambdaBody([&](const cv::Range &range) {
                        for (int i = range.start; i < range.end; i++) {
                        feed_monocular(message, i);
                        }
                    }));
    } else {
        PRINT_ERROR(RED "[ERROR]: invalid number of images passed %zu, we only support mono or stereo tracking", num_images);
        std::exit(EXIT_FAILURE);
    }
}