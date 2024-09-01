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
    size_t cam_id = message.sensor_ids.at(msg_id);
    std::lock_guard<std::mutex> lck(mtx_feeds.at(cam_id));
    cv::Mat mask = message.masks.at(msg_id);
    cv::Mat img = img_curr.at(cam_id);
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
        return;
    }
    // First we should make that the last images have enough features so we can do KLT
    // This will "top-off" our number of tracks so always have a constant number
    int pts_before_detect = (int)pts_last[cam_id].size();
    auto pts_left_old = pts_last[cam_id];
    auto ids_left_old = ids_last[cam_id];
    rT3 = boost::posix_time::microsec_clock::local_time();
    // Our return success masks, and predicted new features
    std::vector<uchar> mask_ll;
    std::vector<cv::KeyPoint> pts_left_new = pts_left_old;
    // perform_detection_monocular(img_pyramid_last[cam_id], img_mask_last[cam_id], pts_left_old, ids_left_old);

}