#ifndef OV_CORE_TRACK_EXTERNAL_H
#define OV_CORE_TRACK_EXTERNAL_H

#include "TrackBase.h"

namespace ov_core {

class TrackExternal : public TrackBase {
public:
    explicit TrackExternal(std::unordered_map<size_t, std::shared_ptr<CamBase>> cameras, int numfeats, int numaruco, bool stereo,
                    HistogramMethod histmethod, int fast_threshold, int gridx, int gridy, int minpxdist)
        : TrackBase(cameras, numfeats, numaruco, stereo, histmethod), threshold(fast_threshold), grid_x(gridx), grid_y(gridy),
        min_px_dist(minpxdist) {} 
    
    /**
    * @brief Process a new image
    * @param message Contains our timestamp, images, and camera ids
    */
    void feed_new_camera(const CameraData &message) override;

protected : 
    /**
    * @brief Process a new monocular image
    * @param message Contains our timestamp, images, and camera ids
    * @param msg_id the camera index in message data vector
    */
    void feed_monocular(const CameraData &message, size_t msg_id);

    // Performs a screening and filtering of the incomming tracing points, we won't just take them raw
    void perform_screening(const std::vector<TrackedFeature> &features0, std::vector<TrackedFeature> &features1, 
            std::vector<TrackedFeature> &featuresOut ,size_t id0, size_t id1, std::vector<uchar> &mask_out);

    // Parameters for our FAST grid detector
    int threshold;
    int grid_x;
    int grid_y;

    // Minimum pixel distance to be "far away enough" to be a different extracted feature
    int min_px_dist;
    // The current image map. Will be filled blanck in release mode.
    std::map<size_t, cv::Mat> img_curr; 
    std::map<size_t, std::vector<TrackedFeature>> features_last , features_curr ; 
} ; 


} // namespace ov_core
#endif /* OV_CORE_TRACK_EXTERNAL_H */
