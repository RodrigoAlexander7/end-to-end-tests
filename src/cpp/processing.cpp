#include "processing.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>

namespace cansat {

void ImageProcessor::load_calibration(const cv::Mat& map1_left, const cv::Mat& map2_left,
                                       const cv::Mat& map1_right, const cv::Mat& map2_right) {
    calib_.map1_left = map1_left.clone();
    calib_.map2_left = map2_left.clone();
    calib_.map1_right = map1_right.clone();
    calib_.map2_right = map2_right.clone();
    calib_.loaded = true;
    std::cout << "[processing] Calibration remap matrices loaded" << std::endl;
}

void ImageProcessor::split_stereo(const cv::Mat& stereo_frame, cv::Mat& left, cv::Mat& right) {
    int half_width = stereo_frame.cols / 2;

    // Use ROI references — zero copy
    cv::Mat left_roi = stereo_frame(cv::Rect(0, 0, half_width, stereo_frame.rows));
    cv::Mat right_roi = stereo_frame(cv::Rect(half_width, 0, half_width, stereo_frame.rows));

    // Copy to output buffers (needed since stereo_frame may be overwritten next capture)
    left_roi.copyTo(left);
    right_roi.copyTo(right);
}

void ImageProcessor::undistort(const cv::Mat& left_in, const cv::Mat& right_in,
                                cv::Mat& left_out, cv::Mat& right_out) {
    if (!calib_.loaded) {
        // No calibration: pass through
        left_in.copyTo(left_out);
        right_in.copyTo(right_out);
        return;
    }

    cv::remap(left_in, left_out, calib_.map1_left, calib_.map2_left,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
    cv::remap(right_in, right_out, calib_.map1_right, calib_.map2_right,
              cv::INTER_LINEAR, cv::BORDER_CONSTANT);
}

void ImageProcessor::make_anaglyph(const cv::Mat& left, const cv::Mat& right, cv::Mat& anaglyph) {
    // Split channels (BGR order in OpenCV)
    cv::split(left, left_channels_.data());
    cv::split(right, right_channels_.data());

    // Anaglyph: B=right_B, G=right_G, R=left_R
    anaglyph_channels_[0] = right_channels_[0]; // Blue from right
    anaglyph_channels_[1] = right_channels_[1]; // Green from right
    anaglyph_channels_[2] = left_channels_[2];  // Red from left

    cv::merge(anaglyph_channels_.data(), 3, anaglyph);
}

void ImageProcessor::process(const cv::Mat& stereo_frame, cv::Mat& anaglyph) {
    if (stereo_frame.empty()) {
        throw std::runtime_error("Empty stereo frame");
    }

    // Step 1: Split
    split_stereo(stereo_frame, left_raw_, right_raw_);

    // Step 2: Undistort
    undistort(left_raw_, right_raw_, left_undist_, right_undist_);

    // Step 3: Anaglyph
    make_anaglyph(left_undist_, right_undist_, anaglyph);
}

} // namespace cansat
