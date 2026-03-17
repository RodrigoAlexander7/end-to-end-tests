#pragma once

#include <opencv2/core.hpp>
#include <string>
#include <array>

namespace cansat {

/**
 * Calibration data loaded from a NumPy .npz file.
 */
struct CalibrationData {
    cv::Mat camera_matrix_left;
    cv::Mat dist_coeffs_left;
    cv::Mat camera_matrix_right;
    cv::Mat dist_coeffs_right;

    // Precomputed remap matrices for undistortion
    cv::Mat map1_left, map2_left;
    cv::Mat map1_right, map2_right;

    bool loaded = false;
};

/**
 * Image processing module: split, undistort, anaglyph generation.
 * Preallocates all intermediate buffers on first use.
 */
class ImageProcessor {
public:
    ImageProcessor() = default;

    /**
     * Load calibration from remap matrices passed as NumPy arrays.
     * Called from Python via pybind11 with the actual matrix data.
     */
    void load_calibration(const cv::Mat& map1_left, const cv::Mat& map2_left,
                          const cv::Mat& map1_right, const cv::Mat& map2_right);

    /** Check if calibration is loaded. */
    bool has_calibration() const { return calib_.loaded; }

    /**
     * Split a side-by-side stereo frame into left and right images.
     * @param stereo_frame  Input 2560x720 image.
     * @param left          Output left 1280x720 (preallocated).
     * @param right         Output right 1280x720 (preallocated).
     */
    void split_stereo(const cv::Mat& stereo_frame, cv::Mat& left, cv::Mat& right);

    /**
     * Apply remap-based undistortion to left and right images.
     * Requires calibration to be loaded.
     */
    void undistort(const cv::Mat& left_in, const cv::Mat& right_in,
                   cv::Mat& left_out, cv::Mat& right_out);

    /**
     * Generate a red-cyan anaglyph from left/right images.
     * Left → red channel, Right → cyan (green + blue).
     * Uses optimized channel operations, no per-pixel loops.
     */
    void make_anaglyph(const cv::Mat& left, const cv::Mat& right, cv::Mat& anaglyph);

    /**
     * Full processing pipeline: split → undistort → anaglyph.
     * @param stereo_frame  Input 2560x720 side-by-side image.
     * @param anaglyph      Output anaglyph image.
     */
    void process(const cv::Mat& stereo_frame, cv::Mat& anaglyph);

private:
    CalibrationData calib_;

    // Preallocated intermediate buffers
    cv::Mat left_raw_, right_raw_;
    cv::Mat left_undist_, right_undist_;
    std::array<cv::Mat, 3> left_channels_, right_channels_;
    std::array<cv::Mat, 3> anaglyph_channels_;
};

} // namespace cansat
