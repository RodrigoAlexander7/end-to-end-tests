#pragma once

#include "capture.hpp"
#include "processing.hpp"
#include "encoding.hpp"

#include <vector>
#include <cstdint>
#include <functional>

namespace cansat {

/**
 * Configuration for the acquisition pipeline.
 */
struct PipelineConfig {
    int device_id = 0;
    int frame_width = 2560;
    int frame_height = 720;
    int fps = 30;
    int jpeg_quality = 80;
    int num_captures = 3;
    int interval_seconds = 3;
};

/**
 * Full acquisition pipeline: capture → process → encode.
 * Owns all modules and preallocated buffers.
 */
class Pipeline {
public:
    explicit Pipeline(const PipelineConfig& config = PipelineConfig{});
    ~Pipeline() = default;

    /** Initialize camera and processing modules. */
    void initialize();

    /** Load calibration remap matrices. */
    void load_calibration(const cv::Mat& map1_left, const cv::Mat& map2_left,
                          const cv::Mat& map1_right, const cv::Mat& map2_right);

    /**
     * Capture and process a single frame.
     * @return JPEG-encoded anaglyph as byte vector.
     */
    std::vector<uint8_t> capture_and_process();

    /**
     * Run full acquisition: capture num_captures images with interval_seconds delay.
     * @return Vector of JPEG-encoded anaglyphs.
     */
    std::vector<std::vector<uint8_t>> run_acquisition();

    /** Release camera resources. */
    void shutdown();

    /** Get/set JPEG quality. */
    void set_jpeg_quality(int quality);
    int get_jpeg_quality() const { return encoder_.get_quality(); }

    /** Get pipeline config. */
    const PipelineConfig& config() const { return config_; }

private:
    PipelineConfig config_;
    CameraCapture camera_;
    ImageProcessor processor_;
    JpegEncoder encoder_;

    // Preallocated buffers
    cv::Mat frame_buffer_;
    cv::Mat anaglyph_buffer_;
    std::vector<uint8_t> jpeg_buffer_;
};

} // namespace cansat
