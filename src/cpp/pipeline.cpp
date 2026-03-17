#include "pipeline.hpp"
#include <chrono>
#include <thread>
#include <iostream>
#include <stdexcept>

namespace cansat {

Pipeline::Pipeline(const PipelineConfig& config)
    : config_(config),
      camera_(config.device_id, config.frame_width, config.frame_height, config.fps),
      encoder_(config.jpeg_quality) {
    // Reserve typical JPEG buffer size
    jpeg_buffer_.reserve(200 * 1024);
}

void Pipeline::initialize() {
    camera_.open();
    std::cout << "[pipeline] Initialized" << std::endl;
}

void Pipeline::load_calibration(const cv::Mat& map1_left, const cv::Mat& map2_left,
                                 const cv::Mat& map1_right, const cv::Mat& map2_right) {
    processor_.load_calibration(map1_left, map2_left, map1_right, map2_right);
}

std::vector<uint8_t> Pipeline::capture_and_process() {
    // Capture
    if (!camera_.grab_frame(frame_buffer_)) {
        throw std::runtime_error("Frame capture failed");
    }

    // Process: split → undistort → anaglyph
    processor_.process(frame_buffer_, anaglyph_buffer_);

    // Encode to JPEG
    if (!encoder_.encode(anaglyph_buffer_, jpeg_buffer_)) {
        throw std::runtime_error("JPEG encoding failed");
    }

    // Return a copy (the internal buffer will be reused next call)
    return jpeg_buffer_;
}

std::vector<std::vector<uint8_t>> Pipeline::run_acquisition() {
    std::vector<std::vector<uint8_t>> results;
    results.reserve(config_.num_captures);

    for (int i = 0; i < config_.num_captures; ++i) {
        std::cout << "[pipeline] Capture " << (i + 1) << "/" << config_.num_captures << std::endl;

        auto start = std::chrono::steady_clock::now();

        try {
            results.push_back(capture_and_process());
            std::cout << "[pipeline] Image " << (i + 1) << ": "
                      << results.back().size() << " bytes" << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "[pipeline] Error on capture " << (i + 1) << ": " << e.what() << std::endl;
            results.push_back({}); // Empty buffer signals failure
        }

        // Wait for the remaining interval (subtract processing time)
        if (i < config_.num_captures - 1) {
            auto elapsed = std::chrono::steady_clock::now() - start;
            auto wait = std::chrono::seconds(config_.interval_seconds) - elapsed;
            if (wait > std::chrono::milliseconds(0)) {
                std::this_thread::sleep_for(wait);
            }
        }
    }

    return results;
}

void Pipeline::shutdown() {
    camera_.release();
    std::cout << "[pipeline] Shutdown complete" << std::endl;
}

void Pipeline::set_jpeg_quality(int quality) {
    encoder_.set_quality(quality);
    config_.jpeg_quality = quality;
}

} // namespace cansat
