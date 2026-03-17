#pragma once

#include <opencv2/core.hpp>
#include <vector>
#include <cstdint>

namespace cansat {

/**
 * JPEG encoding module with configurable quality.
 * Preallocates the output buffer to avoid repeated allocations.
 */
class JpegEncoder {
public:
    /**
     * @param quality JPEG quality (0-100). Default 80 balances size and clarity.
     */
    explicit JpegEncoder(int quality = 80);

    /** Set JPEG quality (0-100). */
    void set_quality(int quality);

    /** Get current JPEG quality. */
    int get_quality() const { return quality_; }

    /**
     * Encode a cv::Mat to JPEG.
     * @param image     Input BGR image.
     * @param buffer    Output byte buffer (reused across calls).
     * @return true on success.
     */
    bool encode(const cv::Mat& image, std::vector<uint8_t>& buffer);

    /**
     * Encode and return the buffer directly.
     * Less efficient (copy on return) but convenient for pybind11.
     */
    std::vector<uint8_t> encode(const cv::Mat& image);

private:
    int quality_;
    std::vector<int> params_;

    void update_params();
};

} // namespace cansat
