#include "encoding.hpp"
#include <opencv2/imgcodecs.hpp>
#include <iostream>

namespace cansat {

JpegEncoder::JpegEncoder(int quality) : quality_(quality) {
    update_params();
}

void JpegEncoder::set_quality(int quality) {
    quality_ = std::max(0, std::min(100, quality));
    update_params();
}

void JpegEncoder::update_params() {
    params_ = {cv::IMWRITE_JPEG_QUALITY, quality_};
}

bool JpegEncoder::encode(const cv::Mat& image, std::vector<uint8_t>& buffer) {
    if (image.empty()) {
        std::cerr << "[encoding] Empty image, cannot encode" << std::endl;
        return false;
    }

    try {
        // imencode reuses the buffer's allocated memory when possible
        if (!cv::imencode(".jpg", image, buffer, params_)) {
            std::cerr << "[encoding] JPEG encoding failed" << std::endl;
            return false;
        }
        return true;
    } catch (const cv::Exception& e) {
        std::cerr << "[encoding] OpenCV error: " << e.what() << std::endl;
        return false;
    }
}

std::vector<uint8_t> JpegEncoder::encode(const cv::Mat& image) {
    std::vector<uint8_t> buffer;
    // Reserve typical JPEG size to reduce reallocations
    buffer.reserve(200 * 1024);
    encode(image, buffer);
    return buffer;
}

} // namespace cansat
