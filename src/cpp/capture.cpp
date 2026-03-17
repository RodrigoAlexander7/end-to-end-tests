#include "capture.hpp"
#include <iostream>

namespace cansat {

CameraCapture::CameraCapture(int device_id, int width, int height, int fps)
    : device_id_(device_id), width_(width), height_(height), fps_(fps) {}

CameraCapture::~CameraCapture() {
    release();
}

void CameraCapture::open() {
    if (opened_) return;

    // Use V4L2 backend for direct hardware access on Linux
    cap_.open(device_id_, cv::CAP_V4L2);
    if (!cap_.isOpened()) {
        throw std::runtime_error("Failed to open camera device " + std::to_string(device_id_));
    }

    // Request MJPEG format for hardware-accelerated decoding
    cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
    cap_.set(cv::CAP_PROP_FPS, fps_);

    // Reduce internal buffer to minimize latency
    cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);

    opened_ = true;

    // Discard first few frames (auto-exposure warmup)
    cv::Mat discard;
    for (int i = 0; i < 5; ++i) {
        cap_.read(discard);
    }

    std::cout << "[capture] Camera opened: "
              << cap_.get(cv::CAP_PROP_FRAME_WIDTH) << "x"
              << cap_.get(cv::CAP_PROP_FRAME_HEIGHT)
              << " @ " << cap_.get(cv::CAP_PROP_FPS) << " fps" << std::endl;
}

void CameraCapture::release() {
    if (opened_) {
        cap_.release();
        opened_ = false;
        std::cout << "[capture] Camera released" << std::endl;
    }
}

bool CameraCapture::is_open() const {
    return opened_ && cap_.isOpened();
}

bool CameraCapture::grab_frame(cv::Mat& frame) {
    if (!opened_) {
        std::cerr << "[capture] Camera not opened" << std::endl;
        return false;
    }

    if (!cap_.read(frame)) {
        std::cerr << "[capture] Failed to grab frame" << std::endl;
        return false;
    }

    if (frame.empty()) {
        std::cerr << "[capture] Empty frame received" << std::endl;
        return false;
    }

    return true;
}

} // namespace cansat
