#pragma once

#include <opencv2/videoio.hpp>
#include <opencv2/core.hpp>
#include <string>
#include <stdexcept>

namespace cansat {

/**
 * Camera capture module using V4L2 backend.
 * Keeps the device open for the entire session to avoid reinitialization overhead.
 */
class CameraCapture {
public:
    /**
     * @param device_id  V4L2 device index (default 0)
     * @param width      Full frame width (2560 for stereo side-by-side)
     * @param height     Frame height (720)
     * @param fps        Desired framerate
     */
    explicit CameraCapture(int device_id = 0, int width = 2560, int height = 720, int fps = 30);
    ~CameraCapture();

    CameraCapture(const CameraCapture&) = delete;
    CameraCapture& operator=(const CameraCapture&) = delete;

    /** Open the camera device. Throws on failure. */
    void open();

    /** Release the camera device. */
    void release();

    /** Check if camera is currently open. */
    bool is_open() const;

    /**
     * Grab a single frame into a preallocated buffer.
     * @param frame Output matrix (reused across calls to avoid reallocation).
     * @return true on success.
     */
    bool grab_frame(cv::Mat& frame);

private:
    cv::VideoCapture cap_;
    int device_id_;
    int width_;
    int height_;
    int fps_;
    bool opened_ = false;
};

} // namespace cansat
