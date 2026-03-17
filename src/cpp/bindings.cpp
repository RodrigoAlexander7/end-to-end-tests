#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <opencv2/core.hpp>

#include "pipeline.hpp"

namespace py = pybind11;

namespace {

/**
 * Convert a NumPy array to cv::Mat without copying when possible.
 * Supports float32 and int16 remap matrices.
 */
cv::Mat numpy_to_mat(py::array arr) {
    py::buffer_info buf = arr.request();

    int dtype;
    if (buf.format == py::format_descriptor<float>::format()) {
        dtype = CV_32F;
    } else if (buf.format == py::format_descriptor<int16_t>::format()) {
        dtype = CV_16S;
    } else if (buf.format == py::format_descriptor<uint16_t>::format()) {
        dtype = CV_16U;
    } else if (buf.format == py::format_descriptor<double>::format()) {
        dtype = CV_64F;
    } else if (buf.format == py::format_descriptor<uint8_t>::format()) {
        dtype = CV_8U;
    } else {
        throw std::runtime_error("Unsupported numpy dtype for cv::Mat conversion");
    }

    int ndims = static_cast<int>(buf.ndim);
    if (ndims == 2) {
        return cv::Mat(static_cast<int>(buf.shape[0]),
                       static_cast<int>(buf.shape[1]),
                       dtype, buf.ptr).clone();
    } else if (ndims == 3) {
        int channels = static_cast<int>(buf.shape[2]);
        dtype = CV_MAKETYPE(dtype, channels);
        return cv::Mat(static_cast<int>(buf.shape[0]),
                       static_cast<int>(buf.shape[1]),
                       dtype, buf.ptr).clone();
    } else {
        throw std::runtime_error("Expected 2D or 3D numpy array");
    }
}

/**
 * Convert a JPEG byte vector to a Python bytes object (zero-copy via buffer).
 */
py::bytes jpeg_to_bytes(const std::vector<uint8_t>& jpeg) {
    return py::bytes(reinterpret_cast<const char*>(jpeg.data()), jpeg.size());
}

} // anonymous namespace

PYBIND11_MODULE(cansat_pipeline, m) {
    m.doc() = "CanSat image acquisition and processing pipeline";

    py::class_<cansat::PipelineConfig>(m, "PipelineConfig")
        .def(py::init<>())
        .def_readwrite("device_id", &cansat::PipelineConfig::device_id)
        .def_readwrite("frame_width", &cansat::PipelineConfig::frame_width)
        .def_readwrite("frame_height", &cansat::PipelineConfig::frame_height)
        .def_readwrite("fps", &cansat::PipelineConfig::fps)
        .def_readwrite("jpeg_quality", &cansat::PipelineConfig::jpeg_quality)
        .def_readwrite("num_captures", &cansat::PipelineConfig::num_captures)
        .def_readwrite("interval_seconds", &cansat::PipelineConfig::interval_seconds);

    py::class_<cansat::Pipeline>(m, "Pipeline")
        .def(py::init<const cansat::PipelineConfig&>(), py::arg("config") = cansat::PipelineConfig{})
        .def("initialize", &cansat::Pipeline::initialize,
             "Open camera and initialize processing modules")
        .def("load_calibration",
             [](cansat::Pipeline& self,
                py::array map1_left, py::array map2_left,
                py::array map1_right, py::array map2_right) {
                 self.load_calibration(
                     numpy_to_mat(map1_left), numpy_to_mat(map2_left),
                     numpy_to_mat(map1_right), numpy_to_mat(map2_right));
             },
             py::arg("map1_left"), py::arg("map2_left"),
             py::arg("map1_right"), py::arg("map2_right"),
             "Load stereo calibration remap matrices from numpy arrays")
        .def("capture_and_process",
             [](cansat::Pipeline& self) -> py::bytes {
                 auto jpeg = self.capture_and_process();
                 return jpeg_to_bytes(jpeg);
             },
             "Capture one frame, process, and return JPEG bytes")
        .def("run_acquisition",
             [](cansat::Pipeline& self) -> py::list {
                 auto results = self.run_acquisition();
                 py::list out;
                 for (auto& jpeg : results) {
                     out.append(jpeg_to_bytes(jpeg));
                 }
                 return out;
             },
             "Run full acquisition (N captures with interval). Returns list of JPEG bytes.")
        .def("shutdown", &cansat::Pipeline::shutdown,
             "Release camera and cleanup")
        .def("set_jpeg_quality", &cansat::Pipeline::set_jpeg_quality,
             py::arg("quality"), "Set JPEG encoding quality (0-100)")
        .def("get_jpeg_quality", &cansat::Pipeline::get_jpeg_quality,
             "Get current JPEG quality")
        .def_property_readonly("config", &cansat::Pipeline::config,
             "Get pipeline configuration");
}
