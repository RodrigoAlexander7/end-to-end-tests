cmake_minimum_required(VERSION 3.16)
project(cansat_pipeline LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# Optimization flags for Raspberry Pi Zero 2 W (Cortex-A53, ARMv8-A)
if(CMAKE_SYSTEM_PROCESSOR MATCHES "aarch64|arm")
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -mcpu=cortex-a53 -DNDEBUG")
else()
    set(CMAKE_CXX_FLAGS_RELEASE "-O2 -DNDEBUG")
endif()

# Default to Release build
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

# Find dependencies
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)
# Find pybind11: try system install first, then fall back to pip-installed location
find_package(pybind11 QUIET)
if(NOT pybind11_FOUND)
    execute_process(
        COMMAND python3 -m pybind11 --cmakedir
        OUTPUT_VARIABLE PYBIND11_CMAKE_DIR
        OUTPUT_STRIP_TRAILING_WHITESPACE
        RESULT_VARIABLE PYBIND11_RESULT
    )
    if(PYBIND11_RESULT EQUAL 0)
        set(pybind11_DIR "${PYBIND11_CMAKE_DIR}")
        find_package(pybind11 REQUIRED)
    else()
        message(FATAL_ERROR "pybind11 not found. Install with: pip install pybind11")
    endif()
endif()

# Source files
set(PIPELINE_SOURCES
    src/cpp/capture.cpp
    src/cpp/processing.cpp
    src/cpp/encoding.cpp
    src/cpp/pipeline.cpp
)

# pybind11 module
pybind11_add_module(cansat_pipeline
    ${PIPELINE_SOURCES}
    src/cpp/bindings.cpp
)

target_include_directories(cansat_pipeline PRIVATE
    ${OpenCV_INCLUDE_DIRS}
    src/cpp
)

target_link_libraries(cansat_pipeline PRIVATE
    ${OpenCV_LIBS}
)

# Install the module to the project directory for easy access
install(TARGETS cansat_pipeline DESTINATION ${CMAKE_SOURCE_DIR})

message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "OpenCV libs: ${OpenCV_LIBS}")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")
