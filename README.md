# CanSat Image Acquisition & Processing Pipeline

Optimized stereo image capture, anaglyph processing, and XBee transmission pipeline for Raspberry Pi Zero 2 W.

## Architecture

```
┌─────────────────────────────────────────────────┐
│  Python Orchestration (cansat_pipeline.py)       │
│  - Configuration, calibration loading            │
│  - XBee serial transmission with CRC-16          │
├─────────────────────────────────────────────────┤
│  pybind11 Bindings (cansat_pipeline.so)          │
├─────────────────────────────────────────────────┤
│  C++ Pipeline                                    │
│  ┌──────────┐ ┌────────────┐ ┌─────────┐       │
│  │ Capture  │→│ Processing │→│ Encoding│       │
│  │ (V4L2)   │ │ Split/Remap│ │ (JPEG)  │       │
│  │          │ │ Anaglyph   │ │         │       │
│  └──────────┘ └────────────┘ └─────────┘       │
└─────────────────────────────────────────────────┘
```

## Prerequisites

### Raspberry Pi OS (target)

```bash
sudo apt update
sudo apt install -y \
    build-essential cmake \
    libopencv-dev python3-opencv \
    python3-dev python3-pip python3-numpy \
    v4l-utils
```

### Python dependencies

```bash
pip install pybind11 pyserial numpy
```

## Building

```bash
# From project root
mkdir -p build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j$(nproc)

# Copy the built module to the project root
cp cansat_pipeline*.so ../
```

### Cross-compilation note

If building on a different machine, ensure you target `aarch64` / ARMv8-A. The CMakeLists.txt auto-detects ARM and applies `-mcpu=cortex-a53` optimizations.

## Usage

### Full pipeline (capture + XBee transmission)

```bash
python3 scripts/cansat_pipeline.py \
    --device 0 \
    --quality 80 \
    --num-captures 3 \
    --interval 3 \
    --calibration calibration.npz \
    --serial-port /dev/ttyUSB0 \
    --baudrate 9600
```

### Save to disk (no XBee)

```bash
python3 scripts/cansat_pipeline.py \
    --device 0 \
    --quality 80 \
    --output-dir output
```

### Test scripts

```bash
# Full pipeline test (requires camera)
python3 scripts/run_pipeline.py --mode camera

# XBee packet construction test (no hardware needed)
python3 scripts/run_pipeline.py --mode packets
```

## Configuration

| Parameter         | Default | Description                          |
|-------------------|---------|--------------------------------------|
| `--device`        | 0       | V4L2 camera device index             |
| `--width`         | 2560    | Stereo frame width (side-by-side)    |
| `--height`        | 720     | Frame height                         |
| `--quality`       | 80      | JPEG quality (0-100)                 |
| `--num-captures`  | 3       | Number of images to capture          |
| `--interval`      | 3       | Seconds between captures             |
| `--calibration`   | calibration.npz | Path to calibration file    |
| `--serial-port`   | None    | XBee serial port                     |
| `--baudrate`      | 9600    | Serial baudrate                      |

## Calibration File Format

The pipeline expects a NumPy `.npz` file with precomputed remap matrices:

```
map1_left   - float32, shape (720, 1280)
map2_left   - float32, shape (720, 1280)
map1_right  - float32, shape (720, 1280)
map2_right  - float32, shape (720, 1280)
```

Alternatively, if remap matrices are not present, the Python layer will attempt to compute them from:

```
camera_matrix_left  - float64, shape (3, 3)
dist_coeffs_left    - float64, shape (5,) or (8,)
camera_matrix_right - float64, shape (3, 3)
dist_coeffs_right   - float64, shape (5,) or (8,)
```

## XBee Packet Format

Each image is split into packets of ≤200 bytes:

```
┌──────┬────────┬─────┬───────┬─────┬─────────┬─────┐
│ SYNC │ IMG_ID │ SEQ │ TOTAL │ LEN │ PAYLOAD │ CRC │
│ 2B   │ 1B     │ 2B  │ 2B    │ 2B  │ ≤200B   │ 2B  │
└──────┴────────┴─────┴───────┴─────┴─────────┴─────┘

SYNC:    0xAA55 (packet start marker)
IMG_ID:  Image identifier (0-255)
SEQ:     Packet sequence number
TOTAL:   Total number of packets for this image
LEN:     Payload length in this packet
PAYLOAD: JPEG data chunk
CRC:     CRC-16/CCITT over (IMG_ID + SEQ + TOTAL + LEN + PAYLOAD)
```

## Project Structure

```
├── CMakeLists.txt              # Build configuration
├── README.md                   # This file
├── requirements.txt            # Python dependencies
├── requirements_linux.txt      # System package requirements
├── calibration.npz             # Stereo calibration data (user-provided)
├── src/
│   └── cpp/
│       ├── capture.hpp/cpp     # Camera capture (V4L2, MJPEG)
│       ├── processing.hpp/cpp  # Split, undistort, anaglyph
│       ├── encoding.hpp/cpp    # JPEG encoding
│       ├── pipeline.hpp/cpp    # Pipeline orchestrator
│       └── bindings.cpp        # pybind11 Python bindings
└── scripts/
    ├── cansat_pipeline.py      # Main orchestration + XBee TX
    └── run_pipeline.py         # Standalone test script
```

## Optimization Notes

- **Preallocated buffers**: All cv::Mat and std::vector buffers are reused across frames
- **V4L2 + MJPEG**: Direct hardware access, hardware-decoded MJPEG when available
- **Minimal copies**: ROI splitting uses zero-copy references; only copies when necessary
- **Native C++**: All heavy processing (remap, channel ops, JPEG) runs in optimized C++
- **Single camera session**: Device stays open for all captures, avoiding reinit overhead
- **ARM tuning**: `-O2 -mcpu=cortex-a53` for Raspberry Pi Zero 2 W Cortex-A53 cores
- **Chunked TX**: 50ms inter-packet delay balances throughput vs CPU/power consumption
