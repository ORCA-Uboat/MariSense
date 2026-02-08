# Odometry

## Overview

The odometry component of MariSense provides multi-sensor data for ego-motion estimation in maritime environments. The dataset includes synchronized data from LiDAR, IMU, GNSS, and mmWave radar sensors, enabling research in mmWave radar odometry, LiDAR odometry, radar-inertial odometry, and multi-sensor fusion approaches.

### Data Format

All odometry data is stored in **ROS2 format using MCAP files**. Some data segments also include front camera images.

### Dataset Composition

The odometry dataset consists of **25 sequences** with varying durations, covering different maritime scenarios and conditions.

| Sequence    | Duration      |
| ----------- | ------------- |
| Sequence 01 | [To be added] |
| Sequence 02 | [To be added] |
| Sequence 03 | [To be added] |
| Sequence 04 | [To be added] |
| Sequence 05 | [To be added] |
| Sequence 06 | [To be added] |
| Sequence 07 | [To be added] |
| Sequence 08 | [To be added] |
| Sequence 09 | [To be added] |
| Sequence 10 | [To be added] |

## Data Summary

### Included Sensor Data

The odometry MCAP files contain the following synchronized sensor streams:

1. **LiDAR Point Clouds**
   
   - Livox AVIA (400m range, front-mounted)
   - Livox MID360 (360° coverage)
   - Format: `sensor_msgs.msg.PointCloud2`

2. **IMU Measurements**
   
   - LiDAR-integrated IMU (from both LiDARs)
   - Vessel-mounted IMU (TDK-Invensense IIM 46234)
   - Formats: `sensor_msgs.msg.Imu` (LiDAR IMU), Custom format (Vessel IMU)

3. **4D mmWave Radar**
   
   - 10 radar units with multiple waveforms
   - Format: `sensor_msgs.msg.PointCloud2`

4. **GNSS Positioning**
   
   - Dual-antenna UBLOX-F9P system
   - RTK-enabled for centimeter-level accuracy
   - Format: Custom format

5. **Camera Images** (selected sequences)
   
   - Front pinhole camera (~187° FOV)
   - Format: not included in MCAP files

### 

## Sensor Data in MCAP Files

The odometry MCAP files contain synchronized multi-sensor data in ROS2 message formats. Below are the detailed specifications for each sensor type.

### Custom Message Definitions

For custom data formats (Vessel IMU, GNSS positioning, and dual-antenna heading), we provide the corresponding `.msg` definition files with the dataset. These files can be used to:

- Understand the exact data structure
- Build ROS2 packages with proper message types
- Parse the data correctly in your applications

**Provided Message Definition Files**:

- `Imu.msg` - Vessel IMU data format
- `Gps.msg` - GNSS positioning data format
- `GpsRover.msg` - Dual-antenna heading data format

These files are located in the `data/message_definitions/` directory of the dataset.

### 1. LiDAR Data

#### Livox AVIA LiDAR

- **Message Type**: `sensor_msgs.msg.PointCloud2` (ROS2)
- **Detection Range**: 400 m
- **Mounting Position**: Front of vessel
- **Data Included**: Point cloud data and IMU measurements

**PointCloud2 Field Structure**:

```python
from sensor_msgs.msg import PointCloud2

# Standard ROS2 PointCloud2 message with the following fields:
# Field 0: x (FLOAT32) - X coordinate in meters
# Field 1: y (FLOAT32) - Y coordinate in meters
# Field 2: z (FLOAT32) - Z coordinate in meters
# Field 3: intensity (FLOAT32) - Reflectivity intensity (0-255)
# Field 4: tag (UINT8) - Point tag/label
# Field 5: line (UINT8) - Laser line number
# Field 6: timestamp (FLOAT64) - Point timestamp in seconds

# Coordinate system: Right-hand coordinate system
# - X: Forward
# - Y: Left
# - Z: Up
```

#### Livox MID360 LiDAR

- **Message Type**: `sensor_msgs.msg.PointCloud2` (ROS2)
- **Coverage**: 360° horizontal FOV
- **Purpose**: Capturing nearby scenes
- **Data Included**: Point cloud data and IMU measurements

**PointCloud2 Field Structure**: Same as Livox AVIA (see above)

### 2. IMU Data

#### LiDAR-Integrated IMU

- **Message Type**: `sensor_msgs.msg.Imu` (ROS2)
- **Source**: Integrated with Livox LiDARs
- **Data Fields**: Angular velocity, linear acceleration, orientation

#### Vessel IMU

- **Message Type**: Custom format (`Imu.msg`)
- **Chip**: TDK-Invensense IIM 46234
- **Mounting**: On vessel body
- **Purpose**: High-precision vessel motion measurement

**Custom Imu.msg Format**:

```
std_msgs/Header header

float32 roll   # Roll angle, degrees
float32 pitch  # Pitch angle, degrees
float32 yaw    # Yaw angle, degrees
float32 gx     # Angular velocity x-axis, degrees/s
float32 gy     # Angular velocity y-axis, degrees/s
float32 gz     # Angular velocity z-axis, degrees/s
float32 ax     # Acceleration x-axis, g
float32 ay     # Acceleration y-axis, g
float32 az     # Acceleration z-axis, g
```

### 3. mmWave Radar Data

- **Message Type**: `sensor_msgs.msg.PointCloud2` (ROS2)
- **Radar Units**: 10 × 4D mmWave radars
- **Chip**: TI AWR2944
- **Frequency Band**: 77-81 GHz
- **Mode**: Doppler Division Multiple Access (DDMA)
- **Configuration**: Multiple waveforms and parameters

**PointCloud2 Field Structure**:

```python
# ROS2 PointCloud2 message with 4D radar detection fields:
# Field 0: x (FLOAT32) - X coordinate in meters (range × cos(elevation) × cos(azimuth))
# Field 1: y (FLOAT32) - Y coordinate in meters (range × cos(elevation) × sin(azimuth))
# Field 2: z (FLOAT32) - Z coordinate in meters (range × sin(elevation))
# Field 3: speed (FLOAT32) - Doppler velocity in m/s (radial velocity)
# Field 4: snr (UINT16) - Radar cross-section (RCS)
# Field 5: noise (UINT16) - Radial distance in meters

# Coordinate system: Right-hand coordinate system
# - X: Right 
# - Y: Forward (along radar boresight)
# - Z: Up
# Velocity: Negative = approaching, Positive = receding
```

**4D Radar Advantages**:

- **Range**: Distance to target
- **Azimuth & Elevation**: 3D spatial position
- **Doppler Velocity**: Radial velocity for motion estimation
- **Multiple Waveforms**: Different configurations for various scenarios

### 4. GNSS Data

#### GNSS Positioning Data

- **Message Type**: Custom format (`Gps.msg`)
- **Chip**: UBLOX-F9P (dual-chip configuration)
- **Positioning Mode**: RTK-enabled
- **Accuracy**: Centimeter-level (with RTK near shore)

**Custom Gps.msg Format**:

```
std_msgs/Header header

float64 lat      # Latitude
float64 lng      # Longitude
float64 alt      # Altitude
float32 speed    # Speed, m/s
float32 yaw      # Heading, degrees
float32 vel_n    # North velocity component, m/s
float32 vel_e    # East velocity component, m/s
float32 vel_d    # Down velocity component, m/s
float64 time     # Time
uint8 stars      # Number of satellites (max display 12)
float32 hdop     # Horizontal Dilution of Precision
uint8 mode       # Differential positioning mode
float32 rms      # Root Mean Square error
float32 lat_dev  # Latitude standard deviation
float32 lng_dev  # Longitude standard deviation
float32 alt_dev  # Altitude standard deviation
float32 deviation # Overall positioning accuracy
```

#### Dual-Antenna Heading Data

- **Message Type**: Custom format (`GpsRover.msg`)
- **Purpose**: Moving-base orientation from dual-antenna configuration
- **Accuracy**: Centimeter-level baseline measurement

**Custom GpsRover.msg Format**:

```
std_msgs/Header header

float32 length   # Baseline length, centimeters
float32 yaw      # Heading angle, degrees
float32 acc      # Orientation accuracy, centimeters
uint8 mode       # Orientation mode (0, 4, or 5)
```

**Mode Values**:

- `0`: No fix
- `4`: RTK Float
- `5`: RTK Fixed

### 5. Camera Data (Selected Sequences)

- **Availability**: Front camera images provided in some sequences
- **Camera Type**: Fisheye camera
- **Field of View**: ~187°
- **Purpose**: Visual odometry and visual-inertial odometry
- **Format**: Standard image format

## Data Parsing and Conversion

### MCAP to Frame-Based Data

We provide parsing code to convert MCAP files into frame-based readable data format. This conversion tool extracts synchronized sensor data and organizes it by timestamp for easier processing and analysis.

**Conversion Features**:

- Extracts all sensor data from MCAP files
- Synchronizes data across different sensors
- Organizes data by frames/timestamps
- Converts to standard file formats (CSV, JSON, binary)
- Preserves sensor calibration information

**Usage Example**:

```bash
# Convert MCAP file to frame-based format
python tools/convert_mcap_to_frames.py \
    --input data/odometry/sequence_01.mcap \
    --output data/odometry/sequence_01_frames/ \
    --sensors lidar,imu,gnss,radar
```

**Output Structure**:

```
sequence_01_frames/
├── lidar_avia/
│   ├── frame_000000.pcd
│   ├── frame_000001.pcd
│   └── ...
├── lidar_mid360/
│   ├── frame_000000.pcd
│   └── ...
├── imu/
│   ├── lidar_imu.csv
│   └── vessel_imu.csv
├── radar/
│   ├── frame_000000.pcd
│   └── ...
├── gnss/
│   └── gnss_data.csv
├── timestamps.txt
└── metadata.json
```

### Environmental Conditions

[Content to be added: Distribution of environmental conditions]

| Condition   | Number of Sequences   |
| ----------- | --------------------- |
| Clear       | [To be added]         |
| Cloudy      | [To be added]         |
| Overcast    | [To be added]         |
| Rainy       | [To be added]         |
| Foggy       | [To be added]         |
| Daytime     | [To be added]         |
| ----------- | --------------------- |
| Dusk        | [To be added]         |
| Nighttime   | [To be added]         |
| Foggy       | [To be added]         |
| ----------- | --------------------- |
| Sea State 1 | [To be added]         |
| Sea State 2 | [To be added]         |
| ----------- | --------------------- |