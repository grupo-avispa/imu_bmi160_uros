# imu_bmi160_uros
![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/imu_bmi160_uros)
[![Build](https://github.com/grupo-avispa/imu_bmi160_uros/actions/workflows/build.yml/badge.svg)](https://github.com/grupo-avispa/imu_bmi160_uros/actions/workflows/build.yml)


## Overview

Firmware for ESP32 using [micro-ROS](https://micro.ros.org/) and the **IMU BMI160** module. Integrates FreeRTOS for concurrent multitasking and is built with [PlatformIO](https://platformio.org/). Ideal for distributed IMU data processing applications in robotic systems.

**Keywords:** ROS2, imu, micro-ROS, ESP32, FreeRTOS, PlatformIO

**Author: Alberto Tudela<br />**

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies
- [PlatformIO](https://docs.platformio.org/) (Cross-platform build system),
- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics),
- [micro-ROS](https://micro.ros.org/) (ROS 2 client library for microcontrollers),

## Hardware

### IMU BMI160

Bosch Sensortec 6-axis Inertial Measurement Unit (IMU) combining 3-axis accelerometer and 3-axis gyroscope in a single chip. Ideal for motion tracking and orientation sensing in robotic systems.

**Key Features:**
- **6-axis sensors**: 3-axis accelerometer + 3-axis gyroscope
- **Low power consumption**: Ideal for battery-powered applications
- **Fast data rates**: Up to 100 Hz output frequency (configurable)
- **Multiple interfaces**: I2C and SPI supported
- **High precision**: Superior noise characteristics for accurate motion detection

**Technical Specifications:**

| Specification           | Value                                           |
| ----------------------- | ----------------------------------------------- |
| **Supply Voltage**      | 2.4V to 3.6V (typically 3.3V)                   |
| **Current Consumption** | Standby: ~2.5 µA, Active: ~5-10 mA              |
| **Interface**           | I2C (up to 400 kHz) or SPI (up to 10 MHz)       |
| **Accelerometer Range** | ±2g, ±4g, ±8g, ±16g (selectable)                |
| **Gyroscope Range**     | ±125, ±250, ±500, ±1000, ±2000 dps (selectable) |
| **Output Data Rate**    | 25 Hz to 200 Hz (configurable)                  |
| **Typical Noise**       | Accel: ~180 µg/√Hz, Gyro: ~0.007 °/s/√Hz        |

**Physical Connection (I2C Mode - Recommended):**
- **VDD**: 3.3V power supply
- **GND**: Ground
- **SDA** (GPIO 21): I2C data line
- **SCL** (GPIO 22): I2C clock line
- **INT1** (optional): Interrupt output

Refer to the [BMI160 Product Page](https://www.bosch-sensortec.com/products/inertial-measurement-units/imu/bmi160/) for detailed datasheets and application notes.

## Configuration

### WiFi and Transport Configuration

Edit `include/config_transport.hpp` to set your network parameters:

```cpp
/// WiFi network SSID
static char* WIFI_SSID = "YOUR_SSID";

/// WiFi network password
static char* WIFI_PASSWORD = "YOUR_PASSWORD";

/// Micro-ROS agent IP address
static IPAddress AGENT_IP(192, 168, 0, 186);

/// Micro-ROS agent TCP port
static uint16_t AGENT_PORT = 8888;

/// ROS 2 Domain ID (0-232)
static constexpr uint32_t ROS_DOMAIN_ID = 0;
```

## Building and Flashing

Build the project:
```bash
platformio run
```

Upload to ESP32:
```bash
platformio run --target upload
```

Monitor serial output:
```bash
platformio device monitor --baud 921600
```

## Usage

### 1. Start the micro-ROS Agent

On your ROS 2 enabled machine:

```bash
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyUSB0
# or for WiFi:
ros2 run micro_ros_agent micro_ros_agent udp4 --port 8888
```

### 2. Verify Connection

Monitor device output:
```bash
platformio device monitor --baud 921600
```

You should see:
```
Micro-ROS configuration set...
Serial read task created...
Data process task created...
Micro-ROS task created...
[TIMER] Sync timer callback called
Synchronized timestamp with PC agent
```

## Running the micro-ROS Agent with Docker Compose

A `docker-compose.yml` is provided to easily launch the micro-ROS agent in a containerized environment. Two agent modes are available:

### Available Agent Modes

1. **Serial Agent**: Direct communication via UART/USB (stable, no network needed)
   - Best for: Development, debugging, stable environments
   - Connection: `/dev/ttyUSB0` (or your serial port)

2. **UDP Agent**: Network-based communication over WiFi/Ethernet (flexible, scalable)
   - Best for: Wireless systems, distributed architectures
   - Connection: `localhost:9999` (default)

### Prerequisites
- Docker and Docker Compose installed
- ESP32 device connected to your host machine

### Launch the Agent

**Serial Mode (recommended for testing):**
```bash
docker-compose up -d
```

The serial agent will connect at **921600 bps** for optimal throughput with the BMI160 IMU.

### Configuration

#### Serial Connection (recommended)
Update `include/config_transport.hpp`:
```cpp
set_microros_serial_transports(Serial);
```

#### Network/UDP Connection
Update `include/config_transport.hpp`:
```cpp
set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
```

### Stop the Agent

```bash
docker-compose down
```

### View Logs

```bash
docker-compose logs -f serial-agent0
```

### Verify Connection

After launching the agent and uploading the firmware to ESP32, you should see synchronization messages:
