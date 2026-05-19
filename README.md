# gnss_air530z_uros
![ROS2](https://img.shields.io/badge/ros2-jazzy-blue?logo=ros&logoColor=white)
![License](https://img.shields.io/github/license/grupo-avispa/gnss_air530z_uros)


## Overview

Firmware for ESP32 using [micro-ROS](https://micro.ros.org/) and the **Grove GPS Air530** module. Integrates FreeRTOS for concurrent multitasking and is built with [PlatformIO](https://platformio.org/). Ideal for distributed GNSS positioning applications in robotic systems.

**Keywords:** ROS2, gnss, positioning, micro-ROS, ESP32, FreeRTOS, PlatformIO

**Author: Alberto Tudela<br />**

This is research code, expect that it changes often and any fitness for a particular purpose is disclaimed.

## Installation

### Building from Source

#### Dependencies
- [PlatformIO](https://docs.platformio.org/) (Cross-platform build system),
- [Robot Operating System (ROS) 2](https://docs.ros.org/en/jazzy/) (middleware for robotics),
- [micro-ROS](https://micro.ros.org/) (ROS 2 client library for microcontrollers),

## Hardware

### Grove GPS Air530
High-performance multi-mode GNSS module for precise positioning:

| Specification            | Value                                     |
| ------------------------ | ----------------------------------------- |
| **Power Supply Voltage** | 3.3V/5V                                   |
| **Operating Current**    | Up to 60mA                                |
| **Warm Start Time**      | 4s                                        |
| **Cold Start Time**      | 30s                                       |
| **Baudrate**             | 9600 bps                                  |
| **Protocol**             | NMEA                                      |
| **Positioning Modes**    | GPS, Beidou, GLONASS, Galileo, QZSS, SBAS |

**Physical Connection:**
- Red: 5V (or 3.3V)
- Black: GND
- Yellow: RX (GPIO)
- White: TX (GPIO)

Refer to the [GPS Air530 documentation](https://wiki.seeedstudio.com/Grove-GPS-Air530/) for more details.

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
platformio device monitor --baud 115200
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
platformio device monitor --baud 115200
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
docker-compose -f docker-compose.yml -f docker-compose.serial.yml up
```

**UDP Mode (WiFi/Network):**
```bash
docker-compose up
```

Or explicitly specify UDP mode:
```bash
docker-compose -f docker-compose.yml -f docker-compose.udp.yml up
```

### Configuration

Update the firmware to match your chosen agent mode:

**For Serial Connection** - In `include/config_transport.hpp`:
```cpp
set_microros_serial_transports(Serial);
```

**For UDP/WiFi Connection** - In `include/config_transport.hpp`:
```cpp
set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
```

### Stop the Agent

```bash
docker-compose down
```

### View Logs

```bash
docker-compose logs -f
```

### Verify Connection

After launching the agent and uploading the firmware to ESP32, you should see synchronization messages in both the device output and agent logs.
