// Copyright (c) 2026 Alberto J. Tudela Roldán
// Copyright (c) 2026 Grupo Avispa, DTE, Universidad de Málaga
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/**
 * @file config_transport.hpp
 * @brief Transport and network configuration for micro-ROS WiFi communication.
 *
 * This header file contains the WiFi SSID, password, and ROS agent connection
 * parameters. Update these values to match your network and agent setup.
 *
 * @warning This file contains sensitive information (WiFi credentials).
 *          Do not commit to public repositories.
 */

#ifndef CONFIG_TRANSPORT_HPP
#define CONFIG_TRANSPORT_HPP

#include <IPAddress.h>
#include <cstdint>

/**
 * @defgroup WiFiConfig WiFi and ROS2 Configuration Parameters
 * @{
 */

/// WiFi network SSID (network name)
static char WIFI_SSID[] = "YOUR_SSID";

/// WiFi network password (WPA/WPA2)
static char WIFI_PASSWORD[] = "YOUR_PASSWORD";

/// IP address of the micro-ROS agent
static IPAddress AGENT_IP(192, 168, 0, 0);

/// TCP port number for micro-ROS agent communication
static size_t AGENT_PORT = 9999;

/** @} */

#endif // CONFIG_TRANSPORT_HPP
