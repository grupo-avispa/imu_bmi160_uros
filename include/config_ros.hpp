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
 * @file config_ros.hpp
 * @brief ROS 2 configuration parameters for micro-ROS node.
 *
 * This header file contains the configuration parameters for the micro-ROS node,
 * including the domain ID for network isolation.
 */

#ifndef CONFIG_ROS_HPP
#define CONFIG_ROS_HPP

    /**
 * @defgroup ROS2 Configuration Parameters
 * @{
 */

/// ROS 2 Domain ID for network isolation (0-232)
static constexpr uint32_t ROS_DOMAIN_ID = 0;

/// ROS 2 Node name
static char ROS_NODE_NAME[] = "imu_node";

/// ROS 2 topic name for IMU messages
static char ROS_TOPIC_NAME[] = "imu/data";

/// Frame ID for IMU messages
static char ROS_FRAME_ID[] = "imu_link";

/** @} */

#endif // CONFIG_ROS_HPP
