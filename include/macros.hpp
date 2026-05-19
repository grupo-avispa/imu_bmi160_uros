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
 * @file macros.hpp
 * @brief Utility macros for error handling and logging in micro-ROS ESP32 firmware.
 *
 * This header defines macros for checking the return values of ROS2 functions,
 * as well as an error loop function that halts execution if a critical error occurs.
 *
 */

#ifndef MACROS_HPP
#define MACROS_HPP

#include <Arduino.h>

/**
 * @brief Macro for checking ROS2 function return values.
 *        Enters error loop if the function fails.
 * @param fn The ROS2 function to check.
 */
#define RCCHECK(fn)                  \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK)) {   \
      error_loop();                  \
    }                                \
  }

/**
 * @brief Soft version of RCCHECK that logs but doesn't stop execution.
 * @param fn The ROS2 function to check.
 */
#define RCSOFTCHECK(fn)              \
  {                                  \
    rcl_ret_t temp_rc = fn;          \
    if ((temp_rc != RCL_RET_OK)) {   \
    }                                \
  }

/**
 * @brief Infinite error loop function. If something fails, the device stops here.
 *        Flashes the device in a continuous loop.
 */
void error_loop() {
  while (true) {
    delay(100);
  }
}

#endif // MACROS_HPP
