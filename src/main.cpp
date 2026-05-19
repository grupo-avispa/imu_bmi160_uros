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
 * @file main.cpp
 * @brief Main firmware for ESP32 micro-ROS node with FreeRTOS multitasking.
 *
 * This template provides a foundation for building micro-ROS enabled embedded
 * systems using ESP32 with FreeRTOS. It includes:
 * - Serial communication infrastructure
 * - ROS2 publisher/subscriber setup
 * - Time synchronization with ROS agent
 * - Multi-task architecture for concurrent operations
 */

// Arduino Platform
#include <Arduino.h>
#include <Wire.h>

// Local configuration
#include "config_ros.hpp"
#include "config_transport.hpp"
#include "macros.hpp"

// micro-ROS Libraries
#include <micro_ros_platformio.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/imu.h>
#include <rosidl_runtime_c/primitives_sequence_functions.h>

#include <FastIMU.h>

// ROS2 Context and Executor
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

rcl_timer_t sync_timer;  ///< Timer for periodic time synchronization with agent
rcl_publisher_t imu_publisher;
sensor_msgs__msg__Imu msg;

// Combined IMU data struct for atomic transfer between tasks
struct IMUData {
  AccelData accel;
  GyroData gyro;
};

// FreeRTOS Task Management
QueueHandle_t imu_data_queue;          ///< Queue for inter-task communication
TaskHandle_t i2cIMUTaskHandle;   ///< Handle for I2C IMU read task
TaskHandle_t rosTaskHandle;      ///< Handle for ROS executor task

// Hardware Configuration
constexpr uint32_t IMU_ADDRESS = 0x69;  ///< I2C address for BMI160 IMU
constexpr uint8_t SDA_PIN = 21;  ///< SDA pin for I2C communication
constexpr uint8_t SCL_PIN = 22;  ///< SCL pin for I2C communication

// BMI160 Noise Parameters (datasheet, typical values)
/// Gyroscope variance at ±2000 dps, 32 Hz BW: (0.007 * π/180)² * 32 ≈ 4.8e-7 (rad/s)²
constexpr double GYRO_VARIANCE = 4.8e-7;
/// Accelerometer variance at ±2g, 40 Hz BW: (180e-6 * 9.80665)² * 40 ≈ 1.25e-4 (m/s²)²
constexpr double ACCEL_VARIANCE = 1.25e-4;

// Time Synchronization Variables
/// Timeout for ROS agent sync session in milliseconds
const int SYNC_TIMEOUT_MS = 2000;
/// Synchronized system time in nanoseconds from agent
int64_t ros_synced_time_ns = 0;
/// Local time (microseconds) at last synchronization point
uint64_t micros_before_sync = 0;
/// Mutex to protect time synchronization variables
portMUX_TYPE timeSyncMutex = portMUX_INITIALIZER_UNLOCKED;
/// Flag to indicate if time has been synchronized with agent
volatile bool time_synchronized = false;

// Mutexes for shared data access
SemaphoreHandle_t imu_mutex;  ///< FreeRTOS mutex for IMU data access

BMI160 imu;  ///< BMI160 IMU sensor object
calData cal_data = { 0 };  ///< Calibration data for IMU sensor

/**
 * @brief Task for reading IMU data from I2C communication.
 * @param pvParameters Task parameters (unused).
 */
void I2CIMUTask(void* pvParameters);

/**
 * @brief Task for running the micro-ROS executor and publishing IMU messages.
 * @param pvParameters Task parameters (expected value: 1).
 */
void vTaskMicroROS(void* pvParameters);

/**
 * @brief Callback function for synchronizing time with the ROS agent.
 * @param timer Pointer to the timer object.
 * @param last_call_time Time of the last callback invocation.
 */
void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time);

/**
 * @brief Arduino setup function. Initializes hardware, ROS2, and creates tasks.
 */
void setup() {
  // Initialize serial communication with PC
  Serial.begin(921600);

  // Initialize I2C communication for IMU
  Wire.begin(SDA_PIN, SCL_PIN);
  Serial.println("I2C communication initialized...");

  // Initialize and calibrate the IMU sensor
  uint8_t imu_status = imu.init(cal_data, IMU_ADDRESS);
  if (imu_status != 0) {
    Serial.printf("Failed to initialize IMU sensor, error code: %d\n", imu_status);
    error_loop();
  }
  delay(5000);
  imu.calibrateAccelGyro(&cal_data);
  delay(1000);
  // Re-initialize after calibration
  imu.init(cal_data, IMU_ADDRESS);
  Serial.println("IMU sensor initialized and calibrated...");

  // Configure micro-ROS WiFi transport to connect to ROS agent
  // Uncomment and update the following line with your transport settings
  // (e.g., WiFi credentials, agent IP/port)
  // set_microros_wifi_transports(WIFI_SSID, WIFI_PASSWORD, AGENT_IP, AGENT_PORT);
  set_microros_serial_transports(Serial);
  Serial.println("micro-ROS configuration set...");

  allocator = rcl_get_default_allocator();

  // Initialize ROS2 options and set domain ID
  rcl_init_options_t init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, ROS_DOMAIN_ID));

  // Initialize rclc support object with custom options
  RCCHECK(rclc_support_init_with_options(&support, 0, nullptr, &init_options, &allocator));

  // Create node
  RCCHECK(rclc_node_init_default(&node, ROS_NODE_NAME, "", &support));

  // Create IMU publisher
  RCCHECK(rclc_publisher_init_default(
    &imu_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    ROS_TOPIC_NAME));

  // Initialize synchronization timer (5 seconds)
  RCCHECK(
    rclc_timer_init_default2(&sync_timer, &support, RCL_MS_TO_NS(5000), sync_timer_callback, true));

  // Create ROS2 executor with capacity for 1 timer (the sync timer)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));

  // Add sync timer to executor
  RCCHECK(rclc_executor_add_timer(&executor, &sync_timer));

  // Create FreeRTOS mutex for IMU shared data
  imu_mutex = xSemaphoreCreateMutex();
  if (imu_mutex == nullptr) {
    Serial.println("ERROR: Failed to create IMU mutex");
    error_loop();
  }

  // Create FreeRTOS queue for communication between tasks
  imu_data_queue = xQueueCreate(1, sizeof(IMUData));

  // Create task for reading IMU I2C data (core 0, priority 1)
  xTaskCreatePinnedToCore(
    I2CIMUTask, "I2CIMUTask", 4096, NULL, 1, &i2cIMUTaskHandle, 0);
  Serial.println("I2C IMU read task created...");

  // Create task for micro-ROS executor and IMU publishing (core 1, priority 2)
  xTaskCreatePinnedToCore(
      vTaskMicroROS, "vTaskMicroROS", 10000, (void *)1, 2, &rosTaskHandle, 1);

  if (rosTaskHandle == nullptr) {
    Serial.println("ERROR: Failed to create micro-ROS task");
    error_loop();
  }
  Serial.println("micro-ROS tasks created...");
}

/**
 * @brief Arduino loop function. Main loop is handled by FreeRTOS tasks.
 */
void loop() {
  // Main loop does nothing, tasks handle everything via FreeRTOS
}

void vTaskMicroROS(void* pvParameters) {
  configASSERT(((uint32_t)pvParameters) == 1);
  IMUData imu_data;

  // Initialize covariance matrices once (BMI160 datasheet typical values)
  // Orientation is not available from BMI160 (no onboard fusion)
  msg.orientation_covariance[0] = -1.0;

  // Gyroscope covariance: diagonal matrix, (rad/s)²
  msg.angular_velocity_covariance[0] = GYRO_VARIANCE;
  msg.angular_velocity_covariance[4] = GYRO_VARIANCE;
  msg.angular_velocity_covariance[8] = GYRO_VARIANCE;

  // Accelerometer covariance: diagonal matrix, (m/s²)²
  msg.linear_acceleration_covariance[0] = ACCEL_VARIANCE;
  msg.linear_acceleration_covariance[4] = ACCEL_VARIANCE;
  msg.linear_acceleration_covariance[8] = ACCEL_VARIANCE;

  for (;;) {
    // Spin executor to handle timers (sync timer)
    rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

    // Publish IMU data if time is synchronized and data is available
    if (time_synchronized && xQueueReceive(imu_data_queue, &imu_data, 0) == pdTRUE) {
      portENTER_CRITICAL(&timeSyncMutex);
      int64_t synced_time_ns = ros_synced_time_ns;
      uint64_t sync_micros = micros_before_sync;
      portEXIT_CRITICAL(&timeSyncMutex);

      uint64_t elapsed_micros = micros() - sync_micros;
      int64_t current_time_ns = synced_time_ns + (elapsed_micros * 1000);

      msg.header.stamp.sec = current_time_ns / 1000000000LL;
      msg.header.stamp.nanosec = current_time_ns % 1000000000LL;
      msg.header.frame_id.data = const_cast<char*>(ROS_FRAME_ID);
      msg.header.frame_id.size = strlen(ROS_FRAME_ID);

      msg.linear_acceleration.x = imu_data.accel.accelX * 9.80665f;
      msg.linear_acceleration.y = imu_data.accel.accelY * 9.80665f;
      msg.linear_acceleration.z = imu_data.accel.accelZ * 9.80665f;
      msg.angular_velocity.x = imu_data.gyro.gyroX * (M_PI / 180.0f);
      msg.angular_velocity.y = imu_data.gyro.gyroY * (M_PI / 180.0f);
      msg.angular_velocity.z = imu_data.gyro.gyroZ * (M_PI / 180.0f);

      rcl_publish(&imu_publisher, &msg, NULL);
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

void I2CIMUTask(void* pvParameters) {
  IMUData imu_data;

  while (true) {
    if (xSemaphoreTake(imu_mutex, portMAX_DELAY) == pdTRUE) {
      imu.update();
      imu.getAccel(&imu_data.accel);
      imu.getGyro(&imu_data.gyro);
      xSemaphoreGive(imu_mutex);
    }
    xQueueOverwrite(imu_data_queue, &imu_data);  // Send latest data to publish task
    vTaskDelay(pdMS_TO_TICKS(10));  // Prevent watchdog timer issues
  }
}

void sync_timer_callback(rcl_timer_t* timer, int64_t last_call_time){
  Serial.println("[TIMER] Sync timer callback called");

  if (timer == NULL) {
    Serial.println("Error in timer_callback: timer parameter is nullptr");
    return;
  }

  // Synchronize time with the ROS agent
  rmw_uros_sync_session(SYNC_TIMEOUT_MS);

  if (rmw_uros_epoch_synchronized()) {
    // Get synchronized time in nanoseconds
    int64_t synced_time_ns = rmw_uros_epoch_nanos();
    
    // Acquire mutex to safely update synchronized time
    portENTER_CRITICAL(&timeSyncMutex);
    ros_synced_time_ns = synced_time_ns;
    micros_before_sync = micros();
    time_synchronized = true;
    portEXIT_CRITICAL(&timeSyncMutex);
    
    Serial.printf("Synchronized timestamp with PC agent: %lld ns\n", synced_time_ns);
  } 
  else {
      Serial.println("Error in sync_timer_callback: time not synchronized");
      portENTER_CRITICAL(&timeSyncMutex);
      time_synchronized = false;
      portEXIT_CRITICAL(&timeSyncMutex);
  }
}
