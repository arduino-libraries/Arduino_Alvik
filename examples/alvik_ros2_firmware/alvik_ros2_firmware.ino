/*
 * Copyright (c) 2024 Arduino
 *
 * SPDX-License-Identifier: MPL-2.0
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

/**************************************************************************************
 * INCLUDE
 **************************************************************************************/

#include <Arduino_Alvik.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

#include <micro_ros_utilities/string_utilities.h>

#include "wifi_secrets.h"
#include "micro_ros_config.h"

/**************************************************************************************
 * GLOBAL VARIABLES
 **************************************************************************************/

static rcl_subscription_t cmd_vel_sub;
static geometry_msgs__msg__Twist cmd_vel_msg;

static rcl_timer_t odom_timer;
static rcl_publisher_t odom_pub;
static nav_msgs__msg__Odometry odom_msg;

static rclc_support_t support;
static rcl_allocator_t allocator = rcl_get_default_allocator();
static rclc_executor_t executor = rclc_executor_get_zero_initialized_executor();
static rcl_node_t node = rcl_get_zero_initialized_node();

static Arduino_Alvik alvik;

/**************************************************************************************
 * LOCAL FUNCTIONS
 **************************************************************************************/

void error_loop(char const * fmt, ...)
{
  va_list args;
  va_start(args, fmt);

  char msg[64] = {0};
  vsnprintf(msg, sizeof(msg), fmt, args);
  Serial.println(msg);

  va_end(args);

  for(;;)
  {
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

void euler_to_quat(float const x, float const y, float const z, double * q)
{
  float c1 = cos((y*3.14f/180.f)/2.f);
  float c2 = cos((z*3.14f/180.f)/2.f);
  float c3 = cos((x*3.14f/180.f)/2.f);

  float s1 = sin((y*3.14f/180.f)/2.f);
  float s2 = sin((z*3.14f/180.f)/2.f);
  float s3 = sin((x*3.14f/180.f)/2.f);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

void cmd_vel_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  alvik.drive(msg->linear.x, msg->angular.z, M_S, RAD_S);
}

void odom_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    struct timespec ts;
    clock_gettime(CLOCK_REALTIME, &ts);
    odom_msg.header.stamp.sec = ts.tv_sec;
    odom_msg.header.stamp.nanosec = ts.tv_nsec;

    odom_msg.header.frame_id = micro_ros_string_utilities_init("odom");

    float x = 0.f, y = 0.f, theta = 0.f;
    alvik.get_pose(x, y, theta, M, RAD);

    odom_msg.pose.pose.position.x  = x;
    odom_msg.pose.pose.position.y  = y;
    odom_msg.pose.pose.position.z  = 0.f;

    double q[4] = {0.};
    euler_to_quat(0., 0., theta, q);
    odom_msg.pose.pose.orientation.x = q[1];
    odom_msg.pose.pose.orientation.y = q[2];
    odom_msg.pose.pose.orientation.z = q[3];
    odom_msg.pose.pose.orientation.w = q[0];

    float linear = 0.f, angular = 0.f;
    alvik.get_drive_speed(linear, angular, M_S, RAD_S);

    odom_msg.twist.twist.linear.x  = linear;
    odom_msg.twist.twist.linear.y  = 0.f;
    odom_msg.twist.twist.angular.z = angular;

    if (rcl_ret_t const rc = rcl_publish(&odom_pub, &odom_msg, NULL); rc != RCL_RET_OK)
      error_loop("odom_timer_callback::rcl_publish failed with %d", rc);
  }
}

/**************************************************************************************
 * SETUP/LOOP
 **************************************************************************************/

void setup()
{
  Serial.begin();
  for (auto const start = millis(); !Serial && (millis() - start) < 1000; ) { }

  set_microros_wifi_transports(SECRET_WIFI_SSID, SECRET_WIFI_PASS, MICRO_ROS_AGENT_IP, MICRO_ROS_AGENT_PORT);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  alvik.begin();

  if (rcl_ret_t const rc = rclc_support_init(&support, 0, NULL, &allocator); rc != RCL_RET_OK)
    error_loop("rclc_support_init failed with %d", rc);

  if (rcl_ret_t const rc = rclc_node_init_default(&node, "alvik", "", &support); rc != RCL_RET_OK)
    error_loop("rclc_node_init_default failed with %d", rc);

  if (rcl_ret_t const rc = rclc_subscription_init_default(&cmd_vel_sub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel"); rc != RCL_RET_OK)
    error_loop("rclc_subscription_init_default failed with %d", rc);

  if (rcl_ret_t const rc = rclc_timer_init_default(&odom_timer, &support, RCL_MS_TO_NS(50), odom_timer_callback); rc != RCL_RET_OK)
    error_loop("rclc_timer_init_default(odom_timer, ...) failed with %d", rc);

  if (rcl_ret_t const rc = rclc_publisher_init_default(&odom_pub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "/odom"); rc != RCL_RET_OK)
    error_loop("rclc_publisher_init_default(odom_pub, ...) failed with %d", rc);

  if (rcl_ret_t const rc = rclc_executor_init(&executor, &support.context, 2, &allocator); rc != RCL_RET_OK)
    error_loop("rclc_executor_init failed with %d", rc);

  if (rcl_ret_t const rc = rclc_executor_add_subscription(&executor, &cmd_vel_sub, &cmd_vel_msg, &cmd_vel_callback, ON_NEW_DATA); rc != RCL_RET_OK)
    error_loop("rclc_executor_add_subscription(..., cmd_vel_sub, ...) failed with %d", rc);

  if (rcl_ret_t const rc = rclc_executor_add_timer(&executor, &odom_timer); rc != RCL_RET_OK)
    error_loop("rclc_executor_add_timer(..., odom_timer, ...) failed with %d", rc);

  Serial.println("alvik_ros2_firmware setup complete.");
}

void loop()
{
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
