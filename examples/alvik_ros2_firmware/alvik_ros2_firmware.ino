/*
 * Copyright (c) 2024 Arduino
 *
 * SPDX-License-Identifier: MPL-2.0
 * This Source Code Form is subject to the terms of the Mozilla Public
 * License, v. 2.0. If a copy of the MPL was not distributed with this
 * file, You can obtain one at http://mozilla.org/MPL/2.0/.
 */

#include <Arduino_Alvik.h>

#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

#include <geometry_msgs/msg/twist.h>

#include "wifi_secrets.h"

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL)
#error This example is only available for Arduino Portenta, Arduino Nano RP2040 Connect, ESP32 Dev module and Wio Terminal
#endif

rcl_subscription_t subscriber;
geometry_msgs__msg__Twist msg;
//rcl_publisher_t publisher;
//std_msgs__msg__Int32 msg;
rclc_support_t support;
rcl_allocator_t allocator;
rclc_executor_t executor;
rcl_node_t node;

Arduino_Alvik alvik;

void error_loop(){
  while(1){
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
    delay(100);
  }
}

//twist message cb
void subscription_callback(const void *msgin)
{
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;

  char dbg_msg[64] = {0};
  snprintf(dbg_msg, sizeof(dbg_msg), "linear.x = %.2f", msg->linear.x);
  Serial.println(dbg_msg);

  float const left_rpm  = msg->linear.x * 100 - msg->angular.z * 50;
  float const right_rpm = msg->linear.x * 100 + msg->angular.z * 50;

  alvik.set_wheels_speed(left_rpm, right_rpm, RPM);
}

//void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
//{
//  RCLC_UNUSED(last_call_time);
//  if (timer != NULL) {
//    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//    msg.data++;
//  }
//}

void setup() {

  Serial.begin();
  for (auto const start = millis(); !Serial && (millis() - start) < 1000; ) { }

  set_microros_wifi_transports(SECRET_WIFI_SSID, SECRET_WIFI_PASS, "192.168.39.167", 8888);

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  alvik.begin();

  allocator = rcl_get_default_allocator();

  //create init_options
  rcl_ret_t rc = RCL_RET_OK;
  rc = rclc_support_init(&support, 0, NULL, &allocator);
  if (rc != RCL_RET_OK)
  {
    char msg[64] = {0};
    snprintf(msg, sizeof(msg), "rclc_support_init failed with %d", rc);
    Serial.println(msg);
    error_loop();
  }

  // create node
  rc = rclc_node_init_default(&node, "alvik_node", "", &support);
  if (rc != RCL_RET_OK)
  {
    char msg[64] = {0};
    snprintf(msg, sizeof(msg), "rclc_node_init_default failed with %d", rc);
    Serial.println(msg);
    error_loop();
  }

  // create publisher
//  rc = rclc_publisher_init_best_effort(&publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32), "topic_name");
//  if (rc != RCL_RET_OK)
//  {
//    char msg[64] = {0};
//    snprintf(msg, sizeof(msg), "rclc_publisher_init_best_effort failed with %d", rc);
//    Serial.println(msg);
//    error_loop();
//  }

  rc = rclc_subscription_init_default(&subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "/cmd_vel");
  if (rc != RCL_RET_OK)
  {
    char msg[64] = {0};
    snprintf(msg, sizeof(msg), "rclc_subscription_init_default failed with %d", rc);
    Serial.println(msg);
    error_loop();
  }

  rc = rclc_executor_init(&executor, &support.context, 1, &allocator);
  if (rc != RCL_RET_OK)
  {
    char msg[64] = {0};
    snprintf(msg, sizeof(msg), "rclc_executor_init failed with %d", rc);
    Serial.println(msg);
    error_loop();
  }

  rc = rclc_executor_add_subscription(&executor, &subscriber, &msg, &subscription_callback, ON_NEW_DATA);
  if (rc != RCL_RET_OK)
  {
    char msg[64] = {0};
    snprintf(msg, sizeof(msg), "rclc_subscription_init_default failed with %d", rc);
    Serial.println(msg);
    error_loop();
  }

//  msg.data = 0;

  Serial.println("Initialisation complete.");
}

void loop()
{
//  static auto prev = millis();
//  auto const now = millis();
//
//  if ((now - prev) > 1000)
//  {
//    prev = now;
//    Serial.println(msg.data);
//    rcl_publish(&publisher, &msg, NULL);
//    msg.data++;
//  }
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
}
