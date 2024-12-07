#if __has_include("credentials.h")

// For local development (rename credenials-template.h and type your WiFi and
// Husarnet credentials there)
#include "credentials.h"

#else

// WiFi credentials
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

// Husarnet credentials
const char *hostName = HUSARNET_HOSTNAME;
const char *husarnetJoinCode = HUSARNET_JOINCODE; // find at app.husarnet.com

#endif
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>


// #include <std_msgs/msg/int32.h>
#include <std_msgs/msg/string.h>

#if !defined(ESP32) && !defined(TARGET_PORTENTA_H7_M7) && !defined(ARDUINO_GIGA) && !defined(ARDUINO_NANO_RP2040_CONNECT) && !defined(ARDUINO_WIO_TERMINAL) && !defined(ARDUINO_UNOR4_WIFI) && !defined(ARDUINO_OPTA)
#error This example is only available for Arduino Portenta, Arduino Giga R1, Arduino Nano RP2040 Connect, ESP32 Dev module, Wio Terminal, Arduino Uno R4 WiFi and Arduino OPTA WiFi
#endif

rcl_publisher_t publisher;
std_msgs__msg__String msg;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

char buffer[500];

#define AGENT_PORT 8888
#define AGENT_IP "192.168.0.151"
#define NODE_NAME "talker_esp32"

#if defined(LED_BUILTIN)
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif

#define RCCHECK(fn)              \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
      error_loop();              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

void error_loop()
{
  while (1)
  {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

// void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
// {
//   RCLC_UNUSED(last_call_time);
//   if (timer != NULL)
//   {
//     RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//     msg.data++;
//   }
// }

void setup()
{
  Serial.begin(115200);

  Serial.printf("Starting micro_ros (ssid: %s, password: %s)...", ssid, password);
  set_microros_wifi_transports((char *)ssid, (char *)password, AGENT_IP, AGENT_PORT);
  Serial.printf("done\n");


  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_best_effort(
      &publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter"));

  RCCHECK(rmw_uros_sync_session(5000));

  Serial1.printf("done, sys time = %d\r\n", rmw_uros_epoch_millis());
}

void loop(void) {
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  static int cnt = 0;
  // sprintf(buffer, "Hello World: %d, sys_clk: %d", cnt++,
  // xTaskGetTickCount());
  sprintf(buffer, "Hello World: %d", cnt++);
  Serial1.printf("Publishing: \"%s\" [free heap: %d]\r\n", buffer,
                 xPortGetFreeHeapSize());

  msg.data = micro_ros_string_utilities_set(msg.data, buffer);

  RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));

  micro_ros_string_utilities_destroy(&(msg.data));
  vTaskDelayUntil(&xLastWakeTime, 1000);
}

// int i = 0;

// void loop()
// {
//   RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
//   msg.data++;
//   Serial.printf("loop %d", i++);
// }