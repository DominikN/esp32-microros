#if __has_include("credentials.h")

// For local development (rename credenials-template.h and type your WiFi and
// Husarnet credentials there)
#include "credentials.h"

#else

// WiFi credentials
const char *ssid = WIFI_SSID;
const char *password = WIFI_PASS;

// MicroROS Agent IP
const char *microRosAgentIP = MICROROS_AGENT_IP;

#endif
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
std_msgs__msg__String msg_in;
std_msgs__msg__String msg_out;

char buffer_out[500];

#define AGENT_PORT 8888
#define NODE_NAME "talker_esp32"
#define ROS_NAMESPACE ""
#define DOMAIN_ID 255 // 255 - use domain id from agent

#if defined(LED_BUILTIN)
#define LED_PIN LED_BUILTIN
#else
#define LED_PIN 13
#endif

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){return false;}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

enum states
{
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    static int cnt = 0;

    sprintf(buffer_out, "Hello World: %d", cnt++);
    // Serial.printf("Publishing: \"%s\" [free heap: %d]\r\n", buffer,
    //              xPortGetFreeHeapSize());

    msg_out.data = micro_ros_string_utilities_set(msg_out.data, buffer_out);

    RCSOFTCHECK(rcl_publish(&publisher, &msg_out, NULL));

    micro_ros_string_utilities_destroy(&(msg_out.data));
  }
}

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  Serial.printf("I heard: [%s]\r\n", msg->data.data);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

// Functions create_entities and destroy_entities can take several seconds.
// In order to reduce this rebuild the library with
// - RMW_UXRCE_ENTITY_CREATION_DESTROY_TIMEOUT=0
// - UCLIENT_MAX_SESSION_CONNECTION_ATTEMPTS=3
bool create_entities()
{
  allocator = rcl_get_default_allocator();

  // create init_options
  init_options = rcl_get_zero_initialized_init_options();
  RCCHECK(rcl_init_options_init(&init_options, allocator));
  RCCHECK(rcl_init_options_set_domain_id(&init_options, DOMAIN_ID));
  RCCHECK(rclc_support_init_with_options(&support, 0, NULL, &init_options, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, NODE_NAME, ROS_NAMESPACE, &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
      &publisher,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter"));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
      &subscriber,
      &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter2"));
  msg_in.data = micro_ros_string_utilities_init_with_size(500);

  // create timer,
  RCCHECK(rclc_timer_init_default2(
      &timer,
      &support,
      RCL_MS_TO_NS(500),
      timer_callback,
      true));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 5, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_in, &subscription_callback, ON_NEW_DATA));

  // RCCHECK(rmw_uros_sync_session(5000));

  return true;
}

void destroy_entities()
{
  rmw_context_t *rmw_context = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_subscription_fini(&subscriber, &node));
  RCSOFTCHECK(rcl_publisher_fini(&publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void setup()
{
  Serial.begin(115200);

  Serial.printf("Starting micro_ros (ssid: %s, password: %s)...", ssid, password);
  set_microros_wifi_transports((char *)ssid, (char *)password, (char *)microRosAgentIP, AGENT_PORT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  state = WAITING_AGENT;

  Serial.printf("done, sys time = %d\r\n", rmw_uros_epoch_millis());

  create_entities();
}

int start = 0;
rmw_ret_t ret;

void loop()
{
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  vTaskDelayUntil(&xLastWakeTime, 10);
  ret = rclc_executor_spin_some(&executor, 0);
}

// void loop() {
//   static TickType_t xLastWakeTime = xTaskGetTickCount();
//   static int cnt = 0;
//   vTaskDelayUntil(&xLastWakeTime, 10);
//   switch (state) {
//     case WAITING_AGENT:
//       EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(400, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
//       break;
//     case AGENT_AVAILABLE:
//       state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
//       if (state == WAITING_AGENT) {
//         destroy_entities();
//       };
//       break;
//     case AGENT_CONNECTED:
//       EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(400, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
//       if (state == AGENT_CONNECTED) {
//         // rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100));
//         rclc_executor_spin_some(&executor, 0);
//       }
//       break;
//     case AGENT_DISCONNECTED:
//       destroy_entities();
//       state = WAITING_AGENT;
//       break;
//     default:
//       break;
//   }

//   if (state == AGENT_CONNECTED) {
//     digitalWrite(LED_PIN, 1);
//   } else {
//     digitalWrite(LED_PIN, 0);
//   }
// }

