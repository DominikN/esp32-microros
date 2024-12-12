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

const char *husarnetHostname = "esp32test";
const char *husarnetJoincode = "fc94:b01d:1803:8dd8:b293:5c7d:7639:932a/xxxxxxxxxxxxxxxxxx";

#endif
#include <micro_ros_arduino.h>

#include <micro_ros_utilities/string_utilities.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>

#include <husarnet.h>

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

TickType_t xLastHeartbeatTime;

HusarnetClient husarnet;

#define AGENT_PORT 8888
#define NODE_NAME "talker_esp32"
#define ROS_NAMESPACE ""
#define DOMAIN_ID 255 // 255 - use domain id from agent



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
      return false;              \
    }                            \
  }
#define RCSOFTCHECK(fn)          \
  {                              \
    rcl_ret_t temp_rc = fn;      \
    if ((temp_rc != RCL_RET_OK)) \
    {                            \
    }                            \
  }

// override the set_microros_wifi_transports included from micro_ros_arduino
void set_microros_wifi_transports_2(char * agent_ip, uint32_t agent_port){
	static struct micro_ros_agent_locator locator;
	locator.address.fromString(agent_ip);
	locator.port = agent_port;

	rmw_uros_set_custom_transport(
		false,
		(void *) &locator,
		arduino_wifi_transport_open,
		arduino_wifi_transport_close,
		arduino_wifi_transport_write,
		arduino_wifi_transport_read
	);
}

void publishHelloWorld()
{
  static int cnt = 0;

  sprintf(buffer_out, "Hello World: %d", cnt++);
  // Serial.printf("Publishing: \"%s\" [free heap: %d]\r\n", buffer,
  //              xPortGetFreeHeapSize());

  msg_out.data = micro_ros_string_utilities_set(msg_out.data, buffer_out);

  RCSOFTCHECK(rcl_publish(&publisher, &msg_out, NULL));

  micro_ros_string_utilities_destroy(&(msg_out.data));
}

void timer_callback(rcl_timer_t *timer, int64_t last_call_time)
{
  RCLC_UNUSED(last_call_time);
  if (timer != NULL)
  {
    publishHelloWorld();
  }
}

void subscription_callback(const void *msgin)
{
  const std_msgs__msg__String *msg = (const std_msgs__msg__String *)msgin;
  // Serial.printf("I heard: [%s]\r\n", msg->data.data);

  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
  xLastHeartbeatTime = xTaskGetTickCount();
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
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_in, &subscription_callback, ON_NEW_DATA));

  if (RMW_RET_OK != rmw_uros_sync_session(2000))
  {
    Serial.printf("Failed to sync session");
  }

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
  RCSOFTCHECK(rcl_init_options_fini(&init_options));
  RCSOFTCHECK(rclc_support_fini(&support));

  micro_ros_string_utilities_destroy(&(msg_in.data));
}

void publisherTask(void *pvParameters)
{
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  while (1)
  {
    if (pdFALSE == xTaskDelayUntil(&xLastWakeTime, 500))
    {
      Serial.printf("publisher loop delayed: %d\r\n", xTaskGetTickCount() - xLastWakeTime);
      xLastWakeTime = xTaskGetTickCount();
    }
    publishHelloWorld();
  }
}

// ======================================================
// Setup and loop
// ======================================================

void setup()
{
  Serial.begin(115200);


  // =============== Husarnet =================

  // Connect to the WiFi network
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  Serial.println("Connecting to WiFi");
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi connection failure (err: %d)\n", WiFi.status());
    delay(5000);
    ESP.restart();
  }

  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  // Join the Husarnet network
  husarnet.join(husarnetHostname, husarnetJoincode);

  while (!husarnet.isJoined())
  {
    Serial.println("Waiting for Husarnet network...");
    delay(1000);
  }
  Serial.println("Husarnet network joined");

  Serial.print("Husarnet IP: ");
  Serial.println(husarnet.getIpAddress().c_str());

  // ==========================================

  Serial.printf("Starting micro_ros (ssid: %s, password: %s)...", ssid, password);
  // set_microros_wifi_transports((char *)ssid, (char *)password, (char *)microRosAgentIP, AGENT_PORT);
  // set_microros_wifi_transports((char *)ssid, (char *)password, (char *)"fc94:35c6:6537:085c:8053:bf06:45c2:bc04", AGENT_PORT);

  set_microros_wifi_transports_2((char *)"fc94:35c6:6537:085c:8053:bf06:45c2:bc04", AGENT_PORT);

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  delay(2000);

  Serial.printf("done, sys time = %d\r\n", rmw_uros_epoch_millis());

  create_entities();
  xLastHeartbeatTime = xTaskGetTickCount();

  // xTaskCreate(publisherTask, "publisherTask", 8192, NULL, 1, NULL);
}

void loop()
{
  static TickType_t xLastWakeTime = xTaskGetTickCount();
  static int cnt = 0;

  if (pdFALSE == xTaskDelayUntil(&xLastWakeTime, 10))
  {
    Serial.printf("loop delayed: %d\r\n", xTaskGetTickCount() - xLastWakeTime);
    xLastWakeTime = xTaskGetTickCount();
  }

  if (xLastWakeTime - xLastHeartbeatTime > pdMS_TO_TICKS(10000))
  {
    Serial.printf("No heartbeat received for 5s, restarting...");
    destroy_entities();
    while (false == create_entities())
    {
      destroy_entities();
      vTaskDelay(2000);
      Serial.printf("Failed to create entities, retrying...");
    }
    Serial.printf("Reconnection success, sys time = %d\r\n", rmw_uros_epoch_millis());

    if (RMW_RET_OK != rmw_uros_sync_session(2000))
    {
      Serial.printf("Failed to sync session");
    }

    xLastHeartbeatTime = xTaskGetTickCount();
  }

  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5));

  // Serial.printf("Free heap: %d, hb: %d, last: %d, diff: %d\r\n", ESP.getFreeHeap(), xLastHeartbeatTime, xLastWakeTime, xLastWakeTime - xLastHeartbeatTime);
};

//   if (state == AGENT_CONNECTED) {
//     digitalWrite(LED_PIN, 1);
//   } else {
//     digitalWrite(LED_PIN, 0);
//   }
// }
