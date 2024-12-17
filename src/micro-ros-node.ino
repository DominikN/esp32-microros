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

#include <NeoPixelBus.h>
#include <micro_ros_arduino.h>
#include <micro_ros_utilities/string_utilities.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/string.h>
#include <sensor_msgs/msg/image.h>


// #include "methods/NeoEsp32I2sMethod.h"

NeoPixelBus<NeoRgbFeature, NeoEsp32LcdX8Ws2811Method> ledStrip(100, 14); // note: older WS2811 and longer strip

rclc_support_t support;
rcl_init_options_t init_options;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_publisher_t publisher;
rcl_subscription_t subscriber;
rcl_subscription_t subscriber_led;
std_msgs__msg__String msg_in;
std_msgs__msg__String msg_out;

sensor_msgs__msg__Image image_msg;

char buffer_out[500];

TickType_t xLastHeartbeatTime;

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

void ImageMsgInit(sensor_msgs__msg__Image *arg_message) {
  size_t number_of_leds = 48;
  // Initialize the header
  arg_message->header.frame_id.data = (char *)malloc(100 * sizeof(char));
  strcpy(arg_message->header.frame_id.data, "led_strip_frame");
  arg_message->header.frame_id.capacity = 100;
  arg_message->header.frame_id.size = strlen(arg_message->header.frame_id.data);

  // Initialize other fields
  arg_message->height = 1;
  arg_message->width = number_of_leds;
  arg_message->encoding.data = (char *)malloc(number_of_leds * sizeof(char));
  strcpy(arg_message->encoding.data, "rgba8");
  arg_message->encoding.capacity = 10;
  arg_message->encoding.size = strlen(arg_message->encoding.data);
  arg_message->is_bigendian = 0;
  arg_message->step = number_of_leds * 4; // 18 * 3 for RGB
  arg_message->data.capacity = number_of_leds * 4;
  arg_message->data.size = number_of_leds * 4;
  arg_message->data.data = (uint8_t *)malloc(number_of_leds * 4 * sizeof(uint8_t));
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

// void subscription_led_callback(const void *msgin)
// {
//   Serial.println("Received image");
//   const sensor_msgs__msg__Image * img = (const sensor_msgs__msg__Image *)msgin;
//   uint8_t StripLength = ledStrip.PixelCount();

//   // Assuming PixelStrip is an object that handles the LED strip
//   for (int i = 0; i < StripLength; i++) {
//     uint8_t red = img->data.data[4 * i];
//     uint8_t green = img->data.data[4 * i + 1];
//     uint8_t blue = img->data.data[4 * i + 2];
//     ledStrip.SetPixelColor(i, RgbColor(red, green, blue));      // red
//   }
//   ledStrip.Show();
//   xLastHeartbeatTime = xTaskGetTickCount();
// }

void subscription_led_callback(const void *msgin)
{
    Serial.println("Received image");
    const sensor_msgs__msg__Image * img = (const sensor_msgs__msg__Image *)msgin;
    uint8_t StripLength = ledStrip.PixelCount(); // This should be 100

    // Assuming the message has fewer pixels (48) than the LED strip (100)
    uint8_t messagePixelCount = 48;

    for (int i = 0; i < StripLength; i++) {
        // Map LED index to message pixel index
        int mappedIndex = (i * messagePixelCount) / StripLength;

        // Ensure we don't exceed the message bounds
        uint8_t red = img->data.data[4 * mappedIndex];
        uint8_t green = img->data.data[4 * mappedIndex + 1];
        uint8_t blue = img->data.data[4 * mappedIndex + 2];

        // Set the LED color based on the mapped message data
        ledStrip.SetPixelColor(i, RgbColor(red, green, blue));
    }

    ledStrip.Show();
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

  RCCHECK(rclc_subscription_init_default(
    &subscriber_led,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Image),
    "panther/lights/channel_1_frame"));
  ImageMsgInit(&image_msg); 

  // create timer,
  RCCHECK(rclc_timer_init_default2(
      &timer,
      &support,
      RCL_MS_TO_NS(500),
      timer_callback,
      true));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber, &msg_in, &subscription_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(&executor, &subscriber_led, &image_msg, &subscription_led_callback, ON_NEW_DATA));

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
  RCSOFTCHECK(rcl_subscription_fini(&subscriber_led, &node));
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

  ledStrip.Begin();

  // for(int i=0; i<100; i++) {
  //   ledStrip.SetPixelColor(i, RgbColor(20, 20, 20));      // red
  // }
  // ledStrip.Show();

  Serial.printf("Connecting to WiFi (ssid: %s, password: %s)...", ssid, password);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  if (WiFi.waitForConnectResult() != WL_CONNECTED) {
    Serial.printf("WiFi connection failure (err: %d)\n", WiFi.status());
    delay(5000);
    ESP.restart();
  }

  Serial.print("Local IP: ");
  Serial.println(WiFi.localIP());

  Serial.printf("Starting micro_ros (ssid: %s, password: %s)...", ssid, password);
  // set_microros_wifi_transports((char *)ssid, (char *)password, (char *)microRosAgentIP, AGENT_PORT);

  set_microros_wifi_transports_2((char *)microRosAgentIP, AGENT_PORT);

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

  if (xLastWakeTime - xLastHeartbeatTime > pdMS_TO_TICKS(5000))
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
