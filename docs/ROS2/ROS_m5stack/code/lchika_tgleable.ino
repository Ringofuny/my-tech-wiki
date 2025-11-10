//
// micro-ROS subscriber & publisher sample for M5Stack
//
#include <M5Stack.h>
#include <micro_ros_arduino.h>

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_subscription_t subscriber;
rcl_publisher_t publisher;

std_msgs__msg__Int32 sub_msg;   // Subscriber 用
std_msgs__msg__Int32 pub_msg;   // Publisher 用

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

#define LED_PIN 16

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}

void error_loop(){
  M5.Lcd.print("Error!!\n");
}

void subscription_callback(const void * msgin)
{  
  const std_msgs__msg__Int32 * msg = (const std_msgs__msg__Int32 *)msgin;

  // LED ON/OFF
  digitalWrite(LED_PIN, msg->data);

  // LCD表示
  M5.Lcd.printf("Recv: %d\n", msg->data);
}

void setup() {
  M5.begin();
  M5.Power.begin();
  M5.Lcd.setTextSize(2); 
  M5.Lcd.print("Hello micro-ROS\n");
  M5.Lcd.print("/micro_ros_arduino_subscriber\n");
  
  set_microros_transports();  // USB MAC <-> M5Stack
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);

  delay(2000);

  allocator = rcl_get_default_allocator();

  // init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_subscriber"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "togle_topic"));

  // executor (1 subscription)
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_subscription(
    &executor, &subscriber, &sub_msg, &subscription_callback, ON_NEW_DATA));
}

void loop() {
  delay(50);

  // run executor
  RCCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));

  M5.update();

  // Aボタン: STOP (0b0111)
  if (M5.BtnA.wasPressed()) {
    pub_msg.data = 0b0111;
    M5.Lcd.print("Send: STOP\n");
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }

  // Bボタン: START (0b1000)
  if (M5.BtnB.wasPressed()) {
    pub_msg.data = 0b1000;
    M5.Lcd.print("Send: START\n");
    RCSOFTCHECK(rcl_publish(&publisher, &pub_msg, NULL));
  }
}
