# マイコン->pc

- /dev/ttyUSB0が出ない **->** /dev/ttyACM0で解決

## M5Stackのコードを用意(Arduino IDE)
~~~cpp
#include <micro_ros_arduino.h> 

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>

rcl_publisher_t publisher;
std_msgs__msg__Int32 msg;
rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;
rcl_timer_t timer;

#define LED_PIN 13

#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}


void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}

void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}

void setup() {
  set_microros_transports();
  
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);  
  
  delay(2000);

  allocator = rcl_get_default_allocator();

  //create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
    "micro_ros_arduino_node_publisher"));

  // create timer,
  const unsigned int timer_timeout = 1000;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    timer_callback));

  // create executor
  RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));

  msg.data = 0;
}

void loop() {
  delay(100);
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
}
~~~

[micoroROSのヘッダー(.zipライブラリ)](https://github.com/micro-ROS/micro_ros_arduino/releases/tag/v2.0.8-humble)

1. ライブラリのインクルード

~~~cpp 
#include <micro_ros_arduino.h> 

#include <stdio.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

#include <std_msgs/msg/int32.h>
~~~

2. エラー処理

~~~cpp
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
~~~

- エラーがあれば無限ループでled点滅

~~~cpp
void error_loop(){
  while(1){
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);
  }
}
~~~

3. タイマーコールバック
- 一秒ごとに送信データを１ずつ増やして送信

~~~cpp
void timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{  
  RCLC_UNUSED(last_call_time);
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&publisher, &msg, NULL));
    msg.data++;
  }
}
~~~

4. setup
- micro-ROSトランスポート初期化、デフォルトではシリアル通信(wifiもある)

~~~cpp
set_microros_transports();
~~~

- allocator を使ってノードやタイマー、パブリッシャなどの初期化時にメモリを安全に確保する
構造体ごと代入?

~~~cpp
allocator = rcl_get_default_allocator();
~~~

- ノード、パブリッシャ、タイマー、エグゼキュータを作成
- RCCHECKは関数が正常終了したかを確認する便利マクロ
- ROSIDL_GET_MSG_TYPE_SUPPORT は メッセージ型の情報をrclに渡すためのマクロ

~~~cpp
//create init_options
RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

// create node
RCCHECK(rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support));

// create publisher
RCCHECK(rclc_publisher_init_default(
&publisher,
&node,
ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
"micro_ros_arduino_node_publisher"));

// create timer,
const unsigned int timer_timeout = 1000;
RCCHECK(rclc_timer_init_default(
&timer,
&support,
RCL_MS_TO_NS(timer_timeout),
timer_callback));

// create executor
RCCHECK(rclc_executor_init(&executor, &support.context, 1, &allocator));
RCCHECK(rclc_executor_add_timer(&executor, &timer));
~~~

5. loop
- タイマーイベントを処理

~~~cpp
delay(100);
RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(100)));
~~~

## Linux側
1. micro-ros-agentのセットアップ

~~~bash
source /opt/ros/$ROS_DISTRO/setup.bash
mkdir uros_ws && cd uros_ws
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash
~~~

2. micro-ROS-Agentをビルドして実行

~~~bash
cd ~/uros_ws
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.sh
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0 --baud 115200
~~~

![成功](success.png)

## 大参考
[micro-ros-agentのsetupまで](https://kanpapa.com/2022/09/ros2-m5stack-microros.html)

[usbでの走らせ方](https://zenn.dev/array/books/5efdb438cf8be3/viewer/4415c4)

[m5stackのコード](https://github.com/micro-ROS/micro_ros_arduino/blob/humble/examples/micro-ros_publisher/micro-ros_publisher.ino)
