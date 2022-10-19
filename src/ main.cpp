#include <Arduino.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// microros dependencies
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

// microros message definitions
#include <nav_msgs/msg/odometry.h>
#include <geometry_msgs/msg/twist.h>

// custom transport layer definition
#include "balltze_transport.h"


#define ERROR_LOOP_LED_PIN (16)
#define RCCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)) {error_loop();}}
#define RCSOFTCHECK(fn) { rcl_ret_t temp_rc = fn; if((temp_rc != RCL_RET_OK)){}}
#define EXECUTE_EVERY_N_MS(MS, X)  do { \
  static volatile int64_t init = -1; \
  if (init == -1) { init = uxr_millis();} \
  if (uxr_millis() - init > MS) { X; init = uxr_millis();} \
} while (0)\

// error loop used to handle exceptions of uROS
void error_loop(){
  while(1){
    digitalWrite(ERROR_LOOP_LED_PIN, !digitalRead(ERROR_LOOP_LED_PIN));
    delay(100);
  }
}

rclc_support_t support;
rcl_node_t node;
rcl_timer_t timer;
rclc_executor_t executor;
rcl_allocator_t allocator;
rcl_subscription_t cmd_vel_subscriber;
rcl_publisher_t odometry_publisher;
rcl_publisher_t joint_state_publisher;
nav_msgs__msg__Odometry odometry_msg;
geometry_msgs__msg__Twist cmd_vel_msg;

TwoWire ImuWire(18, 19);
Adafruit_BNO055 bno = Adafruit_BNO055(50, 0x29, &ImuWire);

bool micro_ros_init_successful;

enum states {
  WAITING_AGENT,
  AGENT_AVAILABLE,
  AGENT_CONNECTED,
  AGENT_DISCONNECTED
} state;

void subscription_callback(const void * msgin) {
  const geometry_msgs__msg__Twist * msg = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vel_msg.linear.x = msg->linear.x;
  cmd_vel_msg.angular.z = msg->angular.z;
}

void control_timer_callback(rcl_timer_t * timer, int64_t last_call_time) {
  (void) last_call_time;
  if (timer != NULL) {
    RCSOFTCHECK(rcl_publish(&odometry_publisher, &odometry_msg, NULL));
  }
}

bool create_entities() {
  allocator = rcl_get_default_allocator();

  // create init_options
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));

  // create node
  RCCHECK(rclc_node_init_default(&node, "rmbot_hardware_node", "", &support));

  // create subscriber
  RCCHECK(rclc_subscription_init_default(
    &cmd_vel_subscriber,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
    "cmd_vel"));

  // create publisher
  RCCHECK(rclc_publisher_init_default(
    &odometry_publisher,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry),
    "odometry/wheels"));

  // create timer running at 25 Hz
  const unsigned int timer_timeout = 40;
  RCCHECK(rclc_timer_init_default(
    &timer,
    &support,
    RCL_MS_TO_NS(timer_timeout),
    control_timer_callback));

  // create executor
  executor = rclc_executor_get_zero_initialized_executor();
  RCCHECK(rclc_executor_init(&executor, &support.context, 2, &allocator));
  RCCHECK(rclc_executor_add_timer(&executor, &timer));
  RCCHECK(rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &cmd_vel_msg, &subscription_callback, ON_NEW_DATA));

  // set frame id for transform between parent and child in odometry message
  odometry_msg.header.frame_id = micro_ros_string_utilities_set(odometry_msg.header.frame_id, "/odom");
  odometry_msg.child_frame_id = micro_ros_string_utilities_set(odometry_msg.child_frame_id, "/base_link");

  // set non zero values on diagonals of covariance matrices for velocity and position
  for (size_t i = 0; i < 36; i += 6) {
    odometry_msg.pose.covariance[i] = 0.0001;
    odometry_msg.twist.covariance[i] = 0.0001;
  }

  // synchronize time between uROS and uROS agent
  static time_t timeout_ms = 100;
  static int64_t time_ms = 0;
  while (time_ms <= 0) {
    RCCHECK(rmw_uros_sync_session(timeout_ms));
    time_ms = rmw_uros_epoch_millis(); 
  }
}

void destroy_entities()
{
  rmw_context_t * rmw_context = rcl_context_get_rmw_context(&support.context);
  (void) rmw_uros_set_context_entity_destroy_session_timeout(rmw_context, 0);

  RCSOFTCHECK(rcl_publisher_fini(&odometry_publisher, &node));
  RCSOFTCHECK(rcl_timer_fini(&timer));
  RCSOFTCHECK(rclc_executor_fini(&executor));
  RCSOFTCHECK(rcl_node_fini(&node));
  RCSOFTCHECK(rclc_support_fini(&support));
}

void add_quat_to_msg(imu::Quaternion& quat)
{
  odometry_msg.pose.pose.orientation.w = quat.w();
  odometry_msg.pose.pose.orientation.x = quat.x();
  odometry_msg.pose.pose.orientation.y = quat.y();
  odometry_msg.pose.pose.orientation.z = quat.z();
}


void setup() {
  // setup microros transport layer 
  rmw_uros_set_custom_transport(
		true,
		NULL,
		balltze_transport_open,
		balltze_transport_close,
		balltze_transport_write,
		balltze_transport_read
	);


  // // setup error LED
  pinMode(ERROR_LOOP_LED_PIN, OUTPUT);
  digitalWrite(ERROR_LOOP_LED_PIN, LOW);

    /* Initialise the sensor */
  if (!bno.begin())
  {
    //There was a problem detecting the BNO055
    error_loop();
  }

  state = WAITING_AGENT;
}

void loop() {

  imu::Quaternion quat = bno.getQuat();
  add_quat_to_msg(quat);

  switch (state) {
    case WAITING_AGENT:
      EXECUTE_EVERY_N_MS(500, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_AVAILABLE : WAITING_AGENT;);
      break;
    case AGENT_AVAILABLE:
      state = (true == create_entities()) ? AGENT_CONNECTED : WAITING_AGENT;
      if (state == WAITING_AGENT) {
        destroy_entities();
      };
      break;
    case AGENT_CONNECTED:
      EXECUTE_EVERY_N_MS(200, state = (RMW_RET_OK == rmw_uros_ping_agent(100, 1)) ? AGENT_CONNECTED : AGENT_DISCONNECTED;);
      if (state == AGENT_CONNECTED) {
        rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
      }
      break;
    case AGENT_DISCONNECTED:
      destroy_entities();
      state = WAITING_AGENT;
      break;
    default:
      break;
  }
}