#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

// ==== L298N MOTOR DRIVER PINS ====
#define ENA 4    // PWM for left motor
#define IN1 18
#define IN2 5

#define ENB 0   // PWM for right motor
#define IN3 16
#define IN4 17

// ==== ENCODER PINS ====
#define LEFT_ENC_A 32
#define LEFT_ENC_B 33
#define RIGHT_ENC_A 25
#define RIGHT_ENC_B 26
// ==== ROBOT PARAMETERS ====
const float WHEEL_RADIUS = 0.045; // meters
const float WHEEL_BASE = 0.22;    // meters
const int TICKS_PER_REV = 1232;

volatile long left_ticks = 0;
volatile long right_ticks = 0;

// ==== ROS2 VARIABLES ====
rcl_publisher_t odom_publisher;
rcl_subscription_t cmd_vel_subscriber;
geometry_msgs__msg__Twist twist_msg;
nav_msgs__msg__Odometry odom_msg;

rclc_executor_t executor;
rclc_support_t support;
rcl_allocator_t allocator;
rcl_node_t node;

// ==== MOTOR CONTROL ====
void setMotorL298N(int pwmPin, int in1, int in2, float speed) {
  int pwm = abs(speed * 255);
  pwm = constrain(pwm, 0, 255);
  digitalWrite(in1, speed >= 0 ? HIGH : LOW);
  digitalWrite(in2, speed >= 0 ? LOW : HIGH);
  ledcWrite(pwmPin, pwm);
}

void setMotorSpeed(float linear, float angular) {
  float v_left = linear - (angular * WHEEL_BASE / 2.0);
  float v_right = linear + (angular * WHEEL_BASE / 2.0);

  setMotorL298N(0, IN1, IN2, v_left);   // channel 0 for ENA
  setMotorL298N(1, IN3, IN4, v_right);  // channel 1 for ENB
}

// ==== ENCODER INTERRUPTS ====
void IRAM_ATTR leftEncoderISR() {
  bool b = digitalRead(LEFT_ENC_B);
  left_ticks += b ? -1 : 1;
}

void IRAM_ATTR rightEncoderISR() {
  bool b = digitalRead(RIGHT_ENC_B);
  right_ticks += b ? 1 : -1;
}

// ==== ROS CALLBACK ====
void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *msg = (const geometry_msgs__msg__Twist *)msgin;
  setMotorSpeed(msg->linear.x, msg->angular.z);
}

// ==== ODOMETRY ====
void publishOdometry() {
  static long last_left = 0, last_right = 0;

  noInterrupts();
  long current_left = left_ticks;
  long current_right = right_ticks;
  interrupts();

  long delta_left = current_left - last_left;
  long delta_right = current_right - last_right;
  last_left = current_left;
  last_right = current_right;

  float dist_left = (delta_left / (float)TICKS_PER_REV) * 2 * PI * WHEEL_RADIUS;
  float dist_right = (delta_right / (float)TICKS_PER_REV) * 2 * PI * WHEEL_RADIUS;

  float dist_center = (dist_left + dist_right) / 2.0;

  odom_msg.twist.twist.linear.x = dist_center / 0.05;  // 20 Hz loop
  odom_msg.twist.twist.angular.z = (dist_right - dist_left) / WHEEL_BASE / 0.05;

  rcl_publish(&odom_publisher, &odom_msg, NULL);
}

// ==== SETUP ====
void setup() {
  // === PWM CHANNELS ===
  ledcSetup(0, 1000, 8); ledcAttachPin(ENA, 0);
  ledcSetup(1, 1000, 8); ledcAttachPin(ENB, 1);

  // === MOTOR PINS ===
  pinMode(IN1, OUTPUT); pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT); pinMode(IN4, OUTPUT);

  // === ENCODER PINS ===
  pinMode(LEFT_ENC_A, INPUT_PULLUP);
  pinMode(LEFT_ENC_B, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A), leftEncoderISR, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A), rightEncoderISR, RISING);

  // === MICRO-ROS INIT ===
  set_microros_transports();
  delay(2000);
  allocator = rcl_get_default_allocator();
  rclc_support_init(&support, 0, NULL, &allocator);
  rclc_node_init_default(&node, "esp32_node", "", &support);
  rclc_publisher_init_default(&odom_publisher, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom");
  rclc_subscription_init_default(&cmd_vel_subscriber, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel");
  rclc_executor_init(&executor, &support.context, 1, &allocator);
  rclc_executor_add_subscription(&executor, &cmd_vel_subscriber, &twist_msg, &cmdVelCallback, ON_NEW_DATA);
}

// ==== LOOP ====
void loop() {
  rclc_executor_spin_some(&executor, RCL_MS_TO_NS(50));
  publishOdometry();
  delay(50); // 20 Hz loop
}
