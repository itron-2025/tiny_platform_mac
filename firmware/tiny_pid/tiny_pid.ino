// =============================================================================
//  tiny_pid.ino  -  STAGE 2 firmware: CLOSED-LOOP mecanum driver for
//                   tiny_platform_mac, driven over micro-ROS.
//
//  WHAT CHANGES FROM tiny_open, AND WHY IT MATTERS HERE. The open-loop firmware
//  computed duty from a straight line fitted to the wheels spinning IN THE AIR.
//  That line knows nothing about the robot's weight, the tyre contact patch, or
//  the mecanum rollers scrubbing sideways, so on the floor it under-drove badly:
//  a 0.25 m/s command worked out to about 8% duty and the robot simply sat
//  there. This firmware keeps that fit as a FEEDFORWARD -- an instant, roughly
//  right opening bid -- and puts a PI controller around the encoder feedback to
//  make up whatever it gets wrong. The integrator is what carries the load.
//
//  Chain per wheel, every 20 ms:
//      /tiny/cmd_vel -> inverse kinematics -> target rad/s
//      encoder delta -> raw rad/s -> 1D Kalman -> measured rad/s
//      duty = FF(target) + PI(target - measured)
//      -> TB6612
//
//  Down (host -> ESP32)
//      /tiny/cmd_vel     geometry_msgs/Twist         normal driving
//      /tiny/wheel_duty  std_msgs/Float32MultiArray  BRING-UP ONLY, raw duty per
//                        wheel, straight past the kinematics AND the PID. This
//                        is how direction calibration and duty_sweep.py work,
//                        and it must stay open loop or those tools would be
//                        measuring the controller instead of the motors.
//      /tiny/pid_gains   std_msgs/Float32MultiArray  [KP x4, KI x4, KD x4],
//                        live retuning without a reflash; reverts on reboot.
//
//  Up (ESP32 -> host)
//      /tiny/wheel_actual_vel  Float32MultiArray[4]  rad/s, KALMAN-FILTERED
//                              (tiny_open published the raw value here)
//      /tiny/wheel_ticks       Int32MultiArray[4]    cumulative counts
//
//  /tiny/wheel_duty WINS while it is fresh, overriding /tiny/cmd_vel entirely,
//  so a calibration publisher cannot be fought by a teleop node.
//
//  Conventions (ROS REP-103): +x forward, +y LEFT, +z yaw counter-clockwise.
//  Wheel order everywhere: [FL, FR, RL, RR]. Everything sits under /tiny (see
//  config.h MICROROS_NAMESPACE and CLAUDE.md 3.10).
//
//  Toolchain: esp32 core 2.0.x ONLY, micro_ros_arduino 2.0.7-humble.
//  Start the agent BEFORE the board boots -- `make up` handles it.
//  micro-ROS owns the serial port: the LED is the only status channel.
// =============================================================================

#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <geometry_msgs/msg/twist.h>
#include <std_msgs/msg/float32_multi_array.h>
#include <std_msgs/msg/int32_multi_array.h>

#include "config.h"
#include "Encoder.h"
#include "Motor.h"
#include "Pid.h"
#include "KalmanFilter.h"

// ----------------------------------------------------------------------------
//  micro-ROS handles
// ----------------------------------------------------------------------------
rcl_node_t      node;
rcl_allocator_t allocator;
rclc_support_t  support;
rclc_executor_t executor;

rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_wheel_duty;
rcl_subscription_t sub_gains;
rcl_publisher_t    pub_actual;
rcl_publisher_t    pub_ticks;
rcl_timer_t        control_timer;

geometry_msgs__msg__Twist        msg_cmd;
std_msgs__msg__Float32MultiArray msg_duty;
std_msgs__msg__Float32MultiArray msg_gains;
std_msgs__msg__Float32MultiArray msg_actual;
std_msgs__msg__Int32MultiArray   msg_ticks;

// micro-ROS needs preallocated storage for every sequence field, incoming as
// well as outgoing. Nothing here is malloc'd at runtime.
static float   duty_in_buf[NUM_WHEELS];
static float   gains_buf[3 * NUM_WHEELS];
static float   actual_buf[NUM_WHEELS];
static int32_t ticks_buf[NUM_WHEELS];

int32_t tick_total[NUM_WHEELS] = { 0, 0, 0, 0 };

// ----------------------------------------------------------------------------
//  Per-wheel hardware and control state
// ----------------------------------------------------------------------------
Encoder        enc[NUM_WHEELS];
Motor          mot[NUM_WHEELS];
Pid            pid[NUM_WHEELS];
KalmanFilter1D kf[NUM_WHEELS];
uint16_t       stall_ticks[NUM_WHEELS] = { 0, 0, 0, 0 };

volatile float    cmd_vx = 0.0f, cmd_vy = 0.0f, cmd_wz = 0.0f;
volatile uint32_t last_cmd_ms = 0;
volatile float    raw_duty[NUM_WHEELS] = { 0, 0, 0, 0 };
volatile uint32_t last_duty_ms = 0;

// ----------------------------------------------------------------------------
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  for (int i = 0; i < NUM_WHEELS; i++) mot[i].stop();
  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);                       // fast blink = alive, no agent
  }
}

// ----------------------------------------------------------------------------
//  Callbacks
// ----------------------------------------------------------------------------
void cmd_vel_callback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  cmd_vx = (float)m->linear.x;
  cmd_vy = (float)m->linear.y;
  cmd_wz = (float)m->angular.z;
  last_cmd_ms = millis();
}

void wheel_duty_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  size_t n = m->data.size;
  if (n > NUM_WHEELS) n = NUM_WHEELS;
  for (size_t i = 0; i < n; i++)          raw_duty[i] = m->data.data[i];
  for (size_t i = n; i < NUM_WHEELS; i++) raw_duty[i] = 0.0f;
  last_duty_ms = millis();
}

// Live gain update. Ignores a message of the wrong length rather than applying
// half of it: a partially-applied gain set is a controller nobody can reason
// about, and the sender is a tuning script that can simply be fixed.
void gains_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m =
      (const std_msgs__msg__Float32MultiArray *)msgin;
  if (m->data.size != 3 * NUM_WHEELS) return;
  for (int i = 0; i < NUM_WHEELS; i++) {
    const float kp = m->data.data[i];
    const float ki = m->data.data[NUM_WHEELS + i];
    const float kd = m->data.data[2 * NUM_WHEELS + i];
    // Clamp: a typo in a tuning script should not be able to make the robot
    // unstable at full duty.
    pid[i].setGains(kp < 0 ? 0 : (kp > 100 ? 100 : kp),
                    ki < 0 ? 0 : (ki > 200 ? 200 : ki),
                    kd < 0 ? 0 : (kd > 10  ? 10  : kd));
  }
}

// ----------------------------------------------------------------------------
//  Mecanum inverse kinematics. X-roller layout (rollers form an "X" seen from
//  above). If the wheels are mounted the other way the vy terms flip -- verify
//  on the real robot, the encoders cannot see it.
// ----------------------------------------------------------------------------
void inverse_kinematics(float vx, float vy, float wz, float *w_out) {
  const float r = WHEEL_RADIUS_M;
  w_out[FL] = (vx - vy - L_SUM * wz) / r;
  w_out[FR] = (vx + vy + L_SUM * wz) / r;
  w_out[RL] = (vx + vy - L_SUM * wz) / r;
  w_out[RR] = (vx - vy + L_SUM * wz) / r;
}

// ----------------------------------------------------------------------------
//  Control loop, every CONTROL_PERIOD_MS.
// ----------------------------------------------------------------------------
void control_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  // MEASURED dt, not the nominal period. The executor introduces timer jitter,
  // and dividing the tick delta by a fixed 20 ms aliases that jitter straight
  // into the velocity signal -- it was a large part of the raw measurement
  // noise on the Tomcat chassis. Clamped to [0.5, 1.5]x nominal so one
  // pathological gap cannot distort the loop.
  static uint32_t last_us = 0;
  const uint32_t now_us = micros();
  float dt = (last_us == 0) ? CONTROL_DT_S : (now_us - last_us) * 1e-6f;
  last_us = now_us;
  if (dt < 0.5f * CONTROL_DT_S) dt = 0.5f * CONTROL_DT_S;
  if (dt > 1.5f * CONTROL_DT_S) dt = 1.5f * CONTROL_DT_S;

  const uint32_t now = millis();
  const bool duty_fresh = (now - last_duty_ms) < CMD_TIMEOUT_MS;
  const bool cmd_fresh  = (now - last_cmd_ms)  < CMD_TIMEOUT_MS;

  // Body velocity -> wheel targets, normalised so exceeding the ceiling slows
  // the robot down rather than bending its path.
  float w[NUM_WHEELS] = { 0, 0, 0, 0 };
  if (cmd_fresh && !duty_fresh) {
    inverse_kinematics(cmd_vx, cmd_vy, cmd_wz, w);
    float peak = 0.0f;
    for (int i = 0; i < NUM_WHEELS; i++) {
      const float a = fabsf(w[i]);
      if (a > peak) peak = a;
    }
    if (peak > MAX_WHEEL_RAD_S) {
      const float scale = MAX_WHEEL_RAD_S / peak;
      for (int i = 0; i < NUM_WHEELS; i++) w[i] *= scale;
    }
  }

  for (int i = 0; i < NUM_WHEELS; i++) {
    // Encoder -> raw rad/s, using the measured dt.
    const long delta = enc[i].readDelta() * ENC_DIR[i];
    tick_total[i] += (int32_t)delta;
    const float raw_vel = ((float)delta / COUNTS_PER_WHEEL_REV) * 2.0f * PI / dt;

    // The filter always runs: it tracks the physical wheel, not the command,
    // so pausing it during a stop would leave it stale on the next start.
    const float vel = kf[i].update(raw_vel);

    float duty;
    if (duty_fresh) {
      // Bring-up path: raw duty, no kinematics, no PID, no feedforward. The
      // calibration tools MUST see the motor's own response; running them
      // through the controller would measure the controller.
      duty = raw_duty[i];
      pid[i].reset();
      stall_ticks[i] = 0;

    } else {
      // Feedforward sign comes from the TARGET, never the measurement: the
      // measurement jitters through zero at low speed and would chatter the
      // offset term between +FF_OFFSET and -FF_OFFSET every tick.
      float w_t = w[i];
      float ff  = 0.0f;
      if (fabsf(w_t) < W_TARGET_EPS) {
        w_t = 0.0f;
      } else {
        ff = copysignf(FF_OFFSET[i] + FF_GAIN[i] * fabsf(w_t), w_t);
      }

      if (!cmd_fresh || w_t == 0.0f) {
        // Commanded stop (or the watchdog fired): brake with P only, and clear
        // the integrator EVERY tick.
        //
        // Holding the integrator here is a trap worth naming: on the Tomcat
        // chassis, stopping right after a wheel had stalled left a fully
        // charged integral that kept driving forward for as long as the
        // measured speed stayed above the release threshold -- the robot
        // lurched when told to stop. P-only braking decays monotonically and
        // then releases.
        pid[i].reset();
        duty = (fabsf(vel) < STOP_VEL_EPS) ? 0.0f
                                           : pid[i].update(0.0f, vel, dt, 0.0f);
        stall_ticks[i] = 0;
      } else {
        duty = pid[i].update(w_t, vel, dt, ff);

        // Stall protection: pushed hard, not turning, for STALL_TIMEOUT_MS ->
        // drop back to the feedforward level and clear the windup. Protects the
        // motor and the TB6612 from sustained stall current, and means releasing
        // a blocked wheel gives a fresh response rather than a stored lurch.
        if (fabsf(vel) < STALL_VEL_EPS && fabsf(duty) > STALL_DUTY) {
          if (stall_ticks[i] < 60000) stall_ticks[i]++;
        } else {
          stall_ticks[i] = 0;
        }
        if (stall_ticks[i] > (STALL_TIMEOUT_MS / CONTROL_PERIOD_MS)) {
          pid[i].reset();
          duty = ff;
        }
      }
    }

    mot[i].drive(duty);
    actual_buf[i] = vel;              // FILTERED speed, unlike tiny_open
    ticks_buf[i]  = tick_total[i];
  }

  static uint8_t decim = 0;
  if (++decim >= PUBLISH_DECIMATION) {
    decim = 0;
    msg_actual.data.size = NUM_WHEELS;
    RCSOFTCHECK(rcl_publish(&pub_actual, &msg_actual, NULL));
    msg_ticks.data.size = NUM_WHEELS;
    RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));
  }

  if (duty_fresh || cmd_fresh) {
    digitalWrite(LED_PIN, HIGH);                  // solid = driving
  } else {
    digitalWrite(LED_PIN, (now / 500) % 2);       // slow blink = connected, idle
  }
}

// ----------------------------------------------------------------------------
void init_messages() {
  msg_duty.data.data     = duty_in_buf;
  msg_duty.data.size     = 0;
  msg_duty.data.capacity = NUM_WHEELS;
  msg_duty.layout.dim.data = NULL;
  msg_duty.layout.dim.size = 0;
  msg_duty.layout.dim.capacity = 0;

  msg_gains.data.data     = gains_buf;
  msg_gains.data.size     = 0;
  msg_gains.data.capacity = 3 * NUM_WHEELS;
  msg_gains.layout.dim.data = NULL;
  msg_gains.layout.dim.size = 0;
  msg_gains.layout.dim.capacity = 0;

  msg_actual.data.data     = actual_buf;
  msg_actual.data.size     = NUM_WHEELS;
  msg_actual.data.capacity = NUM_WHEELS;
  msg_actual.layout.dim.data = NULL;
  msg_actual.layout.dim.size = 0;
  msg_actual.layout.dim.capacity = 0;
  for (int i = 0; i < NUM_WHEELS; i++) actual_buf[i] = 0.0f;

  msg_ticks.data.data     = ticks_buf;
  msg_ticks.data.size     = NUM_WHEELS;
  msg_ticks.data.capacity = NUM_WHEELS;
  msg_ticks.layout.dim.data = NULL;
  msg_ticks.layout.dim.size = 0;
  msg_ticks.layout.dim.capacity = 0;
  for (int i = 0; i < NUM_WHEELS; i++) ticks_buf[i] = 0;
}

// ----------------------------------------------------------------------------
void setup() {
  // MOTOR PINS FIRST. STBY is jumpered to 3V3 on this robot, so the TB6612
  // outputs are live from power-on while IN1/IN2 are still floating inputs.
  // Nothing may be reordered ahead of this loop -- see tiny_open.ino.
  for (int i = 0; i < NUM_WHEELS; i++) {
    mot[i].begin(MOT_IN1[i], MOT_IN2[i], MOT_PWM[i], MOT_PWM_CH[i],
                 PWM_FREQ_HZ, PWM_RES_BITS, MOTOR_DIR[i]);
    mot[i].stop();
  }

  set_microros_transports();
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  for (int i = 0; i < NUM_WHEELS; i++) {
    enc[i].begin(ENC_A[i], ENC_B[i], ENC_USE_PULLUP);
    pid[i].begin(PID_KP[i], PID_KI[i], PID_KD[i], PID_I_MAX, (float)PWM_MAX);
    kf[i].begin(KF_Q, KF_R);
  }

  delay(2000);                        // give the agent a moment

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, MICROROS_NODE_NAME, MICROROS_NAMESPACE, &support));

  RCCHECK(rclc_subscription_init_default(
      &sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), TOPIC_CMD_VEL));
  RCCHECK(rclc_subscription_init_default(
      &sub_wheel_duty, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_WHEEL_DUTY));
  RCCHECK(rclc_subscription_init_default(
      &sub_gains, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_PID_GAINS));

  RCCHECK(rclc_publisher_init_default(
      &pub_actual, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray), TOPIC_WHEEL_ACTUAL));
  RCCHECK(rclc_publisher_init_default(
      &pub_ticks, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray), TOPIC_WHEEL_TICKS));

  RCCHECK(rclc_timer_init_default(
      &control_timer, &support, RCL_MS_TO_NS(CONTROL_PERIOD_MS), control_callback));

  init_messages();

  // 3 subscriptions + 1 timer.
  RCCHECK(rclc_executor_init(&executor, &support.context, 4, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_cmd_vel, &msg_cmd, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_wheel_duty, &msg_duty, &wheel_duty_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_gains, &msg_gains, &gains_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // Start stale so the wheels stay stopped until something commands them.
  last_cmd_ms  = millis() - CMD_TIMEOUT_MS - 1;
  last_duty_ms = last_cmd_ms;
}

void loop() {
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
}
