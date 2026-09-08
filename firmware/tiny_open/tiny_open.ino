// =============================================================================
//  tiny_open.ino  -  STAGE 1 firmware: OPEN-LOOP mecanum driver for
//                    tiny_platform_mac, driven over micro-ROS.
//
//  PURPOSE: get the four wheels turning with the fewest possible unknowns, and
//  make direction calibration easy. There is NO encoder feedback in the control
//  path and NO PID here, so nothing depends on COUNTS_PER_WHEEL_REV, on the PID
//  gains, or on MAX_WHEEL_RAD_S being right. Encoders are read for REPORTING
//  ONLY. Once directions are calibrated, move to firmware/tiny_pid.
//
//  WHY OPEN LOOP FIRST: with a wheel wired backwards, open loop shows it
//  immediately -- that wheel visibly spins the wrong way. Closed loop turns the
//  same fault into "the integrator winds up until something stalls", which looks
//  like a tuning problem and wastes hours.
//
//  Two ways to command it:
//
//    /cmd_vel     geometry_msgs/Twist         normal driving
//                 -> mecanum inverse kinematics -> per-wheel rad/s
//                 -> normalise -> duty (+ deadband boost) -> 2x TB6612
//
//    /wheel_duty  std_msgs/Float32MultiArray  BRING-UP ONLY, one wheel at a time
//                 -> raw signed duty per wheel, straight to the motors.
//                    No kinematics, no deadband boost: what you send is what
//                    the motor gets, which is exactly what direction
//                    calibration needs. See docs/BRINGUP.md.
//
//  /wheel_duty WINS while it is fresh. A duty message received within
//  CMD_TIMEOUT_MS overrides /cmd_vel entirely, so a calibration publisher left
//  running cannot be fought by a teleop node. Stop the duty publisher and
//  /cmd_vel takes over again after the timeout.
//
//  Published for diagnostics (never used to control anything here):
//    /wheel_actual_vel   std_msgs/Float32MultiArray[4]  rad/s, unfiltered
//    /wheel_ticks        std_msgs/Int32MultiArray[4]    cumulative counts
//
//  EVERYTHING SITS UNDER THE /tiny NAMESPACE -- the node is /tiny/base and the
//  topics are /tiny/cmd_vel, /tiny/wheel_duty, /tiny/wheel_actual_vel,
//  /tiny/wheel_ticks. That is what keeps the Tomcat chassis (bare /cmd_vel, on
//  the same host, same DDS domain) from driving this robot. See config.h.
//
//  Conventions (ROS REP-103, must match the host side):
//    +x = forward, +y = LEFT, +z rotation = counter-clockwise (from above)
//    Wheel order everywhere: [FL, FR, RL, RR]
//
//  Toolchain (HARD constraints, see CLAUDE.md section 3.3):
//    - esp32 core 2.0.x (2.0.17). NOT 3.x: IDF v5 is ABI-incompatible with the
//      precompiled Humble micro-ROS library, and 3.x removed ledcSetup().
//    - micro_ros_arduino 2.0.7-humble.
//
//  Start the agent BEFORE the board boots, or the firmware sticks in
//  error_loop() forever (CLAUDE.md section 3.2). `make agent` handles this.
//
//  NOTE: micro-ROS owns the serial port, so Serial.print() is impossible.
//  The on-board LED is the only status channel -- see LED_PIN in config.h.
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

// ----------------------------------------------------------------------------
//  micro-ROS handles
// ----------------------------------------------------------------------------
rcl_node_t      node;
rcl_allocator_t allocator;
rclc_support_t  support;
rclc_executor_t executor;

rcl_subscription_t sub_cmd_vel;
rcl_subscription_t sub_wheel_duty;
rcl_publisher_t    pub_actual;
rcl_publisher_t    pub_ticks;
rcl_timer_t        control_timer;

geometry_msgs__msg__Twist        msg_cmd;      // incoming body velocity
std_msgs__msg__Float32MultiArray msg_duty;     // incoming raw per-wheel duty
std_msgs__msg__Float32MultiArray msg_actual;   // outgoing measured speeds
std_msgs__msg__Int32MultiArray   msg_ticks;    // outgoing cumulative ticks

// Static backing storage: micro-ROS needs preallocated buffers for every
// sequence field, incoming as well as outgoing. No malloc at runtime.
static float   duty_in_buf[NUM_WHEELS];
static float   actual_buf[NUM_WHEELS];
static int32_t ticks_buf[NUM_WHEELS];

// Cumulative encoder counts since boot (ENC_DIR applied).
int32_t tick_total[NUM_WHEELS] = { 0, 0, 0, 0 };

// ----------------------------------------------------------------------------
//  Hardware objects (one per wheel). No Pid / KalmanFilter: this is open loop.
// ----------------------------------------------------------------------------
Encoder enc[NUM_WHEELS];
Motor   mot[NUM_WHEELS];

// Latest commands. Written by the subscription callbacks, read by the control
// timer. All three run in the same executor thread, so no locking is needed.
volatile float    cmd_vx = 0.0f;         // m/s, forward
volatile float    cmd_vy = 0.0f;         // m/s, left
volatile float    cmd_wz = 0.0f;         // rad/s, counter-clockwise
volatile uint32_t last_cmd_ms = 0;

volatile float    raw_duty[NUM_WHEELS] = { 0, 0, 0, 0 };
volatile uint32_t last_duty_ms = 0;

// ----------------------------------------------------------------------------
//  Error handling. A failed rcl_* call blinks the LED forever -- there is no
//  recovery path and no reconnect logic, by design: see CLAUDE.md section 3.2.
//  In practice this almost always means "the agent was not listening when the
//  board booted".
// ----------------------------------------------------------------------------
#define RCCHECK(fn)     { rcl_ret_t rc = fn; if (rc != RCL_RET_OK) { error_loop(); } }
#define RCSOFTCHECK(fn) { rcl_ret_t rc = fn; (void)rc; }

void error_loop() {
  // Stop the motors before parking here forever. Without this a fault mid-drive
  // would leave the wheels powered with nothing left to command them.
  for (int i = 0; i < NUM_WHEELS; i++) mot[i].stop();
#if NUM_STBY > 0
  // Also drop STBY: a hardware disable of the H-bridges, not just duty 0.
  // Does nothing on this robot -- STBY is jumpered to 3V3 (see config.h).
  for (unsigned i = 0; i < NUM_STBY; i++) digitalWrite(STBY_PINS[i], LOW);
#endif

  while (true) {
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));
    delay(100);                       // fast blink = firmware alive, no agent
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

// Raw per-wheel duty, for bring-up. A short message leaves the remaining wheels
// at zero rather than at whatever they held, so a truncated command can never
// leave a wheel running.
void wheel_duty_callback(const void *msgin) {
  const std_msgs__msg__Float32MultiArray *m =
      (const std_msgs__msg__Float32MultiArray *)msgin;

  size_t n = m->data.size;
  if (n > NUM_WHEELS) n = NUM_WHEELS;

  for (size_t i = 0; i < n; i++)          raw_duty[i] = m->data.data[i];
  for (size_t i = n; i < NUM_WHEELS; i++) raw_duty[i] = 0.0f;

  last_duty_ms = millis();
}

// ----------------------------------------------------------------------------
//  Mecanum inverse kinematics: body velocity -> four wheel angular velocities.
//
//  Standard X-roller layout: the rollers form an "X" seen from above. If the
//  wheels are mounted the other way the vy terms flip.
//  ⚠️ VERIFY ON THE REAL ROBOT before trusting it: command pure +vy (strafe
//  left) and check the robot actually goes left instead of spinning or
//  grinding. docs/BRINGUP.md has the procedure.
// ----------------------------------------------------------------------------
void inverse_kinematics(float vx, float vy, float wz, float *w_out) {
  const float r = WHEEL_RADIUS_M;
  w_out[FL] = (vx - vy - L_SUM * wz) / r;
  w_out[FR] = (vx + vy + L_SUM * wz) / r;
  w_out[RL] = (vx + vy - L_SUM * wz) / r;
  w_out[RR] = (vx - vy + L_SUM * wz) / r;
}

// ----------------------------------------------------------------------------
//  Control timer, every CONTROL_PERIOD_MS.
// ----------------------------------------------------------------------------
void control_callback(rcl_timer_t *timer, int64_t last_call_time) {
  (void)last_call_time;
  if (timer == NULL) return;

  const uint32_t now = millis();
  const bool duty_fresh = (now - last_duty_ms) < CMD_TIMEOUT_MS;
  const bool cmd_fresh  = (now - last_cmd_ms)  < CMD_TIMEOUT_MS;

  // Encoders are read FIRST, before anything is commanded, so the stiction
  // breaker below can see whether each wheel is actually turning. (It also
  // means the speeds published this tick describe the command that was applied
  // last tick, which is the honest pairing.)
  float meas_w[NUM_WHEELS];
  for (int i = 0; i < NUM_WHEELS; i++) {
    const long delta = enc[i].readDelta() * ENC_DIR[i];
    tick_total[i] += (int32_t)delta;

    const float rev = (float)delta / COUNTS_PER_WHEEL_REV;
    meas_w[i]     = (rev * 2.0f * PI) / CONTROL_DT_S;   // unfiltered rad/s
    actual_buf[i] = meas_w[i];
    ticks_buf[i]  = tick_total[i];
  }

  float duty[NUM_WHEELS];

  if (duty_fresh) {
    // --- Bring-up path: raw duty, no kinematics, no deadband boost. --------
    for (int i = 0; i < NUM_WHEELS; i++) duty[i] = raw_duty[i];

  } else if (cmd_fresh) {
    // --- Normal path: /cmd_vel -> inverse kinematics -> duty. --------------
    float w[NUM_WHEELS];
    inverse_kinematics(cmd_vx, cmd_vy, cmd_wz, w);

    // Normalise. If any wheel exceeds the assumed top speed, scale ALL wheels
    // by the same factor. Clipping them individually would silently distort the
    // motion (a fast diagonal would curve); scaling preserves the direction.
    float peak = 0.0f;
    for (int i = 0; i < NUM_WHEELS; i++) {
      float a = fabsf(w[i]);
      if (a > peak) peak = a;
    }
    if (peak > MAX_WHEEL_RAD_S) {
      const float scale = MAX_WHEEL_RAD_S / peak;
      for (int i = 0; i < NUM_WHEELS; i++) w[i] *= scale;
    }

    // rad/s -> duty through each wheel's OWN measured line (config.h), rather
    // than one shared scale factor. The four motors differ by ~6% in gain and
    // 2x in deadband; feeding them a common map is exactly what makes an
    // open-loop mecanum robot curve instead of driving straight.
    for (int i = 0; i < NUM_WHEELS; i++) {
      const float mag = fabsf(w[i]);
      if (mag < W_TARGET_EPS) {
        duty[i] = 0.0f;               // exact zero request stays a true stop
      } else {
        float d = FF_OFFSET[i] + FF_GAIN[i] * mag;

        // STICTION BREAKER. FF_OFFSET is the KINETIC friction intercept -- the
        // duty that keeps a moving wheel moving. Breaking a stopped one loose
        // takes more, and by a lot on this chassis: RL needs 120 against an
        // FF_OFFSET of 37.8, so a modest command leaves it stationary while the
        // other three pull away and the robot yaws on every start.
        //
        // While a commanded wheel measures as not turning, floor its duty at
        // the breakaway value measured for that specific wheel. As soon as it
        // moves, meas_w rises past STOPPED_EPS and the FF line takes over --
        // this is a start-up assist, not a permanent boost. (The closed-loop
        // firmware gets this for free from the integrator; open loop has no
        // such memory, so it is done explicitly here.)
        if (fabsf(meas_w[i]) < STOPPED_EPS && d < BREAKAWAY_DUTY[i]) {
          d = BREAKAWAY_DUTY[i];
        }
        duty[i] = (w[i] > 0.0f) ? d : -d;
      }
    }

  } else {
    // --- Watchdog: nothing fresh on either topic -> stop. ------------------
    for (int i = 0; i < NUM_WHEELS; i++) duty[i] = 0.0f;
  }

  // Bring-up safety belt, applied to BOTH paths including raw duty: the
  // rad/s -> duty map is still a guess at this stage. See DUTY_LIMIT in config.h.
  for (int i = 0; i < NUM_WHEELS; i++) {
    if (duty[i] >  DUTY_LIMIT) duty[i] =  DUTY_LIMIT;
    if (duty[i] < -DUTY_LIMIT) duty[i] = -DUTY_LIMIT;
    mot[i].drive(duty[i]);
  }

  msg_actual.data.size = NUM_WHEELS;
  RCSOFTCHECK(rcl_publish(&pub_actual, &msg_actual, NULL));
  msg_ticks.data.size = NUM_WHEELS;
  RCSOFTCHECK(rcl_publish(&pub_ticks, &msg_ticks, NULL));

  // Status LED. Reaching here at all means the agent is connected (otherwise
  // setup() would have parked in error_loop), so this only distinguishes
  // "being commanded" from "idle".
  if (duty_fresh || cmd_fresh) {
    digitalWrite(LED_PIN, HIGH);                     // solid = driving
  } else {
    // Slow blink at ~1 Hz = connected but idle. Deliberately different from
    // error_loop()'s 5 Hz so the two are told apart across the room.
    digitalWrite(LED_PIN, (now / 500) % 2);
  }
}

// ----------------------------------------------------------------------------
//  Message buffer setup. Every sequence field needs storage before use.
//  layout.dim is left at capacity 0: `ros2 topic pub "{data: [...]}"` sends an
//  empty dim list, which is all this firmware is commanded with.
// ----------------------------------------------------------------------------
void init_messages() {
  msg_duty.data.data     = duty_in_buf;
  msg_duty.data.size     = 0;
  msg_duty.data.capacity = NUM_WHEELS;
  msg_duty.layout.dim.data     = NULL;
  msg_duty.layout.dim.size     = 0;
  msg_duty.layout.dim.capacity = 0;

  msg_actual.data.data     = actual_buf;
  msg_actual.data.size     = NUM_WHEELS;
  msg_actual.data.capacity = NUM_WHEELS;
  msg_actual.layout.dim.data     = NULL;
  msg_actual.layout.dim.size     = 0;
  msg_actual.layout.dim.capacity = 0;
  for (int i = 0; i < NUM_WHEELS; i++) actual_buf[i] = 0.0f;

  msg_ticks.data.data     = ticks_buf;
  msg_ticks.data.size     = NUM_WHEELS;
  msg_ticks.data.capacity = NUM_WHEELS;
  msg_ticks.layout.dim.data     = NULL;
  msg_ticks.layout.dim.size     = 0;
  msg_ticks.layout.dim.capacity = 0;
  for (int i = 0; i < NUM_WHEELS; i++) ticks_buf[i] = 0;
}

// ----------------------------------------------------------------------------
void setup() {
  // MOTOR PINS FIRST, BEFORE ANYTHING ELSE. On this robot STBY is jumpered to
  // 3V3, so the TB6612 outputs are live from power-on while IN1/IN2 are still
  // floating inputs -- FR and RL can twitch until these lines are driven low.
  // Nothing here may be reordered ahead of this loop; every statement moved
  // above it lengthens that window. (With STBY on a GPIO the ordering would not
  // matter, because the pull-down would hold the drivers off regardless.)
#if NUM_STBY > 0
  for (unsigned i = 0; i < NUM_STBY; i++) {
    pinMode(STBY_PINS[i], OUTPUT);
    digitalWrite(STBY_PINS[i], LOW);      // drivers off while pins settle
  }
#endif
  for (int i = 0; i < NUM_WHEELS; i++) {
    mot[i].begin(MOT_IN1[i], MOT_IN2[i], MOT_PWM[i], MOT_PWM_CH[i],
                 PWM_FREQ_HZ, PWM_RES_BITS, MOTOR_DIR[i]);
    mot[i].stop();
  }
#if NUM_STBY > 0
  for (unsigned i = 0; i < NUM_STBY; i++) digitalWrite(STBY_PINS[i], HIGH);
#endif

  // micro-ROS over USB serial @115200; the agent must match this baud.
  set_microros_transports();

  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);

  for (int i = 0; i < NUM_WHEELS; i++) {
    enc[i].begin(ENC_A[i], ENC_B[i], ENC_USE_PULLUP);
  }

  // Give the agent a moment to be ready.
  delay(2000);

  allocator = rcl_get_default_allocator();
  RCCHECK(rclc_support_init(&support, 0, NULL, &allocator));
  RCCHECK(rclc_node_init_default(&node, MICROROS_NODE_NAME, MICROROS_NAMESPACE, &support));

  RCCHECK(rclc_subscription_init_default(
      &sub_cmd_vel, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist),
      TOPIC_CMD_VEL));

  RCCHECK(rclc_subscription_init_default(
      &sub_wheel_duty, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      TOPIC_WHEEL_DUTY));

  RCCHECK(rclc_publisher_init_default(
      &pub_actual, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32MultiArray),
      TOPIC_WHEEL_ACTUAL));

  RCCHECK(rclc_publisher_init_default(
      &pub_ticks, &node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32MultiArray),
      TOPIC_WHEEL_TICKS));

  RCCHECK(rclc_timer_init_default(
      &control_timer, &support,
      RCL_MS_TO_NS(CONTROL_PERIOD_MS), control_callback));

  init_messages();

  // Executor handles: 2 subscriptions + 1 timer.
  RCCHECK(rclc_executor_init(&executor, &support.context, 3, &allocator));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_cmd_vel, &msg_cmd, &cmd_vel_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_subscription(
      &executor, &sub_wheel_duty, &msg_duty, &wheel_duty_callback, ON_NEW_DATA));
  RCCHECK(rclc_executor_add_timer(&executor, &control_timer));

  // Start stale, so the wheels stay stopped until something actually commands
  // them. millis() is small here, so subtract rather than assign zero.
  last_cmd_ms  = millis() - CMD_TIMEOUT_MS - 1;
  last_duty_ms = last_cmd_ms;
}

// ----------------------------------------------------------------------------
void loop() {
  // Pump the executor; the timer and the two subscription callbacks do the work.
  RCSOFTCHECK(rclc_executor_spin_some(&executor, RCL_MS_TO_NS(5)));
}
