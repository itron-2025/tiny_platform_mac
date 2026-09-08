// =============================================================================
//  config.h  -  All hardware pins and machine constants in ONE place.
//               (CLOSED-LOOP variant, see tiny_pid.ino)
//
//  Pins, directions and machine constants are COPIED VERBATIM from
//  firmware/tiny_open/config.h -- they describe the robot, not the controller.
//  If you rewire or recalibrate, change BOTH files (and docs/WIRING.md).
//
//  The main sketch must never contain a bare GPIO number or a bare machine
//  constant. If you rewire the robot, this file and docs/WIRING.md are the only
//  two places that change.
//
//  Wheel indexing is FIXED for the whole project:
//      0 = FL (front-left)   1 = FR (front-right)
//      2 = RL (rear-left)    3 = RR (rear-right)
//  Every array below MUST follow that order.
// =============================================================================
#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

#define NUM_WHEELS 4
enum WheelIndex { FL = 0, FR = 1, RL = 2, RR = 3 };

// ----------------------------------------------------------------------------
//  Encoder pins (quadrature A + B per motor).
//
//  These eight sit on one side of the 38-pin DevKit header (36/39/34/35/32/33/
//  25/26) so the harness can be a single ribbon.
//
//  GPIO 34/35/36/39 are INPUT-ONLY and have NO internal pull-up hardware --
//  pinMode(34, INPUT_PULLUP) is silently a no-op on them. docs/WIRING.md asks
//  for external 10k pull-ups to 3V3 on all eight lines: mandatory if the
//  encoder outputs are open-drain (otherwise those wheels read a constant 0),
//  harmless if they are push-pull.
// ----------------------------------------------------------------------------
const uint8_t ENC_A[NUM_WHEELS] = { 34, 36, 32, 25 }; // FL, FR, RL, RR
const uint8_t ENC_B[NUM_WHEELS] = { 35, 39, 33, 26 };

// Requests INPUT_PULLUP where the hardware has one (32/33/25/26). No effect on
// 34/35/36/39 -- those depend on the external resistors.
#define ENC_USE_PULLUP true

// ----------------------------------------------------------------------------
//  Motor driver pins (2x TB6612 -> 4 channels).
//
//  GPIO12 IS DELIBERATELY ABSENT AND MUST STAY THAT WAY. It is the MTDI
//  strapping pin: held high at boot it switches the flash rail to 1.8 V and the
//  board bootloops forever, while still flashing perfectly in download mode.
//  The Tomcat chassis lost a day to exactly this. See CLAUDE.md section 3.1.
// ----------------------------------------------------------------------------
const uint8_t MOT_IN1[NUM_WHEELS] = { 13, 16, 18, 22 };
const uint8_t MOT_IN2[NUM_WHEELS] = { 14, 17, 19, 23 };
const uint8_t MOT_PWM[NUM_WHEELS] = { 27, 15,  5, 21 };

// TB6612 STBY.
//
// ON THIS ROBOT STBY IS HARDWIRED TO 3V3 -- the D64A board brings out a single
// STBY pin and it was jumpered straight to the logic rail (user, 2026-09-08).
// No GPIO controls it, so GPIO4 is now a free spare.
//
// WHAT THAT COSTS, so nobody rediscovers it the hard way: the drivers are
// enabled from the instant power is applied. GPIO15 and GPIO5 -- the FR and RL
// PWM pins -- are strapping pins, internally pulled HIGH, and emit a signal
// during boot, while IN1/IN2 are still floating inputs until setup() runs.
// So for roughly the first 300 ms after power-on those two channels can twitch.
// Harmless with the chassis elevated; a small lurch on the floor.
// setup() configures the motor pins as its very first action to keep that
// window as short as software can make it.
//
// To get the protection back, wire STBY to GPIO4 with a 10k pull-down to GND
// and set STBY_ON_GPIO to 1 below. GPIO4 floats at reset, so the pull-down
// holds the drivers off through the whole boot.
#define STBY_ON_GPIO 0               // 0 = hardwired to 3V3 in hardware

#if STBY_ON_GPIO
const uint8_t STBY_PINS[] = { 4 };
#define NUM_STBY (sizeof(STBY_PINS) / sizeof(STBY_PINS[0]))
#else
#define NUM_STBY 0
#endif

// ----------------------------------------------------------------------------
//  PWM (LEDC) -- ESP32 Arduino core 2.0.x API (ledcSetup / ledcAttachPin).
//  Core 3.x removed those calls; if the compiler says 'ledcSetup' was not
//  declared, the CORE VERSION is wrong. Do not rewrite these calls.
// ----------------------------------------------------------------------------
#define PWM_FREQ_HZ   20000          // 20 kHz -> above hearing
#define PWM_RES_BITS  10             // 10-bit -> duty 0..1023
#define PWM_MAX       ((1 << PWM_RES_BITS) - 1)
const uint8_t MOT_PWM_CH[NUM_WHEELS] = { 0, 1, 2, 3 };

// ----------------------------------------------------------------------------
//  Direction calibration.
//
//  MEASURED ON THE REAL CHASSIS 2026-09-08 with tools/wheel_sweep.py: each wheel
//  driven alone at duty +180 while a human watched which way it rolled.
//
//    wheel   rolled at +duty   encoder read   ->  MOTOR_DIR   ENC_DIR
//    FL      BACKWARD          up (+6223)         -1          -1
//    FR      BACKWARD          up (+5903)         -1          -1
//    RL      forward           DOWN (-5201)       +1          -1
//    RR      forward           up (+6507)         +1          +1
//
//  Why FL/FR need BOTH flipped: reversing MOTOR_DIR makes the wheel roll the
//  other way, which also reverses the encoder, so ENC_DIR has to follow or the
//  count would end up negative for a forward command. RL is the opposite case --
//  the motor was already correct and only its A/B pair is swapped, so only
//  ENC_DIR moves. The two are NOT independent knobs; always re-derive both from
//  a fresh observation rather than nudging one.
//
//  Invariant this encodes: positive command -> wheel drives the robot FORWARD
//  -> /tiny/wheel_ticks for that wheel INCREASES.
// ----------------------------------------------------------------------------
const int8_t MOTOR_DIR[NUM_WHEELS] = { -1, -1, +1, +1 }; // FL, FR, RL, RR
const int8_t ENC_DIR[NUM_WHEELS]   = { -1, -1, -1, +1 }; // FL, FR, RL, RR

// ----------------------------------------------------------------------------
//  Chassis geometry (measured 2026-09-08).
//    wheel radius 40 mm (80 mm mecanum wheel)
//    wheelbase  (front<->rear axle) 204 mm -> LX = 102 mm
//    track width(left <->right     ) 325 mm -> LY = 162.5 mm
//  LX/LY only scale the wz (rotation) coupling term; an error there shows up as
//  "rotation is faster/slower than commanded", not as a wrong direction.
// ----------------------------------------------------------------------------
#define WHEEL_RADIUS_M   0.040f
#define LX               0.102f
#define LY               0.1625f
#define L_SUM            (LX + LY)   // 0.2645 m

// ----------------------------------------------------------------------------
//  Encoder counts per WHEEL revolution.
//
//  MEASURED 2026-09-08 with tools/counts_per_rev.py: each wheel hand-turned one
//  full revolution against a mark, with the motors coasting.
//      FL 563   FR 581   RL 572   RR 581   ->  mean 574.25
//  The 3.2% spread across the four matches the +/-2.8% you get from lining a
//  mark back up by eye over a single turn, so this is one clean measurement, not
//  four noisy ones.
//
//  IT REPLACES 520, WHICH WAS WRONG BY 10.4%. 520 came from the datasheet-ish
//  figures 13 PPR x 4 (quadrature) x 10:1 gearbox. Even allowing the full hand
//  error the measured value only spans 558..590, so 520 is firmly excluded --
//  the gearbox is very likely 11:1 rather than the 10:1 the part number implies
//  (13 x 4 x 11 = 572, right in the middle of the measured range). 12 x 4 x 12 =
//  576 fits about as well; the measurement cannot separate them and it does not
//  matter, because only the PRODUCT ppr * quad * gear is physically observable.
//  So this stays the measured number rather than a tidier theoretical one.
//
//  WHY IT HAD TO BE MEASURED, and why no amount of testing could have found it:
//  this constant scales the PID's measurement AND its setpoint together, and
//  scales the odometry by the same factor the other way. With a wrong value the
//  loop still tracks its target perfectly while every wheel runs at the wrong
//  speed, and the odometry under-reports by exactly the compensating amount.
//  The two errors cancel in every self-consistency check. Only a mark on the
//  wheel, or a tape measure on the floor, can see it. The Tomcat chassis ran a
//  full benchmark campaign on a value that was 2.5x wrong.
//
//  TO REFINE: turn each wheel FIVE turns instead of one
//  (`tools/counts_per_rev.py --turns 5`). The alignment error is a one-off at
//  the end, so five turns divides it by five -> about +/-0.55%, enough to
//  separate 572 from 576. Not urgent: the remaining uncertainty here is under
//  1%, well inside the error in the wheel radius and in floor slip.
//
//  Keep this in step with the host-side params once those exist.
// ----------------------------------------------------------------------------
#define COUNTS_PER_WHEEL_REV 574.0f

// ----------------------------------------------------------------------------
//  Control loop timing
// ----------------------------------------------------------------------------
#define CONTROL_PERIOD_MS  20        // 50 Hz
#define CONTROL_DT_S       (CONTROL_PERIOD_MS / 1000.0f)

// ----------------------------------------------------------------------------
//  Safety
// ----------------------------------------------------------------------------
// No command within this window -> wheels stop. Last line of defence against a
// crashed teleop node or a dropped USB link.
#define CMD_TIMEOUT_MS  500

// Hard cap on the duty this firmware will ever emit, as a fraction of PWM_MAX.
// Left at 1.0 because tools/duty_sweep.py has to reach full duty to measure the
// top speed at all. The /cmd_vel path is limited by MAX_WHEEL_RAD_S instead, so
// this only really governs the raw /tiny/wheel_duty bring-up channel -- which is
// unguarded BY DESIGN (calibration has to see exactly what it asked for) and
// should not be left publishing unattended.
#define DUTY_LIMIT_FRAC  1.00f
#define DUTY_LIMIT       (DUTY_LIMIT_FRAC * PWM_MAX)

// ----------------------------------------------------------------------------
//  Open-loop speed mapping -- MEASURED 2026-09-08, tools/duty_sweep.py
//  (chassis elevated, duty staircase 0..1023, speed from the settled tail of
//   each step).
//
//      wheel   deadband fwd/rev   top speed fwd/rev
//      FL          60 /  80        99.51 / 105.24 rad/s
//      FR          80 / 100       102.54 / 103.05
//      RL         120 /  60       100.16 / 102.58
//      RR          60 / 100       105.48 / 104.09
//
//  TWO SEPARATE CONSTANTS, because they answer two different questions. The
//  first version of this file used MAX_WHEEL_RAD_S for both and that is a bug
//  waiting to happen: it is simultaneously "the fastest we allow" and "the
//  speed that corresponds to full duty". Set it to 0.8x the top speed as a
//  safety derate and the duty map silently gains a 25% error -- ask for 79.6
//  rad/s, get full duty, actually spin 99.5. Splitting them keeps the derate
//  a derate.
// ----------------------------------------------------------------------------

// Commanded speed ceiling. The inverse kinematics scales ALL FOUR wheels down
// by a common factor when any one exceeds this, so the robot keeps going in the
// commanded direction instead of curving.
//
// 45 rad/s = 1.8 m/s at the 40 mm wheel; the hardware tops out near 99.5
// (4.0 m/s). Inherited from the open-loop firmware, where it had to be raised
// from 25 because the robot could not move on the floor at all:
//
//   FF_OFFSET / FF_GAIN were measured with the CHASSIS ELEVATED. That line
//   describes the duty needed to spin a free wheel, and nothing else. Put the
//   robot on the ground and the same command has to overcome the machine's
//   weight, the tyre contact patch and the mecanum rollers' scrub -- none of
//   which the no-load fit knows about. A 0.25 m/s command works out to about 8%
//   duty, which is plenty to spin a wheel in the air and nowhere near enough to
//   move a robot.
//
// Raising the ceiling raises the duty for a given stick position, which is the
// blunt fix. 45 rad/s = 1.8 m/s at the 40 mm wheel; the hardware tops out near
// 99.5 (4.0 m/s), so this stays well inside what the motors can do while giving
// combined motions (diagonal + rotation ask more of one wheel than the body
// speed alone) room before the normalisation clips them.
//
// THIS FIRMWARE IS THAT FIX. The PID integrator adds duty until the wheel
// actually reaches the commanded speed, on any surface and under any load,
// instead of trusting a fit taken in mid-air. The ceiling here is therefore a
// genuine speed limit again rather than a workaround for missing torque.
#define MAX_WHEEL_RAD_S  45.0f

// Per-wheel feedforward: duty = sign(w) * (FF_OFFSET[i] + FF_GAIN[i] * |w|).
//
// IN THIS FIRMWARE THE FEEDFORWARD IS NO LONGER THE WHOLE ANSWER -- it is the
// opening bid, and the PID corrects whatever it gets wrong. That changes what
// its errors cost: under-driving on the floor is now made up by the integrator
// within a fraction of a second instead of leaving the robot stuck. Keeping the
// FF at all still matters, though: it puts the duty in the right neighbourhood
// instantly, so the integrator only has to cover the difference, which is the
// difference between a loop that responds crisply and one that always lags.
//
// Mean of the forward and reverse fits (tools/duty_sweep.py --sign +1 / -1):
//
//      wheel   forward fit           reverse fit           mean used here
//      FL      21.6 + 9.91*w         22.5 + 9.46*w         22.1 + 9.69*w
//      FR      25.7 + 9.67*w         34.8 + 9.57*w         30.3 + 9.62*w
//      RL      37.8 + 9.89*w         32.0 + 9.62*w         34.9 + 9.76*w
//      RR      22.9 + 9.39*w         21.5 + 9.61*w         22.2 + 9.50*w
//
// PER WHEEL, AND DELIBERATELY NOT AVERAGED ACROSS WHEELS. The four motors
// differ by ~6% in gain and by 2x in deadband; that spread is precisely what
// makes an open-loop mecanum robot curve instead of driving straight, so
// averaging it away would throw out the one thing these numbers are for.
// Averaging the two DIRECTIONS is a different matter and is fine: the biggest
// asymmetry (FR, 25.7 vs 34.8) costs under 1 rad/s, inside the +/-10% this
// open-loop map already carries, and the closed loop will absorb it.
const float FF_OFFSET[NUM_WHEELS] = { 22.1f, 30.3f, 34.9f, 22.2f };  // duty
const float FF_GAIN[NUM_WHEELS]   = {  9.69f, 9.62f, 9.76f, 9.50f }; // duty/(rad/s)

// Breakaway (stiction) duty per wheel: the duty that gets a STOPPED wheel
// moving, larger than the kinetic FF_OFFSET above.
//
// NOT USED BY THIS FIRMWARE. The integrator does this job properly: it keeps
// adding duty until the wheel actually moves, whatever the surface, instead of
// jumping to a fixed number measured in mid-air. Kept here only so the two
// config.h files stay comparable, and as a record of the measurement.
//
// MEASURED IN BOTH DIRECTIONS, and that turned out to matter more than
// anything else here. The first version used forward-only figures
// { 60, 80, 120, 60 } and the robot could not strafe or yaw at low speed at
// all: every wheel the kinematics asked to run BACKWARDS just sat there while
// the others pulled away. Breakaway friction is not symmetric, and not even
// asymmetric in the same direction on every wheel:
//
//      wheel   forward   reverse   max   +10% margin -> used
//      FL         60        80      80        88
//      FR         80       100     100       110
//      RL        120        60     120       132
//      RR         60       100     100       110
//
// Values are the larger of the two directions, since the firmware cannot know
// which way a wheel will be asked to turn, plus 10% for battery sag and
// temperature. They are also quantised to the sweep's step grid, so each is
// already the first step that DID move the wheel rather than a tight bound.
const float BREAKAWAY_DUTY[NUM_WHEELS] = { 88.0f, 110.0f, 132.0f, 110.0f };

// Below this commanded wheel speed the request is treated as an exact stop.
// Without it the feedforward's intercept (~22 duty) would be emitted for an
// arbitrarily small target, so a stick resting a hair off centre would leave
// all four motors buzzing at a duty too low to actually turn them.
// 0.1 rad/s is 4 mm/s at the 40 mm wheel -- nothing worth commanding.
#define W_TARGET_EPS  0.1f

// A wheel measuring slower than this is treated as "not turning" by the
// stiction breaker below. One encoder count per 20 ms control tick is
// 2*pi/574/0.02 = 0.55 rad/s, so anything at or under quantisation noise must
// count as stopped; 1.0 leaves a little margin above it without swallowing
// speeds anyone would deliberately command.
#define STOPPED_EPS  1.0f              // ⚠️ ESTIMATE -- measure and replace


// ============================================================================
//  CLOSED-LOOP TUNING BLOCK - the only part to edit between tuning iterations.
//
//  Seeded from the Tomcat chassis, which is a defensible starting point rather
//  than a guess: its FF_GAIN was 8.91..9.64 duty per rad/s against 9.50..9.76
//  here, so the plant these gains see -- duty in, rad/s out -- is within a few
//  percent of the one they were tuned on. What differs is the load: that robot
//  was tuned elevated, this one has to carry itself, which the integrator
//  covers rather than the gains.
//
//  Tune in this order (docs/BRINGUP.md): KP until it tracks without ringing,
//  then KI until steady-state error disappears, and leave KD at 0.
// ============================================================================

// Per-wheel gains, order FL, FR, RL, RR. Starting identical: the wheel-to-wheel
// spread lives in the FEEDFORWARD above, which is where a static difference
// belongs. Split these only if the benchmark shows one wheel behaving
// differently in a way FF cannot express.
const float PID_KP[NUM_WHEELS] = { 6.4f, 6.4f, 6.4f, 6.4f };
const float PID_KI[NUM_WHEELS] = { 12.0f, 12.0f, 12.0f, 12.0f };
// KD stays 0. The measurement is one encoder count per 20 ms tick = 0.55 rad/s
// of quantisation; differentiating that amplifies the quantisation more than it
// adds damping. PI is enough for a velocity loop.
const float PID_KD[NUM_WHEELS] = { 0.0f, 0.0f, 0.0f, 0.0f };

// Integrator ceiling, in duty units. This is the headroom the loop has to make
// up whatever the feedforward got wrong -- and since FF was measured with the
// wheels in the air, on the floor that gap is exactly the load. 400 lets the
// integrator roughly double a mid-speed feedforward, which is ample; the
// conditional anti-windup in Pid.h keeps it from charging while saturated.
#define PID_I_MAX  400.0f

// 1D Kalman on the raw wheel speed. Only the RATIO Q/R sets the steady-state
// gain, so these are scale-free and carry over from Tomcat unchanged.
// K ~ 0.25 -> about 3 samples (60 ms) of lag, noise attenuated ~2.6x.
// Larger R = smoother but laggier; larger Q = more responsive but noisier.
#define KF_Q   0.45f
#define KF_R   5.4f

// Commanded stop: release the motor once the wheel is essentially stationary,
// instead of holding a braking duty against a wheel that has already stopped.
#define STOP_VEL_EPS  0.4f           // rad/s

// Stall protection. A wheel that is being pushed hard and still is not turning
// (blocked wheel, dead driver channel, a motor wire that fell out) would
// otherwise sit at full integrator output indefinitely, cooking the motor and
// the TB6612. After the timeout the drive is backed off to the feedforward
// level and the integrator cleared, so releasing the wheel gives a fresh start
// rather than a windup lurch.
#define STALL_VEL_EPS    0.6f        // rad/s: "not turning" (above quantisation)
#define STALL_DUTY       450.0f      // duty:  "pushing hard"
#define STALL_TIMEOUT_MS 1500

#define PUBLISH_DECIMATION  1        // publish every Nth tick (1 = 50 Hz)

// ----------------------------------------------------------------------------
//  micro-ROS identifiers. CHANGING ANY OF THESE MUST BE MIRRORED ON THE HOST.
// ----------------------------------------------------------------------------
// Node name and NAMESPACE. The namespace is the whole cross-talk defence:
// everything this board publishes or subscribes to lands under /tiny/, so the
// Tomcat chassis running on the same host (bare /cmd_vel, node esp32_base)
// cannot drive this robot and cannot collide with its node name.
//
// ROS_DOMAIN_ID would have been the textbook answer and does NOT work here:
// the prebuilt agent talks to Fast-DDS directly, never reads ROS_DOMAIN_ID and
// always uses domain 0. See the long comment in the Dockerfile.
//
// Topic names below are RELATIVE on purpose -- rcl resolves them against the
// namespace, giving /tiny/cmd_vel and friends. Do not prefix them by hand.
#define MICROROS_NODE_NAME   "base"
#define MICROROS_NAMESPACE   "tiny"
#define TOPIC_CMD_VEL        "cmd_vel"            // down: host -> ESP32 (Twist)
#define TOPIC_WHEEL_DUTY     "wheel_duty"         // down: host -> ESP32 (raw duty, bring-up)
#define TOPIC_WHEEL_ACTUAL   "wheel_actual_vel"   // up:   ESP32 -> host (rad/s)
#define TOPIC_WHEEL_TICKS    "wheel_ticks"        // up:   ESP32 -> host (counts)
// Live PID retuning, so gains can be changed without a reflash. Layout is
// [KP x4, KI x4, KD x4] in FL,FR,RL,RR order -- 12 floats. A reboot reverts to
// the PID_* defaults above, which is deliberate: it makes an experiment
// impossible to leave behind by accident.
#define TOPIC_PID_GAINS      "pid_gains"          // down: host -> ESP32 (12 floats)

// On-board LED. The ONLY status channel: micro-ROS owns the serial port, so
// Serial.print() is impossible in this firmware.
//   fast blink  = firmware running, no agent (or a failed rcl_* call)
//   solid on    = agent connected AND a fresh command is arriving
//   slow blink  = agent connected, command stale (watchdog stopped the wheels)
//   dark        = board did not boot -> suspect wiring, see CLAUDE.md 3.1
#define LED_PIN 2

#endif // CONFIG_H
