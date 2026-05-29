/* ============================================================================
 *  HapticElbow  ·  Adaptive Stroke Rehabilitation Exoskeleton  ·  Firmware
 *  Niel Boon & Senne Peeters · KU Leuven · 2026
 * ----------------------------------------------------------------------------
 *  Hardware:
 *    - ODrive S1                     (UART, axis0)            Serial1
 *    - 2x MPU-6050 IMU               (I2C, 0x68 + 0x69)       Wire
 *    - DRV2605L LRA-driver           (I2C, 0x5A · PWM on D13) Wire + Timer4
 *
 *  Architecture:
 *    - Cooperative state machine          (see SystemStatus)
 *    - 10 Hz telemetry output             (POS / VEL / TRQ / STATE / IMU1 / IMU2)
 *    - Non-blocking command parser        (newline-delimited, case-insensitive)
 *    - Hybrid control:
 *        · Position mode  for calibration, presets, settings
 *        · Torque mode    during active therapy
 *    - Impedance controller in torque mode:
 *        torque = assist + damping + virtual_spring
 *      with slew-rate limit and hard safety clamp.
 *
 *  IMPORTANT RULE:
 *    No `delay()` is allowed inside the main therapy loop. A delay() call would
 *    freeze the loop and the ODrive serial channel would time out. Only the
 *    one-shot setup / smooth-transition helpers use delay().
 *
 *  Serial protocol (115200 baud, towards dashboard, one line per field):
 *      POS:<deg>     elbow angle (motor) relative to software-zero
 *      VEL:<rev/s>   filtered motor velocity
 *      TRQ:<Nm>      current applied torque
 *      STATE:<n>     0..5  →  see SystemStatus
 *      IMU1:<deg>    upper-arm pitch  (atan2 over accelerometer)
 *      IMU2:<deg>    lower-arm pitch
 *
 *  Commands IN (from dashboard, all case-insensitive):
 *      RESET, CHANGE_SETTINGS, SAVE_DONE,
 *      STRAIGHT, start, stop,
 *      MODE_ASSISTIVE | MODE_RESISTIVE | MODE_FREERIDE,
 *      SET_LIM_MIN:<v>,  SET_LIM_MAX:<v>,
 *      SET_A_TORQUE:<v>, SET_A_FACTOR:<v>, SET_A_DAMP:<v>,
 *      SET_T_DAMP:<v>,   SET_SPRING:<v>,   SET_MAX_TQ:<v>,
 *      SET_HAPTIC:<v>,   SET_ANGLE:<v>,    SET_SHOCK:<v>,
 *      SET_POS_GAIN:<v>, SET_VEL_GAIN:<v>, SET_VEL_INT:<v>, SET_BW:<v>
 * ========================================================================== */

#include <ODriveUART.h>
#include <Wire.h>

// ============================================================================
//   1.  KINEMATICS / HARDWARE CONSTANTS  (not runtime-adjustable)
// ============================================================================
static constexpr float OFFSET_TO_STRAIGHT  = -65.0f;       // [deg] hardware-index → software 0°
static constexpr float DEGREES_TO_TURNS    = 1.0f / 360.0f;

static constexpr float VELOCITY_THRESHOLD  = 0.02f;        // [rev/s] dead-zone in assist
static constexpr float MAX_SAFE_VELOCITY   = 0.80f;        // [rev/s] emergency stop above this
static constexpr float VELOCITY_FILTER     = 0.80f;        // EMA weight on previous value
static constexpr float MAX_TORQUE_STEP     = 0.015f;       // [Nm/loop] slew-rate
static constexpr float BASE_DAMPING        = 0.05f;        // permanent damper

// I2C addresses
static constexpr uint8_t MPU_UPPER_ARM     = 0x68;
static constexpr uint8_t MPU_LOWER_ARM     = 0x69;
static constexpr uint8_t DRV_ADDR          = 0x5A;

// Timing
static constexpr uint32_t TELEMETRY_INTERVAL_MS = 100;     // 10 Hz
static constexpr uint32_t HAPTIC_COOLDOWN_MS    = 2500;
static constexpr uint32_t HAPTIC_PULSE_MS       = 50;

// Soft-start: during this window after therapy activation, only light damping
// is applied (no assist, no virtual spring). This lets the patient + robot
// reach equilibrium first and prevents the mechanical kick we used to see at
// the very first impedance command. See README §3.5.6.
static constexpr uint32_t SOFT_START_MS      = 1500;
static constexpr float    SOFT_START_DAMPING = 0.25f;

// Vibration suppression (high-pass damping):
// When the arm is not rigidly coupled to the brace (or when no arm is in
// the brace at all), the combined inertia is lower than what the impedance
// controller was tuned for. The system becomes "too sharp" and can
// oscillate (positive feedback of the assist loop on small vibrations).
// The high-pass damping automatically damps the fast components of the
// motor velocity (= the difference between instantaneous and LPF-filtered
// velocity), without affecting normal motion:
//
//   normal motion: fb.vel ≈ filtered_vel  →  hf ≈ 0  →  no damping
//   oscillation:   fb.vel oscillates fast →  hf large →  strong damping
//
// Active in every mode (assistive, resistive, freeride) and during
// soft-start. The gain sits in TherapyConfig (runtime-adjustable via
// SET_SHOCK). See README §3.5.7.
static constexpr float    HF_DAMPING_GAIN_DEFAULT = 1.50f;

// ODrive control modes (see ODrive docs)
static constexpr int ODRV_MODE_TORQUE   = 1;
static constexpr int ODRV_MODE_POSITION = 3;

// ============================================================================
//   2.  THERAPY CONFIG  (runtime-adjustable via SET_* commands)
// ============================================================================
struct TherapyConfig {
  float assist_torque             = 0.60f;
  float assist_ramp_factor        = 1.00f;
  float assist_extension_damping  = 0.40f;
  float training_resistance       = 2.50f;
  float virtual_spring_stiffness  = 0.10f;
  float safety_torque_max         = 1.50f;
  float limit_min                 = 0.0f;
  float limit_max                 = 100.0f;
  float haptic_strength           = 1.00f;
  float hf_damping_gain           = HF_DAMPING_GAIN_DEFAULT;   // shock reduction
};

// ============================================================================
//   3.  STATE
// ============================================================================
enum SystemStatus : uint8_t {
  WAITING_FOR_INDEX    = 0,
  WAITING_FOR_STRAIGHT = 1,
  MOVING_TO_START      = 2,
  READY                = 3,
  THERAPY_ACTIVE       = 4,
  SETTINGS_ADJUST      = 5
};

enum TherapyMode : uint8_t {
  ASSISTIVE = 0,
  RESISTIVE = 1,
  FREERIDE  = 2
};

struct RuntimeState {
  SystemStatus status = WAITING_FOR_INDEX;
  TherapyMode  mode   = ASSISTIVE;

  // Calibration references (ODrive turn-space)
  float hardware_index_turns = 0.0f;
  float software_zero_turns  = 0.0f;
  float current_target_turns = 0.0f;

  // Live signals
  float imu1_upper_arm   = 0.0f;   // [deg]
  float imu2_lower_arm   = 0.0f;   // [deg]
  float filtered_vel     = 0.0f;   // [rev/s]
  float current_torque   = 0.0f;   // [Nm]

  // Scheduling
  uint32_t last_telemetry_ms = 0;
  uint32_t last_haptic_ms    = 0;
  uint32_t therapy_start_ms  = 0;  // moment of latest "start"
};

// Global instances
ODriveUART    odrive(Serial1);
TherapyConfig cfg;
RuntimeState  st;

// ============================================================================
//   4.  FORWARD DECLARATIONS
// ============================================================================
// IMU / sensors
static void  imuInit(uint8_t addr);
static float imuReadPitchDeg(uint8_t addr);

// Telemetry / comm
static void  telemetryEmit();
static void  commandParseAndDispatch();

// State helpers
static void  setSystemStatus(SystemStatus s);
static void  softReset();

// ODrive control
static void  modeSwitchPosition();
static void  modeSwitchTorque();
static void  moveSmooth(float from_pos, float to_pos, float duration_s);

// Therapy
static void  runTherapy();

// Haptic
static void  pwmD13_init();
static void  drvInit();
static void  drvWriteReg(uint8_t reg, uint8_t val);
static void  triggerHapticPulse();

// Utility
static inline float clampf(float x, float lo, float hi) {
  return (x < lo) ? lo : (x > hi) ? hi : x;
}

// ============================================================================
//   5.  SETUP  /  LOOP
// ============================================================================
// One-time bring-up: open both serial links and the I2C bus, wake the IMUs,
// set up the haptic driver, and wait for the ODrive to finish its self-test.
void setup() {
  Serial1.begin(115200);
  Serial.begin(115200);
  Wire.begin();

  while (!Serial);    // wait until host opens the port

  imuInit(MPU_UPPER_ARM);
  imuInit(MPU_LOWER_ARM);

  pwmD13_init();
  drvInit();

  // Wait until the ODrive finishes its self-test
  while (odrive.getState() == AXIS_STATE_UNDEFINED) delay(100);
}

// Main scheduler. Runs continuously: sample the IMUs, push telemetry at a
// fixed rate, handle any incoming command, then advance the state machine.
void loop() {
  // --- Sample IMUs every loop pass (~1 kHz limited by I2C latency) ---
  st.imu1_upper_arm = imuReadPitchDeg(MPU_UPPER_ARM);
  st.imu2_lower_arm = imuReadPitchDeg(MPU_LOWER_ARM);

  // --- 10 Hz telemetry to dashboard ---
  if (millis() - st.last_telemetry_ms >= TELEMETRY_INTERVAL_MS) {
    telemetryEmit();
    st.last_telemetry_ms = millis();
  }

  // --- Incoming commands ---
  if (Serial.available() > 0) commandParseAndDispatch();

  // --- State-machine tick ---
  switch (st.status) {
    case WAITING_FOR_INDEX:
      if (odrive.getState() == AXIS_STATE_IDLE) {
        st.hardware_index_turns = odrive.getFeedback().pos;
        st.current_target_turns = st.hardware_index_turns;
        setSystemStatus(WAITING_FOR_STRAIGHT);
      }
      break;

    case MOVING_TO_START: {
      modeSwitchPosition();
      st.current_target_turns = odrive.getFeedback().pos;
      const float target = st.hardware_index_turns +
                           (OFFSET_TO_STRAIGHT * DEGREES_TO_TURNS);
      moveSmooth(st.current_target_turns, target, 2.0f);
      delay(200);
      st.software_zero_turns  = target;
      st.current_target_turns = target;
      setSystemStatus(READY);
      break;
    }

    case THERAPY_ACTIVE:
      runTherapy();
      break;

    case READY:
    case WAITING_FOR_STRAIGHT:
    case SETTINGS_ADJUST:
    default:
      // idle — waiting for input
      break;
  }
}

// ============================================================================
//   6.  TELEMETRY
// ============================================================================
// Send one telemetry frame to the dashboard: six labelled lines the Python
// side parses. The angle is only meaningful once the software-zero is known
// (status >= READY), so before that we report 0.
static void telemetryEmit() {
  float degrees = 0.0f;
  if (st.status >= READY) {
    degrees = (odrive.getFeedback().pos - st.software_zero_turns) * 360.0f;
  }
  Serial.print(F("POS:"));   Serial.println(degrees);
  Serial.print(F("VEL:"));   Serial.println(st.filtered_vel);
  Serial.print(F("TRQ:"));   Serial.println(st.current_torque);
  Serial.print(F("STATE:")); Serial.println((int)st.status);
  Serial.print(F("IMU1:"));  Serial.println(st.imu1_upper_arm);
  Serial.print(F("IMU2:"));  Serial.println(st.imu2_lower_arm);
}

// ============================================================================
//   7.  COMMAND PARSER  (no heap table; if/else is fine here)
// ============================================================================
// Read one newline-terminated command and act on it. A command is either a
// bare keyword ("start") or "KEY:value" ("SET_A_TORQUE:0.6"); we split on the
// first ':' and read the number when one is present.
static void commandParseAndDispatch() {
  String input = Serial.readStringUntil('\n');
  input.trim();
  if (input.length() == 0) return;

  const int colon = input.indexOf(':');
  const String cmd = (colon == -1) ? input : input.substring(0, colon);
  const float  v   = (colon == -1) ? 0.0f
                                   : input.substring(colon + 1).toFloat();

  // ---- General state transitions -----------------------------------------
  if (cmd.equalsIgnoreCase("RESET")) {
    softReset();
    return;
  }
  if (cmd.equalsIgnoreCase("CHANGE_SETTINGS")) {
    setSystemStatus(SETTINGS_ADJUST);
    modeSwitchPosition();
    moveSmooth(odrive.getFeedback().pos, st.software_zero_turns, 1.5f);
    st.current_target_turns = st.software_zero_turns;
    return;
  }
  if (cmd.equalsIgnoreCase("SAVE_DONE")) {
    if (st.status == SETTINGS_ADJUST) setSystemStatus(READY);
    return;
  }
  if (cmd.equalsIgnoreCase("STRAIGHT")) {
    if (st.status == WAITING_FOR_STRAIGHT) setSystemStatus(MOVING_TO_START);
    return;
  }
  if (cmd.equalsIgnoreCase("start")) {
    if (st.status == READY) {
      modeSwitchTorque();
      st.therapy_start_ms = millis();   // soft-start countdown
      setSystemStatus(THERAPY_ACTIVE);
    }
    return;
  }
  if (cmd.equalsIgnoreCase("stop")) {
    modeSwitchPosition();
    setSystemStatus(READY);
    return;
  }

  // ---- Mode selection ----------------------------------------------------
  if (cmd.equalsIgnoreCase("MODE_ASSISTIVE")) { st.mode = ASSISTIVE; return; }
  if (cmd.equalsIgnoreCase("MODE_RESISTIVE")) { st.mode = RESISTIVE; return; }
  if (cmd.equalsIgnoreCase("MODE_FREERIDE"))  { st.mode = FREERIDE;  return; }

  // ---- Parameter updates --------------------------------------------------
  if (cmd.equalsIgnoreCase("SET_LIM_MIN"))  { cfg.limit_min                = v; return; }
  if (cmd.equalsIgnoreCase("SET_LIM_MAX"))  { cfg.limit_max                = v; return; }
  if (cmd.equalsIgnoreCase("SET_A_TORQUE")) { cfg.assist_torque            = v; return; }
  if (cmd.equalsIgnoreCase("SET_A_FACTOR")) { cfg.assist_ramp_factor       = v; return; }
  if (cmd.equalsIgnoreCase("SET_A_DAMP"))   { cfg.assist_extension_damping = v; return; }
  if (cmd.equalsIgnoreCase("SET_T_DAMP"))   { cfg.training_resistance      = v; return; }
  if (cmd.equalsIgnoreCase("SET_SPRING"))   { cfg.virtual_spring_stiffness = v; return; }
  if (cmd.equalsIgnoreCase("SET_MAX_TQ"))   { cfg.safety_torque_max        = v; return; }
  if (cmd.equalsIgnoreCase("SET_HAPTIC"))   { cfg.haptic_strength          = v; return; }
  if (cmd.equalsIgnoreCase("SET_SHOCK"))    { cfg.hf_damping_gain          = v; return; }

  // ---- ODrive PID passthrough (runtime — NOT persistent in ODrive) -------
  // Changes stay active until power-cycle; the values you stored on the
  // ODrive itself are untouched.
  if (cmd.equalsIgnoreCase("SET_POS_GAIN")) {
    Serial1.print(F("w axis0.controller.config.pos_gain "));
    Serial1.println(v, 4);
    return;
  }
  if (cmd.equalsIgnoreCase("SET_VEL_GAIN")) {
    Serial1.print(F("w axis0.controller.config.vel_gain "));
    Serial1.println(v, 4);
    return;
  }
  if (cmd.equalsIgnoreCase("SET_VEL_INT")) {
    Serial1.print(F("w axis0.controller.config.vel_integrator_gain "));
    Serial1.println(v, 4);
    return;
  }
  if (cmd.equalsIgnoreCase("SET_BW")) {
    Serial1.print(F("w axis0.controller.config.input_filter_bandwidth "));
    Serial1.println(v, 3);
    return;
  }

  // ---- Target angle (only from READY or during preset moves) -------------
  if (cmd.equalsIgnoreCase("SET_ANGLE")) {
    if (st.status >= READY && st.status != SETTINGS_ADJUST) {
      modeSwitchPosition();
      const float safe   = clampf(v, cfg.limit_min, cfg.limit_max);
      const float target = st.software_zero_turns + safe * DEGREES_TO_TURNS;
      moveSmooth(odrive.getFeedback().pos, target, 1.5f);
      st.current_target_turns = target;
      setSystemStatus(READY);
    }
    return;
  }
}

// ============================================================================
//   8.  THERAPY  —  Impedance controller (position + velocity → torque)
// ============================================================================
static void runTherapy() {
  const ODriveFeedback fb = odrive.getFeedback();
  const float degrees = (fb.pos - st.software_zero_turns) * 360.0f;

  // EMA on velocity (calmer torque response)
  st.filtered_vel = VELOCITY_FILTER * st.filtered_vel +
                    (1.0f - VELOCITY_FILTER) * fb.vel;

  // Run-away safety
  if (fabsf(fb.vel) > MAX_SAFE_VELOCITY) {
    st.current_torque = 0.0f;
    odrive.setTorque(0.0f);
    return;
  }

  // Haptic pulse at ROM limits, with cooldown
  if (degrees <= cfg.limit_min || degrees >= cfg.limit_max) {
    if (millis() - st.last_haptic_ms > HAPTIC_COOLDOWN_MS) {
      triggerHapticPulse();
      st.last_haptic_ms = millis();
    }
  }

  // ── VIBRATION SUPPRESSION (always active) ───────────────────────────
  // High-pass damping: takes the difference between instantaneous and
  // LPF-filtered velocity. Kills oscillations with light/loose mounting
  // without damping normal motion.
  const float vel_hf            = fb.vel - st.filtered_vel;
  const float vibration_damping = -cfg.hf_damping_gain * vel_hf;

  // ── SOFT-START ───────────────────────────────────────────────────────
  // The first SOFT_START_MS ms after "start" we apply only light damping
  // and NO assist / training resistance. This prevents the kick caused
  // by the patient + robot snapping into equilibrium before active
  // control kicks in. Between SOFT_START_MS and 2x SOFT_START_MS the gain
  // ramps from 0 → 1 (quadratic ease-in for a soft onset).
  const uint32_t since_start = millis() - st.therapy_start_ms;
  const bool     in_soft_start = (since_start < SOFT_START_MS);

  if (in_soft_start) {
    // Pure transparent damping + vibration suppression.
    float target = -SOFT_START_DAMPING * st.filtered_vel + vibration_damping;
    target = clampf(target, -cfg.safety_torque_max,
                            +cfg.safety_torque_max);
    const float d = target - st.current_torque;
    if (d > +MAX_TORQUE_STEP)      st.current_torque += MAX_TORQUE_STEP;
    else if (d < -MAX_TORQUE_STEP) st.current_torque -= MAX_TORQUE_STEP;
    else                            st.current_torque  = target;
    odrive.setTorque(st.current_torque);
    return;
  }

  // Ease-in of assist over a second SOFT_START_MS window (0..1, quadratic)
  float startup_gain = 1.0f;
  if (since_start < 2 * SOFT_START_MS) {
    const float t = (float)(since_start - SOFT_START_MS) / (float)SOFT_START_MS;
    startup_gain = t * t;
  }

  // --- Three torque contributions -----------------------------------------
  float assist  = 0.0f;
  float damping = 0.0f;
  float spring  = 0.0f;

  // Virtual spring outside ROM (pushes back into range)
  if (degrees < cfg.limit_min) {
    spring = (cfg.limit_min - degrees) * cfg.virtual_spring_stiffness;
  } else if (degrees > cfg.limit_max) {
    spring = (cfg.limit_max - degrees) * cfg.virtual_spring_stiffness;
  }

  // Mode-specific logic within ROM
  const bool inside_rom = (degrees >= cfg.limit_min && degrees <= cfg.limit_max);
  if (inside_rom) {
    switch (st.mode) {

      case ASSISTIVE:
        if (st.filtered_vel > VELOCITY_THRESHOLD) {
          // Patient actively flexing → boost
          assist  = (st.filtered_vel - VELOCITY_THRESHOLD) * cfg.assist_ramp_factor;
          if (assist > cfg.assist_torque) assist = cfg.assist_torque;
          damping = -BASE_DAMPING * st.filtered_vel;
        } else if (st.filtered_vel < -VELOCITY_THRESHOLD) {
          // Patient extending → controlled resistance on the way back
          damping = -cfg.assist_extension_damping * st.filtered_vel;
        } else {
          damping = -BASE_DAMPING * st.filtered_vel;
        }
        break;

      case RESISTIVE:
        if (fabsf(st.filtered_vel) > VELOCITY_THRESHOLD) {
          damping = -cfg.training_resistance * st.filtered_vel;
        } else {
          damping = -BASE_DAMPING * st.filtered_vel;
        }
        break;

      case FREERIDE:
      default:
        // Fully transparent — only the spring if we cross ROM
        break;
    }
  }

  // Apply soft-start gain on the active (direction-giving) components;
  // damping stays at full strength so transients are also damped during
  // the ramp-in.
  assist *= startup_gain;
  spring *= startup_gain;

  // Vibration suppression is always on, in every mode.
  damping += vibration_damping;

  // Sum + hard safety clamp
  const float target = clampf(assist + damping + spring,
                              -cfg.safety_torque_max,
                              +cfg.safety_torque_max);

  // Slew-rate limit (no abrupt torque steps)
  const float delta = target - st.current_torque;
  if (delta > +MAX_TORQUE_STEP)      st.current_torque += MAX_TORQUE_STEP;
  else if (delta < -MAX_TORQUE_STEP) st.current_torque -= MAX_TORQUE_STEP;
  else                                st.current_torque  = target;

  odrive.setTorque(st.current_torque);
}

// ============================================================================
//   9.  IMU  (MPU-6050, accel-based pitch, ±2 g range)
// ============================================================================
// Wake one MPU-6050 out of its default sleep mode so it starts sampling.
static void imuInit(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x6B); Wire.write(0x00);   // PWR_MGMT_1: wake
  Wire.endTransmission(true);
}

// Read the three accelerometer axes from one MPU-6050 and turn the measured
// gravity direction into a pitch angle in degrees.
static float imuReadPitchDeg(uint8_t addr) {
  Wire.beginTransmission(addr);
  Wire.write(0x3B);                     // ACCEL_XOUT_H
  Wire.endTransmission(false);
  Wire.requestFrom((int)addr, 6, (int)true);

  const int16_t rawX = (Wire.read() << 8) | Wire.read();
  const int16_t rawY = (Wire.read() << 8) | Wire.read();
  const int16_t rawZ = (Wire.read() << 8) | Wire.read();

  const float aX = rawX / 16384.0f;
  const float aY = rawY / 16384.0f;
  const float aZ = rawZ / 16384.0f;

  // 3D gravity-vector tilt (well-conditioned everywhere, unlike the 2D
  // atan2(Y, Z) form which collapses near ±90°).
  return (atan2f(aY, sqrtf(aX * aX + aZ * aZ)) * 180.0f) / PI;
}

// ============================================================================
//   10. ODRIVE HELPERS
// ============================================================================
static void setSystemStatus(SystemStatus s) {
  st.status = s;
}

// Put the ODrive back to idle and clear all calibration and runtime state, so
// the next start-up sequence (index search -> straighten -> ready) is clean.
static void softReset() {
  odrive.setState(AXIS_STATE_IDLE);
  st.hardware_index_turns = 0.0f;
  st.software_zero_turns  = 0.0f;
  st.current_target_turns = 0.0f;
  st.filtered_vel         = 0.0f;
  st.current_torque       = 0.0f;
  setSystemStatus(WAITING_FOR_INDEX);
}

// Switch the ODrive into position mode. We command the current position first
// so the controller holds still instead of jumping when the mode changes.
static void modeSwitchPosition() {
  odrive.setPosition(odrive.getFeedback().pos);
  delay(10);
  Serial1.print(F("w axis0.controller.config.control_mode "));
  Serial1.println(ODRV_MODE_POSITION);
  delay(10);
  odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

// Switch the ODrive into torque mode, starting from zero torque so therapy
// always begins from a neutral, non-pushing state.
static void modeSwitchTorque() {
  odrive.setTorque(0.0f);
  delay(10);
  Serial1.print(F("w axis0.controller.config.control_mode "));
  Serial1.println(ODRV_MODE_TORQUE);
  delay(10);
  odrive.setState(AXIS_STATE_CLOSED_LOOP_CONTROL);
}

// Smoothstep interpolation between two positions over `duration_s` seconds.
// Blocking — only used during setup / transitions, never inside the therapy
// loop.
static void moveSmooth(float from_pos, float to_pos, float duration_s) {
  const int      kSteps   = 100;
  const uint32_t kStepMs  = (uint32_t)(duration_s * 1000.0f) / kSteps;
  for (int i = 0; i <= kSteps; i++) {
    const float t = (float)i / kSteps;
    const float f = t * t * (3.0f - 2.0f * t);              // smoothstep
    odrive.setPosition(from_pos + (to_pos - from_pos) * f);
    delay(kStepMs);
  }
}

// ============================================================================
//   11. DRV2605L HAPTIC  (PWM via Timer 4 on pin D13)
// ============================================================================
// Configure the ATmega32U4's 16-bit Timer4 for hardware PWM on pin D13. The
// vibration waveform is then generated by the timer itself rather than being
// bit-banged in software; OCR4A sets the duty cycle (drive strength).
static void pwmD13_init() {
  TCCR4A = 0; TCCR4B = 5; TCCR4C = 0; TCCR4D = 0;
  PLLFRQ = (PLLFRQ & 0xCF) | 0x30;
  OCR4C  = 255;            // 8-bit top
  OCR4A  = 0;              // duty 0
  DDRC  |= _BV(7);         // D13 as output
  TCCR4A = 0x82;           // OCR4A to pin, fast-PWM
}

static void drvWriteReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(DRV_ADDR);
  Wire.write(reg);
  Wire.write(val);
  Wire.endTransmission();
}

// Initialise the DRV2605L to drive the LRA from the external PWM input.
static void drvInit() {
  drvWriteReg(0x01, 0x00);   // out of standby
  drvWriteReg(0x1A, 0xB6);   // LRA mode + closed-loop feedback
  drvWriteReg(0x03, 0x06);   // Library 6 (LRA)
  drvWriteReg(0x01, 0x03);   // input via PWM (D13)
}

// Fire one short haptic "tap": drive the LRA at full scale for
// HAPTIC_PULSE_MS, then switch it off. Intensity is scaled by the
// dashboard's haptic-strength setting.
static void triggerHapticPulse() {
  int pwm = (int)(255.0f * cfg.haptic_strength);
  if (pwm > 255) pwm = 255;
  if (pwm < 0)   pwm = 0;
  OCR4A = (uint8_t)pwm;
  delay(HAPTIC_PULSE_MS);
  OCR4A = 0;
}
