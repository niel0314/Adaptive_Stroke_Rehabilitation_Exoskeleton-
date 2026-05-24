# HapticElbow

**An open-source adaptive elbow exoskeleton for stroke rehabilitation**

*B-KUL-T4lMD2 · Haptic Interfaces Experience*

**Authors:** Niel Boon · Senne Peeters
**Institution:** KU Leuven · Group T · Department of Mechanical Engineering
**Date:** May 2026

---

## Table of contents

1. [Introduction](#1-introduction)
   - [1.1 Why robot-assisted training is needed](#11-why-robot-assisted-training-is-needed)
   - [1.2 Why haptic feedback matters](#12-why-haptic-feedback-matters)
   - [1.3 The compensation problem](#13-the-compensation-problem)
   - [1.4 Existing solutions and the gap we address](#14-existing-solutions-and-the-gap-we-address)
   - [1.5 Project objectives](#15-project-objectives)
2. [Supplies — Bill of materials](#2-supplies--bill-of-materials)
3. [Methods](#3-methods)
   - [3.1 System architecture](#31-system-architecture)
   - [3.2 Why we chose these components](#32-why-we-chose-these-components)
   - [3.3 Mechanical and electrical build](#33-mechanical-and-electrical-build)
   - [3.4 ODrive configuration](#34-odrive-configuration)
   - [3.5 Control logic](#35-control-logic)
   - [3.6 Dual-IMU sensing](#36-dual-imu-sensing)
   - [3.7 Vibrotactile feedback](#37-vibrotactile-feedback)
   - [3.8 Python dashboard](#38-python-dashboard)
   - [3.9 Lessons from troubleshooting](#39-lessons-from-troubleshooting)
4. [Discussion](#4-discussion)
5. [Conclusion and future work](#5-conclusion-and-future-work)
6. [Abbreviations](#6-abbreviations)
7. [References](#7-references)

---

## 1. Introduction

### 1.1 Why robot-assisted training is needed

Neurological injuries — most prominently stroke — are a leading cause of long-term motor disability. The World Health Organization estimates that around 15&nbsp;million people suffer a stroke each year, and about one third are left with permanent impairment [[1]](#7-references). Many survivors lose control over the proximal arm, which directly compromises daily activities such as reaching, lifting and self-care.

Recovery depends on neuroplasticity: the brain reorganises itself in response to repeated, task-specific, voluntary practice [[2]](#7-references). Two facts from the clinical literature are central to our design. First, the Cochrane review by Mehrholz et al. shows that robot-assisted arm training, added to conventional therapy, improves arm function and daily-living scores after stroke [[3]](#7-references). Second, the patient must remain an active participant — passive movement gives only marginal benefit. This is the basis of the **assist-as-needed (AAN)** paradigm in rehabilitation robotics [[4]](#7-references).

### 1.2 Why haptic feedback matters

Motor learning is driven by feedback. Two complementary haptic channels are relevant for a rehabilitation device:

- **Kinaesthetic feedback** — forces transmitted through the mechanical structure. Our system uses a back-drivable brushless motor in current (torque) mode, which gives smooth low-impedance assistance without locking the joint stiff.
- **Vibrotactile feedback** — short skin vibrations that signal discrete events. Bark et al. showed that vibrotactile cues at the forearm reduce path errors and speed up motor learning, because the patient does not need to look at a screen [[5]](#7-references).

### 1.3 The compensation problem

A well-known problem in upper-limb rehabilitation is the **compensation strategy**. When the elbow flexors are too weak, the patient reflexively recruits other muscles. The shoulder lifts or the trunk rotates, so the forearm *looks* like it bends without the elbow doing the work [[6]](#7-references). This is harmful: the wrong movement pattern is rehearsed instead of the correct one. A robot that only assists torque, without watching posture, will silently reinforce the cheat.

Two inertial measurement units (IMUs) — one on the upper arm and one on the forearm — let us tell true elbow flexion apart from proximal compensation. Three findings from the clinical literature shape our detection algorithm (see §3.6.1):

1. In healthy reaching, elbow excursion is about **40–60 %** of the shoulder excursion. In stroke survivors, this drops to **10–30 %** [[7]](#7-references).
2. The expected ratio depends on the *workspace zone*. At low arm elevation the shoulder should barely move, in the middle zone the contributions are balanced, and above shoulder height the elbow must take over [[8]](#7-references).
3. Only the *outgoing* (goal-directed) part of a reach should be scored. The return to rest does not count as compensation [[9]](#7-references).

### 1.4 Existing solutions and the gap we address

Established robotic platforms such as MIT-Manus, ARMin and the Armeo family have clear clinical evidence behind them, but they cost €50&nbsp;000 to €150&nbsp;000, are tied to clinical infrastructure, and are closed-source. Lower-cost open-source projects exist, but they usually rely on stepper motors or geared transmissions that are stiff and not back-drivable.

Our project sits in the gap none of those alternatives covers at once:

1. Precise, back-drivable torque control through field-oriented control.
2. Vibrotactile alerts at the soft range-of-motion limits.
3. Real-time compensation detection grounded in the clinical kinematic literature.
4. A single central tare button so every subsystem uses the same arm-orientation reference.
5. A gamified patient dashboard with a mirror-therapy visualisation.

All hardware, firmware and software are released openly. The full prototype costs approximately **€540**.

### 1.5 Project objectives

- Provide adjustable assistive torque that responds to the patient's own movement intent, instead of dragging the arm through a fixed trajectory.
- Use two IMUs both for compensation detection and for full 3D arm-pose estimation.
- Deliver short, clear vibrotactile alerts at the soft range-of-motion limits.
- Expose every therapy parameter through a real-time dashboard, with gamified training modes for patient engagement.
- Document the design well enough that a third party can rebuild it from this GitHub repository.

---

## 2. Supplies — Bill of materials

All components are available in the EU. CAD files for the printed parts are in *`/hardware/cad/`*, the PCB design in *`/hardware/pcb/`*. **Estimated total prototype cost: approximately €540.**

### 2.1 Actuation and motor control

| Component | Short description | Source | Cost (€) |
|---|---|---|---|
| **ODrive S1** | Single-axis FOC motor controller, 12–50&nbsp;V input, isolated UART. | ODrive Europe | 155 |
| **BLDC motor** (D5312s 330KV) | High-pole outrunner, smooth at low speed, fully back-drivable. | ODrive Robotics | 85 |
| **AMT110 incremental encoder** | Configurable 48–2048&nbsp;CPR via DIP switches, index pulse for homing. | Digi-Key | 30 |
| **Mean Well RSP-320-24 PSU** | 24&nbsp;V / 13.4&nbsp;A enclosed PSU, fits inside the ODrive 12–50&nbsp;V envelope. | Farnell | 65 |
| **230&nbsp;V safety switch with E-stop** | briidea single-phase switch with mushroom E-stop on the mains feed. | Amazon DE | 25 |

### 2.2 Microcontroller and communication

| Component | Short description | Source | Cost (€) |
|---|---|---|---|
| **Arduino Micro** (ATmega32U4) | Has a separate hardware UART (`Serial1`) and 16-bit Timer4 (see §3.2). | Arduino Official | 25 |
| Micro-USB data cable | Serial link between Arduino and host PC. | Any | 5 |
| **Custom PCB** | Self-designed board replacing the breadboard prototype. Gerbers in *`/hardware/pcb/`*. | Self-fabricated | 25 |
| Jumper wires | Dupont jumpers for the off-board IMU and Drake modules. | Any | 6 |

### 2.3 Sensors and haptic feedback

| Component | Short description | Source | Cost (€) |
|---|---|---|---|
| **MPU-6050 IMU × 2** | 6-DOF inertial sensor on I²C. Address `0x68` (upper arm) and `0x69` (forearm). | Adafruit #3886 | 2 × 9 |
| **DRV2605L haptic driver** | I²C-controlled LRA driver, PWM input mode, address `0x5A`. | Adafruit #2305 | 9 |
| **Drake LRA actuator** | Linear resonant actuator with a fast rise time for crisp clicks. | Drake / TacHammer | 20 |

### 2.4 Mechanical structure

| Component | Short description | Source | Cost (€) |
|---|---|---|---|
| **3D-printed exoskeleton frame** | Self-designed upper-arm cuff, elbow housing, motor bracket and forearm linkage. PLA. STEP/STL files in *`/hardware/cad/`*. | Self-designed | 60 |
| Fasteners + Velcro straps | M3/M4 screws and 50&nbsp;mm Velcro for patient attachment. | Local hardware store | 12 |

---

## 3. Methods

All source files are in the public GitHub repository: Arduino firmware in *`/firmware/`*, Python dashboard in *`/software/`*, hardware files (CAD, schematics) in *`/hardware/`*.

### 3.1 System architecture

The system is split into three layers. Each layer runs on its own hardware so that the time-critical motor loop is never blocked by the dashboard.

- **Layer 1 — Python dashboard (PC).** Therapy mode, range-of-motion (ROM) and assist parameters, live graphs, the compensation monitor, the gamified training, the 3D pose view with optional mirror-therapy overlay, the UDP stream to Unity, and a runtime PID-tuning panel for the ODrive. Talks to Layer 2 over USB serial.
- **Layer 2 — Arduino Micro.** A non-blocking finite-state machine. Reads both IMUs over I²C, triggers the vibrotactile pulse, parses commands from the dashboard, and relays torque set-points and PID writes to the ODrive over UART (`Serial1`) at 115&nbsp;200&nbsp;baud.
- **Layer 3 — ODrive S1.** Field-oriented current control at about 8&nbsp;kHz with encoder feedback. Runs independently of the upper layers — if the Arduino link stalls, the ODrive simply holds its last commanded torque.

### 3.2 Why we chose these components

Each choice avoids a specific limitation we hit (or expected to hit) with the obvious alternative.

- **ODrive S1 + BLDC instead of a geared servo.** A geared servo is stiff and not back-drivable, which is a safety problem when a patient is strapped in. The ODrive controls motor current and therefore torque, so the joint behaves like an active spring with no backlash.
- **Arduino Micro instead of Arduino Uno.** The Micro's ATmega32U4 has a separate hardware UART called `Serial1`, while the USB acts as its own CDC port. The UART talks to the ODrive and the USB streams telemetry to the dashboard — both at the same time. On the Uno, the only UART is shared with the USB-to-serial converter, so this architecture is not possible. The Micro also exposes 16-bit Timer4, which we use for hardware PWM into the DRV2605L.
- **Drake LRA instead of an ERM coin motor.** An ERM motor needs 50–100&nbsp;ms to spin up and produces a diffuse buzz. The Drake LRA reaches full amplitude in under 15&nbsp;ms, so it can deliver a sharp "click" at the soft limits.
- **Two MPU-6050 IMUs instead of one motor encoder.** The encoder only knows the angle inside the exoskeleton frame. It cannot tell whether the patient is holding the arm vertically or horizontally, and it cannot detect a shoulder lift. Two IMUs solve both problems.

### 3.3 Mechanical and electrical build

#### Mechanical assembly

The frame consists of an upper-arm cuff, an elbow housing that carries the BLDC motor coaxially with the anatomical elbow joint, and a forearm linkage that ends in a Velcro cuff. Coaxiality between motor shaft and elbow axis is critical — even a few degrees of misalignment puts a parasitic torque on the wrist or shoulder during therapy. The AMT110 encoder mounts on the back of the motor with the manufacturer's adapter ring. The two IMUs clip onto the upper-arm cuff (`0x68`) and forearm cuff (`0x69`). The Drake actuator sits on the inside of the forearm cuff so the pulse couples directly to the skin. All structural parts are printed in PLA. STEP/STL files are in *`/hardware/cad/`*.

#### Electrical wiring

All inter-board connections sit on a self-designed PCB. The Arduino plugs into pin headers so it can be removed without de-soldering, and the IMU breakouts and Drake actuator connect through pluggable headers. The PCB removes the loose-contact and noise problems we had on the breadboard.

The full electrical schematic is shown below.

![Electrical schematic of the HapticElbow system](docs/electrical_schematic.png)

Key connections:

- **Power.** 230&nbsp;V mains → safety switch with E-stop → Mean Well 24&nbsp;V PSU → ODrive S1. The Arduino draws its 5&nbsp;V from the ODrive's regulated output, sharing a common ground. Pressing the red E-stop cuts mains to the PSU, so the bus collapses and the motor becomes immediately back-drivable.
- **UART.** ODrive GPIO7 (TX) connects to Arduino D0 (RX1), and ODrive GPIO8 (RX) connects to Arduino D1 (TX1), at 115&nbsp;200&nbsp;baud.
- **I²C bus** on Arduino SDA = pin&nbsp;2 and SCL = pin&nbsp;3: two MPU-6050 IMUs and one DRV2605L share the same bus.
- **Haptic PWM.** Arduino D13 (Timer4 OCR4A) → DRV2605L IN/TRIG pin.

### 3.4 ODrive configuration

The ODrive's default behaviour is to run a full motor and encoder calibration sweep on every power-up. That sweep needs at least one full motor revolution. Our exoskeleton frame only allows the elbow to move from **0° (fully extended) to 110° (fully flexed)**, so a full sweep would crash the linkage. To avoid this, we run a one-time calibration with the motor uncoupled, save it to the ODrive's non-volatile memory, and tell the ODrive to skip the sweep on every subsequent boot.

#### 3.4.1 DC-bus protection

We set the trip levels just outside the 24&nbsp;V nominal operating point of the PSU. The overvoltage trip catches the regenerative spikes that occur when the motor decelerates the arm; the undervoltage trip catches brownouts.

```python
odrv0.config.dc_bus_overvoltage_trip_level  = 25     # V — 1 V above the 24 V PSU rail
odrv0.config.dc_bus_undervoltage_trip_level = 22     # V — 2 V below to ignore short dips
```

#### 3.4.2 Motor thermistor disabled

We do not run a motor thermistor on the prototype. At the assistive torques we use (1.5&nbsp;Nm continuous, 4&nbsp;Nm peak) the stator stays cool over a 20-minute session, so the ODrive's temperature input is disabled to prevent false thermal foldback.

#### 3.4.3 UART link

UART_A is enabled at 115&nbsp;200&nbsp;baud and mapped to GPIO7 (TX) and GPIO8 (RX).

```python
odrv0.config.enable_uart_a   = True
odrv0.config.uart_a_baudrate = 115200
odrv0.config.gpio7_mode      = GpioMode.UART_A   # TX
odrv0.config.gpio8_mode      = GpioMode.UART_A   # RX
```

#### 3.4.4 Persistent calibration with the encoder index

The AMT110 emits one index pulse per shaft revolution. We use that pulse as the absolute angle reference. The motor and encoder-offset calibration is performed once with the motor uncoupled from the linkage and then flagged as `pre_calibrated`, so it survives every reboot.

```python
odrv0.axis0.encoder.config.use_index                  = True
odrv0.axis0.motor.config.pre_calibrated               = True
odrv0.axis0.encoder.config.pre_calibrated             = True
odrv0.axis0.config.startup_motor_calibration          = False
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_encoder_index_search       = True
```

#### 3.4.5 Conservative limits during the index search

The index search is still potentially risky because the motor rotates without knowing where the mechanical end-stops are. We set the velocity and current ceilings low during this phase.

```python
odrv0.axis0.config.vel_limit   = 2     # turns/s — very slow search
odrv0.axis0.config.current_lim = 2     # A       — low torque ceiling
odrv0.save_configuration()
```

During assembly we rotated the motor shaft until the encoder index lands inside the operational arc (around 50° of flexion). If the index sits outside this arc, the ODrive can never find it during startup.

#### 3.4.6 Runtime PID tuning

The dashboard has a tab called "ODrive Tuning" with four sliders. Each slider sends a high-level command (for example `SET_POS_GAIN:20.0`) to the Arduino, which forwards the matching ODrive command over UART. Changes take effect immediately but are not written to non-volatile memory, so a power-cycle restores the saved settings. The four exposed parameters are `pos_gain` (position controller P gain), `vel_gain` (velocity P gain), `vel_integrator_gain` (velocity I gain) and `input_filter_bandwidth` (command-shaping bandwidth).

### 3.5 Control logic

The therapy controller runs on the Arduino as a non-blocking finite-state machine. There is **no `delay()` anywhere in the main loop**, because a `delay()` call would freeze the loop and the ODrive serial channel would time out. The machine moves through five states: `WAITING_FOR_INDEX`, `WAITING_FOR_STRAIGHT`, `MOVING_TO_START`, `READY` and `THERAPY_ACTIVE`. The motor only produces torque when the patient is actively trying to move, and the strength of that help is a live parameter set by the therapist.

#### 3.5.1 Assistive mode

The encoder velocity is read every control cycle and smoothed with a single-pole IIR low-pass filter:

```
v_filt = 0.80 · v_filt_previous + 0.20 · v_raw
```

The weight 0.80 is the empirical sweet spot we found between latency (a higher weight makes the output sluggish) and noise (a lower weight makes the assist torque flicker).

Based on the filtered velocity, the controller picks one of three regimes:

- **Forward intent** — the patient initiates flexion (`v_filt > 0.02 turns/s`). The threshold 0.02&nbsp;turns/s (around 7°/s) sits just above the encoder noise floor, so it only triggers on real motion. An assistive torque is added:

  ```
  assist = (v_filt − VELOCITY_THRESHOLD) · ASSIST_RAMP_FACTOR
  assist = clip(assist, 0, ASSIST_TORQUE)
  ```

  `ASSIST_RAMP_FACTOR` (ramp rate) and `ASSIST_TORQUE` (ceiling) are live dashboard parameters. The default ceiling is 0.60&nbsp;Nm, because a forearm plus hand has roughly 3&nbsp;Nm of gravity torque at full extension — 0.60&nbsp;Nm helps clearly without taking over. The slider goes up to 4&nbsp;Nm for stronger patients.

- **Backward motion** — `v_filt < −0.02 turns/s`. The motor must never push the arm back automatically, so the assist branch is disabled. A configurable viscous damping `ASSIST_EXTENSION_DAMPING · v_filt` supports the eccentric phase on the way down.

- **Near rest** — `|v_filt| ≤ 0.02 turns/s`. A small baseline damping `BASE_DAMPING · v_filt` keeps the joint calm. The default value 0.05 is light enough not to feel sticky.

#### 3.5.2 Resistive (training) mode

For strength training the assist branch is disabled. Above the velocity threshold a heavy training damping kicks in:

```
if |v_filt| > VELOCITY_THRESHOLD:
    damping = − TRAINING_RESISTANCE · v_filt
else:
    damping = − BASE_DAMPING       · v_filt
```

`TRAINING_RESISTANCE` defaults to 2.50 and is adjustable in the range 0.50–4.00. The lower bound is light enough to feel as a hint, the upper bound is around what most healthy adults can still move smoothly with one arm.

#### 3.5.3 Free-ride mode (used during ROM measurement)

Both assist and damping are zero — the motor outputs no torque and the joint is fully back-drivable. The dashboard switches to this mode automatically when a ROM measurement starts.

#### 3.5.4 Soft-limit virtual spring

The only spring-like behaviour in the controller is the soft-limit virtual spring. It activates only when the angle crosses a soft limit set by the therapist:

```
if angle < limit_min:
    spring = (limit_min − angle) · VIRTUAL_SPRING_STIFFNESS
elif angle > limit_max:
    spring = (limit_max − angle) · VIRTUAL_SPRING_STIFFNESS
else:
    spring = 0     # normal therapy law applies
```

The default stiffness is 0.10&nbsp;Nm/°, so a 10° overshoot produces a 1&nbsp;Nm push back — clearly noticeable but not jarring. We tried 0.30&nbsp;Nm/° first; it felt like hitting a hard wall and bounced the arm past the limit. Lowering the stiffness and pairing it with a vibrotactile pulse (§3.7) produced a much more natural boundary cue.

#### 3.5.5 Torque rate limit and safety guard

The commanded torque is rate-limited by 0.015&nbsp;Nm per loop. At a 100&nbsp;Hz loop rate that is a slew rate of 1.5&nbsp;Nm/s, which is below the threshold at which the change feels abrupt. The torque is then hard-clipped to `± SAFETY_TORQUE_MAX` (default 1.5&nbsp;Nm, max 4&nbsp;Nm). On top of that, a separate guard sets the torque to zero whenever the measured velocity exceeds `MAX_SAFE_VELOCITY = 0.8 turns/s` (about 288°/s) — a software last line of defence beyond the hardware E-stop.

#### 3.5.6 Soft-start phase

The very first impedance command at the moment of START used to produce an audible mechanical kick, because the controller and the loosely-coupled human arm reach equilibrium in a hurry. The firmware now enters a soft-start phase when `THERAPY_ACTIVE` begins:

- From 0 to 1.5&nbsp;s: only a transparent damping component (coefficient 0.25) is applied. No assist, no virtual spring. This gives the patient time to relax into the brace.
- From 1.5 to 3.0&nbsp;s: a quadratic ease-in gain ramps the assist and virtual-spring contributions from 0 to 1. Damping stays at full strength.
- After 3.0&nbsp;s: the full therapy law applies.

The two 1.5&nbsp;s windows are short enough not to feel slow but long enough for the human to anticipate the change.

#### 3.5.7 Vibration suppression

During demonstrations without an arm in the brace, the unloaded mechanism would oscillate at low frequency. The firmware adds a continuous high-pass damping term to the torque command:

```
vel_hf            = v_raw − v_filt
vibration_damping = − HF_DAMPING_GAIN · vel_hf
```

The signal `(v_raw − v_filt)` is the high-frequency residual of the velocity feedback. It is large during oscillation and near zero during smooth voluntary motion, so the term only fires when needed. The default gain is 1.5&nbsp;Nm·s/turn — enough to kill the oscillation in around half a second without affecting normal operation. It is exposed as the "Shock Reduction" slider in the dashboard.

### 3.6 Dual-IMU sensing

The two MPU-6050 IMUs serve two different functions. They are not redundant. Each addresses a separate problem in upper-limb rehabilitation.

#### 3.6.1 Compensation detection

This is the most involved part of the system. The current algorithm is a **reach-based, ratio-modulated, zone-aware detector**, built around the three principles from the literature (§1.3) [[7]](#7-references), [[8]](#7-references), [[9]](#7-references).

**What counts as a reach.** A reach starts when the combined angular velocity `|ω_motor| + |ω_imu1|` stays above 8°/s for at least 0.2&nbsp;s. The threshold 8°/s sits above involuntary tremor and IMU noise, and the 0.2&nbsp;s debounce filters out single noisy samples. The reach ends when the same combined velocity stays below 4°/s for 0.5&nbsp;s. The end threshold is lower than the start threshold to give the algorithm hysteresis, and the longer debounce avoids ending a slow reach prematurely.

Within a reach we track three quantities:

- `max_elbow_excursion` — the maximum absolute change in motor angle from the reach start.
- `max_shoulder_rise` — the maximum upward change in the upper-arm IMU angle, measured against the **session baseline** (the IMU1 angle captured when "Start Monitoring" was pressed, or refreshed when the central tare is used). Only counted while the upper arm is not currently descending.
- `net_motor_delta` and `net_imu1_delta` — the signed change between the reach start and the current sample, used for direction-aware decisions at the end of the reach.

**The score formula.** The score combines a shoulder-rise term with an elbow-credit term:

```
shoulder_score  = max(0, (max_shoulder_rise − 5°) · 100 / 40°)   # capped at 100
elbow_required  = max(5°, max_shoulder_rise · zone_ratio)
effective_elbow = max(0, max_elbow_excursion − 3°)
elbow_credit    = min(1, effective_elbow / elbow_required)
final_score     = shoulder_score · (1 − elbow_credit)
```

Each constant in this formula has a specific reason:

- **5°** — the floor on shoulder rise. IMU baseline noise plus the patient's small natural sway can easily be 3–4°. We do not start counting until the rise exceeds 5°.
- **40°** — the normalisation. A typical full reach lifts the upper arm by 30–45°. Dividing by 40° puts the score on a 0–100 scale.
- **3°** — the wobble subtraction. The elbow naturally rotates 1–3° during gross arm motion even when the patient does not actively bend it. Subtracting 3° prevents this background wobble from being interpreted as deliberate elbow use.
- **zone_ratio** — the expected elbow/shoulder ratio (see table below).

**(1) Zone-based ratio.** The expected ratio depends on the workspace zone of the upper arm [[8]](#7-references):

| Zone (IMU1 angle) | Ratio | Domain |
|---|---|---|
| **LOW** (< 30°) | 0.40 | Hand-to-mouth — the shoulder should stay still. |
| **MID** (30–90°) | 0.30 | Default reaching domain. |
| **HIGH** (> 90°) | 0.50 | Above horizontal — the shoulder is near its anatomical limit, so the elbow must take over. |

The MID ratio is lowest because at moderate elevation the shoulder naturally contributes a fair share. The LOW and HIGH ratios are higher because in those zones the elbow is supposed to do almost all of the work.

**(2) Wobble subtraction.** Subtracting the first 3° of elbow excursion is what prevents the algorithm from being fooled by incidental motion (see above).

**(3) Direction-aware finalisation.** When a reach ends with the upper-arm angle net below where it started (`net_imu1_delta < −5°`), the algorithm classifies it as a return-to-rest motion and does not score it [[9]](#7-references). The lowering phase that follows every outgoing reach used to produce false alarms; this rule removes them.

**Real-time behaviour.** During a reach the score updates live. If the patient lowers the shoulder mid-reach, the visible alarm subsides immediately. At the end of the reach the score is recomputed from the accumulated maxima together with the direction test, and a coloured bar is added to the per-reach history chart. Between reaches the level is forced back to GOOD, so a previous bad reach does not keep the alarm panel red.

**Threshold scaling.** The two default thresholds — warning at 25&nbsp;% and alarm at 40&nbsp;% — are scaled together by the "Sensitivity" slider. The values 25/40 are empirical and leave the therapist room to make the device stricter or more lenient per patient:

| Score | Status | Visual |
|---|---|---|
| Below warning | **GOOD** | Green panel |
| Between warning and alarm | **WARNING** | Amber panel |
| At or above alarm | **COMPENSATION** | Red panel with a short pulse; the event counter increments on the rising edge |

The session chart shows one coloured bar per completed reach (green, amber or red), with the score labelled above each bar. A CSV export button writes the sample log to disk for offline analysis.

#### 3.6.2 Full 3D arm-pose estimation

A single motor encoder only reports the relative angle between the two exoskeleton cuffs. It cannot tell an arm hanging vertically at 60° of flexion from an arm held horizontally at the same 60°. Two IMUs fix this — the upper-arm IMU anchors the proximal segment in the gravity frame, and the forearm IMU together with the motor encoder anchors the distal segment.

Both angles use the 3D-vector gravity formulation, instead of the more common 2D `atan2(Y, Z)` form. The 2D form becomes numerically unstable near ±90° of tilt, while the 3D form is well-conditioned everywhere [[10]](#7-references):

```
angle = atan2( ay, √(ax² + az²) ) · 180 / π
```

#### 3.6.3 Central tare

To make sure the 3D pose view, the compensation monitor and the mirror-therapy overlay all use the same arm reference, a single pair of tare buttons sits in the **dashboard header**, always visible regardless of the active tab.

- The "Tare Vertical" button captures the current pose as 0°/0° (arm hanging down).
- The "Tare Horizontal" button captures the current pose as 90°/0° (arm pointing forward).

Pressing either button stores the per-IMU offsets and propagates them to every subsystem. If the compensation monitor is active when a tare is performed, its session baseline is also reset.

After taring, the dual-IMU pose stream feeds three things: the real-time 3D stick figure in the dashboard, the mirror-therapy overlay (a translucent mirrored arm on the contralateral side), and a UDP datagram stream on port 5005 (`"upper_arm_angle,motor_angle"`) to a separate Unity scene for an optional head-mounted display.

### 3.7 Vibrotactile feedback

The Drake LRA is driven through the DRV2605L in PWM-input mode (mode&nbsp;3, LRA library&nbsp;6). The DRV2605L receives its drive signal on its IN/TRIG pin from Arduino D13, which is tied to hardware Timer4 and OCR4A. Using hardware PWM rather than software bit-banging means the pulse timing is unaffected by what the main control loop happens to be doing.

A naïve approach — applying a sustained PWM signal for the entire soft-limit overshoot — produces a mushy buzz because the LRA's internal mass keeps resonating. We use a short fixed pulse instead, with a 2.5&nbsp;s cooldown between pulses so the alert remains crisp:

```c
OCR4A = 255 · HAPTIC_STRENGTH   // full drive scaled by the dashboard slider
// non-blocking 50 ms hold managed by Timer4 — no delay() in the main loop
OCR4A = 0                       // hard cut
```

`HAPTIC_STRENGTH` is the "Haptic Strength" slider in the dashboard, on a 0–1 scale.

### 3.8 Python dashboard

The dashboard is built with **CustomTkinter** and uses **Matplotlib** for live plots. The source is in *`/software/gui_main.py`*. It parses a structured ASCII stream from the Arduino (`POS`, `VEL`, `TRQ`, `IMU1`, `IMU2`, `STATE`) and shows nine tabs.

| Tab | What it does |
|---|---|
| Project info | Title, authors and a short summary. |
| Settings | Mode switch (Assistive / Resistive), ROM soft limits, assist parameters, resistive coefficient, safety torque ceiling, haptic intensity, and the "Shock Reduction" slider (§3.5.7). |
| 3D arm simulation | A 3D stick figure driven by both IMUs, with an optional "Mirror Therapy" toggle that draws a translucent mirrored arm on the other side of the body [[11]](#7-references). |
| Position control | Preset (0°, 50°, 100°) and slider-based commanded angles, used for setup. |
| ROM test | Switches to free-ride mode and records the patient's minimum and maximum reachable angle. |
| Anti-Compensation | The per-reach monitor of §3.6.1: status panel, live compensation score with progress bar, the live "SHOULDER ↑" and "ELBOW Δ" mini-metrics, session statistics ("Total Reaches", "Good Reaches", "Compensations"), and the per-reach history bar chart. |
| Game Mode | Two gamified modes: "Catch Blocks" (a Fitts-style hold task) and "Ghost Arm (Rhythm)" (a rhythm task with adjustable tempo). |
| Live Graphs | Angle, velocity, acceleration and commanded torque over time. |
| ODrive Tuning | Runtime PID adjustment (§3.4.6): `pos_gain`, `vel_gain`, `vel_integrator_gain` and `input_filter_bandwidth`. |

The "Tare Vertical" and "Tare Horizontal" buttons live in the always-visible header above the tab strip.

### 3.9 Lessons from troubleshooting

- **Breadboard to custom PCB.** The early breadboard prototype had intermittent contacts and picked up noise on the I²C lines. Migrating to a self-designed PCB eliminated the loose-wire failures.
- **No `delay()` in the firmware.** An early version used `delay()` to time the haptic pulse, which froze the main loop for 50&nbsp;ms and caused the ODrive serial link to time out. The current firmware contains no `delay()` inside the therapy loop. The haptic pulse runs on Timer4 in the background. We mark this clearly in the code as a rule that must not be broken.
- **Velocity-filter weight.** The first assistive law used the raw encoder velocity, which made the assist torque flicker unpleasantly. An IIR weight of 0.80 was the empirical sweet spot between latency and smoothness.
- **2D versus 3D IMU formula.** The first IMU implementation used `atan2(Y, Z)`, which collapsed near 90° of tilt. Switching to the 3D form `atan2(Y, √(X² + Z²))` made the angle robust across the entire working envelope [[10]](#7-references).
- **Virtual-spring stiffness.** At 0.30&nbsp;Nm/° the soft limit felt like a hard wall and bounced the arm out of range. Reducing it to 0.10&nbsp;Nm/° and pairing it with the haptic pulse produced a far more natural boundary cue.
- **Encoder-index placement.** In one assembly attempt the index ended up just outside the operational arc, so the ODrive could never finish its startup index search. Re-clocking the motor shaft so the index sits around 50° of flexion solved the issue.
- **Compensation algorithm — three generations.** The first version was a sliding-window detector on raw IMU samples; it triggered constantly and required the user to "pump down" the score with extra good reaches. The second version used a geometric coherence test (`imu2 ≈ imu1 + motor`), but the accelerometer-derived pitch is corrupted by tangential acceleration during dynamic motion, so it produced false alarms even on clean elbow flexions. The current reach-based, ratio-modulated, zone-aware detector (§3.6.1), grounded in the clinical kinematic literature [[7]](#7-references), [[8]](#7-references), [[9]](#7-references), is both more robust and easier to interpret.
- **Soft-start at therapy activation.** Without the soft-start phase (§3.5.6), the first impedance command produced an audible mechanical kick. Splitting the activation into a 1.5&nbsp;s damping-only phase followed by a 1.5&nbsp;s quadratic ramp eliminated this.
- **SPARC smoothness metric removed.** We briefly experimented with a Spectral Arc Length display as a smoothness marker. SPARC is well-validated for discrete reaches, but proved unreliable when applied to a continuous sliding window over mixed motion/rest data. We replaced it with the per-reach compensation chart, which is also a smoothness proxy by construction.

---

## 4. Discussion

### 4.1 What the device does well

The velocity-triggered assist law (§3.5.1) makes the device behave as a cooperative partner. It is quiet when the patient is at rest, gradually contributes torque when the patient initiates motion, and never overrides the patient's direction. This matches the assist-as-needed paradigm [[4]](#7-references).

Two late additions turned out to be more important than expected. The soft-start phase (§3.5.6) eliminated the audible kick at the moment of START, which would otherwise have been the first thing a patient experienced. The high-pass vibration-suppression term (§3.5.7) made the unloaded mechanism quiet during demonstrations. Neither was in the original design — both came out of real bench testing.

The compensation monitor reliably flagged deliberate shoulder-elevation cheats (lifting the shoulder by 10° or more during attempted flexion) and stayed quiet during clean elbow movements with the upper arm held still. The graduated, ratio-based credit means a small natural shoulder co-activation — for example a slight upward drift during a forward reach — is not penalised, while a stuck elbow paired with an actively moving shoulder is detected even when the per-reach delta is small. The 3° wobble subtraction (§3.6.1) is what makes this distinction possible: it filters out the natural incidental elbow rotation that occurs during gross arm motion. The direction-aware finalisation (§3.6.1) is what removes the false alarms during the natural lowering phase.

### 4.2 Constraints and observed limitations

- **IMU drift.** The MPU-6050 has no magnetometer. We sidestep this by using the gravity vector directly — robust against drift but blind to rotations about the vertical axis. A 9-DOF replacement (such as the Bosch BNO085) with on-board sensor fusion would lift this constraint.
- **Mechanical compliance.** Some 3D-printed adapter parts flex slightly under peak torque. Critical load-bearing parts would benefit from CNC-machined aluminium.
- **Compensation thresholds calibrated against healthy data.** The zone ratios (0.40 / 0.30 / 0.50) and the wobble subtraction (3°) come from healthy-reaching kinematic studies. For clinical deployment they would need recalibration against a cohort of stroke survivors.
- **Trunk compensation not detected.** The current dual-IMU setup catches shoulder elevation but not forward trunk leaning, which is the second common compensation mode after shoulder lift. A third IMU on the sternum is the natural extension.
- **Validation scope.** All testing was performed with healthy volunteers in a lab environment. Clinical validation with stroke survivors would be needed before any deployment beyond an engineering proof-of-concept.

---

## 5. Conclusion and future work

### 5.1 Summary

HapticElbow is a low-cost open-source elbow exoskeleton for upper-limb neurorehabilitation. Its main technical contribution is a velocity-triggered proportional assist law, implemented with current-mode FOC on the ODrive S1, combined with a dual-IMU subsystem that detects shoulder compensation and reconstructs the 3D arm pose at the same time. The compensation detector is a per-reach evaluator with ratio, wobble and zone modulation, grounded in the clinical kinematic literature [[7]](#7-references), [[8]](#7-references), [[9]](#7-references). Two safety additions — the soft-start phase and the high-pass vibration-suppression term — make the device comfortable to use even outside ideal lab conditions. Vibrotactile soft-limit alerts via a Drake LRA, a non-blocking Arduino firmware and a Python dashboard with gamified modes and a mirror-therapy overlay complete the system. The total component cost is approximately €540 and every artefact needed to rebuild the device is in this repository.

### 5.2 Future work

- **Trunk IMU.** Adding a third MPU-6050 on the sternum would extend the monitor to trunk-leaning compensation.
- **EMG-based intent detection.** Surface EMG over biceps and triceps would shift detection upstream and allow the device to assist before motion begins.
- **9-DOF IMU upgrade.** The Bosch BNO085 has on-board sensor fusion and would eliminate yaw drift.
- **Aluminium structural parts.** CNC-machined aluminium for the load-bearing parts would remove the residual compliance.
- **Wireless operation.** Moving the dashboard from a desktop PC to a small SBC (Raspberry Pi 4/5) mounted on the harness would untether the patient.
- **Patient study.** A feasibility study with stroke survivors under ethical approval, measuring ROM recovery, compensation incidence and session adherence over a multi-week protocol, is the natural next step.

### 5.3 Lessons learned

- Never use `delay()` inside a robot control loop — a 10&nbsp;ms block is enough to stall the ODrive serial channel.
- Drive the LRA through the DRV2605L in LRA-mode with hardware PWM. Software bit-banging produces barely perceptible vibration.
- The AMT110 encoder-index location is a mechanical design constraint. Verify that the index sits inside the operational arc before final assembly.
- Use the 3D gravity-vector form of the IMU tilt angle from day one. The 2D form fails near 90° and the failure looks like a glitch rather than a singularity.
- Separate the therapy controller from the safety supervisor. Hard guards (velocity ceiling, torque clip, rate limit, vibration-suppression damping) must run regardless of the active therapy mode.
- Ground rehabilitation algorithms in published clinical kinematics rather than ad-hoc heuristics. Our compensation detector needed three redesigns before the literature-anchored approach (§3.6.1) became stable and interpretable.
- Add a soft-start phase to any actively driven physical-interaction robot. The patient's body is never as rigidly coupled as the bench test suggests.
- An absolute encoder would be a better choice than the AMT110 incremental encoder for this application — it would remove the index-search phase entirely.

---

## 6. Abbreviations

| Abbreviation | Meaning |
|---|---|
| AAN | Assist-As-Needed |
| BLDC | Brushless Direct Current (motor) |
| CAD | Computer-Aided Design |
| CPR | Counts Per Revolution |
| DC | Direct Current |
| DOF | Degrees Of Freedom |
| EMG | Electromyography |
| ERM | Eccentric Rotating Mass (motor) |
| FOC | Field-Oriented Control |
| GUI | Graphical User Interface |
| I²C | Inter-Integrated Circuit (serial bus) |
| IIR | Infinite Impulse Response (filter) |
| IMU | Inertial Measurement Unit |
| LRA | Linear Resonant Actuator |
| PCB | Printed Circuit Board |
| PID | Proportional-Integral-Derivative |
| PLA | Polylactic Acid (3D-printing material) |
| PSU | Power Supply Unit |
| PWM | Pulse-Width Modulation |
| ROM | Range Of Motion |
| RPSS | Reaching Performance Scale for Stroke |
| SBC | Single-Board Computer |
| SPARC | Spectral Arc Length (smoothness metric) |
| UART | Universal Asynchronous Receiver-Transmitter |
| UDP | User Datagram Protocol |
| WHO | World Health Organization |

---

## 7. References

[1] World Health Organization, "The top 10 causes of death," Geneva, Switzerland, Dec. 2020. [Online]. Available: https://www.who.int/news-room/fact-sheets/detail/the-top-10-causes-of-death

[2] J. A. Kleim and T. A. Jones, "Principles of experience-dependent neural plasticity: Implications for rehabilitation after brain damage," *J. Speech Lang. Hear. Res.*, vol. 51, no. 1, pp. S225–S239, Feb. 2008, doi: [10.1044/1092-4388(2008/018)](https://doi.org/10.1044/1092-4388(2008/018)).

[3] J. Mehrholz, M. Pohl, T. Platz, J. Kugler, and B. Elsner, "Electromechanical and robot-assisted arm training for improving activities of daily living, arm function, and arm muscle strength after stroke," *Cochrane Database Syst. Rev.*, no. 9, Art. no. CD006876, 2018, doi: [10.1002/14651858.CD006876.pub5](https://doi.org/10.1002/14651858.CD006876.pub5).

[4] L. Marchal-Crespo and D. J. Reinkensmeyer, "Review of control strategies for robotic movement training after neurologic injury," *J. NeuroEng. Rehabil.*, vol. 6, no. 1, p. 20, Jun. 2009, doi: [10.1186/1743-0003-6-20](https://doi.org/10.1186/1743-0003-6-20).

[5] K. Bark, J. W. Wheeler, G. Lee, J. Redmond, and A. M. Okamura, "Comparison of skin stretch and vibrotactile stimulation for feedback of proprioceptive information," in *Proc. IEEE Symp. Haptic Interfaces for Virtual Environment and Teleoperator Syst.*, 2009, pp. 71–78, doi: [10.1109/HAPTIC.2009.4810812](https://doi.org/10.1109/HAPTIC.2009.4810812).

[6] M. F. Levin, J. A. Kleim, and S. L. Wolf, "What do motor recovery and compensation mean in patients following stroke?" *Neurorehabil. Neural Repair*, vol. 23, no. 4, pp. 313–319, May 2009, doi: [10.1177/1545968308328727](https://doi.org/10.1177/1545968308328727).

[7] A. Schwarz, J. M. Veerbeek, J. P. O. Held, J. C. Buurke, and A. R. Luft, "Measures of interjoint coordination post-stroke across different upper limb movement tasks," *Front. Bioeng. Biotechnol.*, vol. 8, art. 620805, Jan. 2021, doi: [10.3389/fbioe.2020.620805](https://doi.org/10.3389/fbioe.2020.620805).

[8] M. F. Levin, J. Desrosiers, D. Beauchemin, N. Bergeron, and A. Rochette, "Development and validation of a scale for rating motor compensations used for reaching in patients with hemiparesis: the Reaching Performance Scale," *Phys. Ther.*, vol. 84, no. 1, pp. 8–22, Jan. 2004, doi: [10.1093/ptj/84.1.8](https://doi.org/10.1093/ptj/84.1.8).

[9] M. C. Cirstea and M. F. Levin, "Compensatory strategies for reaching in stroke," *Brain*, vol. 123, no. 5, pp. 940–953, May 2000, doi: [10.1093/brain/123.5.940](https://doi.org/10.1093/brain/123.5.940).

[10] M. Pedley, "Tilt sensing using a three-axis accelerometer," Freescale Semiconductor Application Note AN3461, Rev. 6, Mar. 2013. [Online]. Available: https://www.nxp.com/docs/en/application-note/AN3461.pdf

[11] H. Thieme, N. Morkisch, J. Mehrholz, M. Pohl, J. Behrens, B. Borgetto, and C. Dohle, "Mirror therapy for improving motor function after stroke," *Cochrane Database Syst. Rev.*, no. 7, Art. no. CD008449, 2018, doi: [10.1002/14651858.CD008449.pub3](https://doi.org/10.1002/14651858.CD008449.pub3).

---

*This README documents the complete design and implementation of HapticElbow. All hardware files, firmware and software are released under the licence specified in this repository.*
