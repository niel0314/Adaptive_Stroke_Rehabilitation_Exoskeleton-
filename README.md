# HapticElbow

**An Open-Source Adaptive Elbow Exoskeleton for Stroke Rehabilitation**

*B-KUL-T4lMD2 · Haptic Interfaces Experience*

**Authors:** Niel Boon · Senne Peeters
**Institution:** KU Leuven · Group T · Department of Mechanical Engineering
**Date:** May 2026

---

## Table of contents

1. [Introduction](#1-introduction)
   - [1.1 Clinical motivation](#11-clinical-motivation)
   - [1.2 Why a haptic interface](#12-why-a-haptic-interface)
   - [1.3 The compensatory-movement problem](#13-the-compensatory-movement-problem)
   - [1.4 Existing solutions and the gap addressed](#14-existing-solutions-and-the-gap-addressed)
   - [1.5 Project objectives](#15-project-objectives)
2. [Supplies — Bill of materials](#2-supplies--bill-of-materials)
   - [2.1 Actuation and motor control](#21-actuation-and-motor-control)
   - [2.2 Microcontroller and communication](#22-microcontroller-and-communication)
   - [2.3 Sensors and haptic feedback](#23-sensors-and-haptic-feedback)
   - [2.4 Mechanical structure](#24-mechanical-structure)
3. [Methods](#3-methods)
   - [3.1 System architecture](#31-system-architecture)
   - [3.2 Rationale for key component choices](#32-rationale-for-key-component-choices)
   - [3.3 Mechanical and electrical build](#33-mechanical-and-electrical-build)
   - [3.4 ODrive configuration](#34-odrive-configuration)
   - [3.5 Control logic](#35-control-logic)
   - [3.6 Dual-IMU sensing](#36-dual-imu-sensing)
   - [3.7 Vibrotactile feedback](#37-vibrotactile-feedback)
   - [3.8 Python GUI](#38-python-gui)
   - [3.9 Notable troubleshooting and iterations](#39-notable-troubleshooting-and-iterations)
4. [Discussion](#4-discussion)
5. [Conclusion and future work](#5-conclusion-and-future-work)
6. [References](#references)

---

## 1. Introduction

### 1.1 Clinical motivation

Neurological injuries, most prominently stroke, are a leading cause of long-term motor disability worldwide. The World Health Organization estimates that approximately 15 million people suffer a stroke each year and that around one third are left with permanent impairment [[1]](#references). A large fraction of survivors lose proximal control of the upper limb, directly compromising essential activities of daily living such as reaching, lifting and self-care.

Recovery is mediated by neuroplasticity: the central nervous system reorganises and forms new synaptic connections in response to repetitive, task-specific, voluntary practice [[2]](#references). The clinical evidence is unambiguous in two directions. First, electromechanical and robot-assisted arm training, when added to conventional therapy, significantly improves arm function and activities of daily living after stroke [[3]](#references), [[4]](#references). Second, the benefit is conditional on the patient remaining an active agent in the movement. Purely passive limb manipulation produces only marginal cortical reorganisation. This active-participation requirement is the foundation of the **assist-as-needed (AAN)** paradigm now widely advocated in rehabilitation robotics [[3]](#references).

### 1.2 Why a haptic interface

Motor learning is driven by sensorimotor feedback. Sigrist et al. showed in a comprehensive review that augmented haptic and multimodal feedback during motor training improves learning retention compared with visual-only protocols [[5]](#references). Two complementary haptic modalities are relevant for a rehabilitation device:

- **Kinaesthetic feedback** — torques and forces transmitted through the mechanical linkage. In our system this is realised by a back-drivable brushless motor driven in current (torque) mode, which can deliver smooth, low-impedance assistance without locking the joint in a stiff position.
- **Vibrotactile feedback** — high-frequency skin vibrations that convey discrete events. Bark et al. demonstrated that vibrotactile cues at the forearm during elbow tasks reduce path errors and accelerate motor learning by offloading information from the visual channel [[6]](#references).

### 1.3 The compensatory-movement problem

A well-documented but frequently overlooked issue in upper-limb rehabilitation is the **compensatory movement strategy**. When the elbow flexors are too weak to produce the requested motion, patients reflexively recruit proximal muscles, elevating the shoulder or rotating the trunk, so that the forearm *appears* to flex without genuine elbow activation [[7]](#references). Neurologically this is counter-productive: the maladaptive pattern is rehearsed instead of the target movement, and any robotic system that simply assists the joint torque without monitoring posture will silently reinforce it.

Wearable inertial measurement units (IMUs) have been validated as effective low-cost tools for tracking limb-segment orientation [[8]](#references), [[12]](#references). Mounting one IMU on the upper arm and one on the forearm enables real-time discrimination between true elbow flexion and proximal compensation.

Recent kinematic studies have made this discrimination both quantitative and task-aware:

- **Schwarz et al. (2020)** measured interjoint coordination on standardised upper-limb tasks and found that, in healthy reaching, the elbow excursion represents approximately **40–60 %** of the shoulder excursion, dropping to **0.1–0.3** in stroke survivors [[15]](#references).
- **Levin's Reaching Performance Scale for Stroke (RPSS)** extends this by partitioning the arm's workspace into zones with different expected coordination patterns: at low elevation the shoulder should remain essentially still (hand-to-mouth domain), at intermediate elevation a balanced shoulder–elbow contribution is normal, and above shoulder height the shoulder is at its anatomical limit so the elbow must take over for any further reach [[16]](#references).
- **Cirstea and Levin (2000)** established the methodological distinction between the *outgoing* (goal-directed) phase of a reach, which is clinically scored, and the *return-to-rest* phase, which is not [[17]](#references).

These three principles directly inform our compensation-monitor algorithm (§3.6.1).

### 1.4 Existing solutions and the gap addressed

Several established robotic platforms — the MIT-Manus / InMotion ARM [[9]](#references), ARMin [[10]](#references) and the Armeo family — have demonstrated robust clinical efficacy. They are, however, prohibitively expensive (€50 000 to €150 000), tied to dedicated clinical infrastructure and largely closed-source. Open-source alternatives have lowered the entry cost dramatically but typically rely on rigid stepper-motor or gear-train actuation that is neither back-drivable nor torque-controlled.

Our contribution targets the intersection that none of the above covers simultaneously:

1. Precise, back-drivable torque control via field-oriented current control.
2. Integrated vibrotactile limit alerts.
3. Real-time, literature-grounded compensation monitoring.
4. A unified one-click central tare for consistent IMU referencing across all subsystems.
5. A gamified patient-facing interface with a Mirror-Therapy visualisation.

All hardware files, firmware and software are released openly so that the device can be replicated for **around €540**.

### 1.5 Project objectives

- **O1.** Provide adjustable assistive torque that responds to the patient's own movement intent, rather than dragging the limb through a predetermined trajectory.
- **O2.** Use a dual-IMU configuration both for shoulder-compensation detection and for full real-time 3D arm-pose estimation.
- **O3.** Deliver crisp vibrotactile alerts at the soft range-of-motion (ROM) boundaries, independent of visual attention.
- **O4.** Expose every therapy parameter to the therapist via a real-time GUI, with gamified training modes for patient engagement.
- **O5.** Document the design completely enough that a third party can rebuild it from the GitHub repository.

---

## 2. Supplies — Bill of materials

All components were selected for commercial availability in the EU and full reproducibility. CAD files for the printed parts are in `/hardware/cad/`, the PCB design in `/hardware/pcb/`. **Estimated total prototype cost: ~€540.**

### 2.1 Actuation and motor control

| Component | Description | Source | Cost (€) |
|---|---|---|---|
| **ODrive S1** | Single-axis FOC controller, 12–50 V input, 2 kW continuous, isolated UART/CAN, on-board brake chopper. | ODrive Europe | ~155 |
| **BLDC motor** (e.g. D5312s 330KV) | High-pole-count outrunner; smooth low-speed torque, low cogging, fully back-drivable when de-energised. | ODrive Robotics | ~85 |
| **AMT110 incremental encoder** | CUI Devices, configurable 48–2048 CPR via DIP switches, dedicated index pulse for ODrive homing. | Digi-Key | ~30 |
| **Mean Well RSP-320-24 PSU** | 24 V / 13.4 A / 320 W enclosed switching PSU with active PFC, universal 88–264 VAC input, integrated OCP/OVP/OTP. Within ODrive 12–50 V envelope. | Farnell | ~65 |
| **230 V AC safety switch + E-stop** | briidea single-phase safety switch (Amazon DE, ASIN B0DXP2B96Y): 16 A rated, EU schuko, mushroom E-stop. Inserted on 230 V mains feeding the PSU. | Amazon DE | ~25 |

### 2.2 Microcontroller and communication

| Component | Description | Source | Cost (€) |
|---|---|---|---|
| **Arduino Micro** (ATmega32U4) | Chosen for its separate hardware UART (`Serial1`) and 16-bit Timer4 — both required (see §3.2). | Arduino Official | ~25 |
| Micro-USB data cable | Data-capable cable for serial link between Arduino and host PC running the GUI. | Any | ~5 |
| **Custom PCB** | Self-designed PCB that replaces the prototype breadboard. Sockets for Arduino (removable), DRV2605L footprint, screw-terminals for ODrive UART, pluggable headers for IMU breakouts and Drake actuator, 5 V / 3.3 V rails. Schematic, layout and Gerbers in `/hardware/pcb/`. | Self-fabricated | ~25 |
| Jumper wires (M-M, M-F) | Dupont jumpers for off-board modules (IMU breakouts, Drake actuator) plugging into the PCB through pin headers. | Any | ~6 |

### 2.3 Sensors and haptic feedback

| Component | Description | Source | Cost (€) |
|---|---|---|---|
| **MPU-6050 IMU × 2** | 6-DOF (3-axis accelerometer + 3-axis gyroscope) on I²C. Address `0x68` (upper arm) and `0x69` (forearm, AD0 pulled high). | Adafruit #3886 | 2 × 9 |
| **DRV2605L haptic driver** | I²C-controlled LRA/ERM driver. PWM input mode used here; address `0x5A`. | Adafruit #2305 | ~9 |
| **Drake LRA haptic actuator** | Linear Resonant Actuator with crisp <15 ms rise/fall time, ideal for click-like limit alerts. | Drake / TacHammer | ~20 |

### 2.4 Mechanical structure

| Component | Description | Source | Cost (€) |
|---|---|---|---|
| **Custom-designed exoskeleton frame** | Fully self-designed: upper-arm cuff, elbow bearing housing, motor bracket, forearm linkage. CAD optimised around the BLDC motor, encoder and IMU placements with the motor axis coaxial with the anatomical elbow joint. PETG. STEP/STL files in `/hardware/cad/`. | Self-designed | ~60 |
| Bearings + fasteners + Velcro straps | Deep-groove ball bearing for the elbow pivot, M3/M4 socket-head screws and hex nuts for assembly, 50 mm Velcro for patient attachment. | Local hardware store | ~12 |

---

## 3. Methods

All source files referenced below are in the public GitHub repository: `/firmware/` (Arduino `.ino`), `/software/` (Python GUI), `/hardware/` (CAD, schematics, Fritzing).

### 3.1 System architecture

The system is organised as a **three-layer distributed control stack**. Separating the layers by computational urgency keeps the time-critical motor loop running at full bandwidth on dedicated hardware, while the GUI and high-level logic remain free to use Python.

- **Layer 1 — High-level (Python GUI, PC).** Therapy-mode selection, ROM and assist parameters, real-time graphs, the compensation monitor, gamified training, the 3D pose view with optional Mirror-Therapy overlay, UDP stream to Unity and a runtime-PID tuning panel for the ODrive. Communicates with Layer 2 over USB serial.
- **Layer 2 — Mid-level (Arduino Micro).** Non-blocking finite-state machine, two-sensor I²C polling, vibrotactile trigger logic, parsing of GUI commands and relay of computed torque set-points and PID-tuning writes to the ODrive. Connects to Layer 3 over UART (`Serial1`) at 115 200 baud.
- **Layer 3 — Low-level (ODrive S1).** Closed-loop field-oriented current control at ~8 kHz with encoder feedback. Runs independently of upper layers; if Arduino communication stalls, the ODrive holds its last commanded torque safely.

### 3.2 Rationale for key component choices

Every component was chosen against an alternative that would have introduced a hard limitation later in the build.

- **ODrive S1 + BLDC over geared servo.** Conventional hobby servos and geared DC motors are stiff and not back-drivable, which is a significant safety risk in physical human–robot interaction. The ODrive controls motor current (and therefore torque) rather than position, so the joint behaves like an active element that can be both transparent (zero command) and assistive (positive command) without any of the backlash of a gearbox.
- **Arduino Micro over Arduino Uno.** The Micro's ATmega32U4 exposes a separate hardware UART (`Serial1`) alongside USB-CDC serial: the UART talks to the ODrive while USB simultaneously streams telemetry to the GUI. The Uno multiplexes USB and its only UART on the same pins, making this architecture impossible. The Micro also exposes 16-bit Timer4, used for hardware PWM into the DRV2605L.
- **Drake LRA over ERM vibration motor.** ERM motors have 50–100 ms mechanical rise and fall times and produce a diffuse buzz, unsuitable for a discrete "click" event. An LRA has <15 ms response and produces a sharply localised pulse, essential for the boundary-alert role.
- **Dual MPU-6050 IMUs over a single encoder.** The motor encoder reports the joint angle within the exoskeleton frame but says nothing about how the patient is holding the limb in space. Two IMUs are required to recover full posture and to detect proximal compensation.

### 3.3 Mechanical and electrical build

#### Mechanical assembly

The mechanical frame was designed from scratch in CAD around the chosen actuator stack — no commercial kit such as EduExo was used. The structure consists of an upper-arm cuff, an elbow bearing housing carrying the BLDC motor coaxially with the anatomical elbow joint, and a forearm linkage that terminates in a Velcro-strapped cuff. **Coaxiality between motor shaft and elbow axis is critical**: even a few degrees of misalignment manifests as a parasitic moment on the patient's wrist or shoulder during therapy. The AMT110 encoder sits on the rear face of the motor shaft via the manufacturer-supplied adapter ring. The two IMUs are clipped onto the upper-arm cuff (`0x68`) and forearm cuff (`0x69`) respectively; the Drake actuator is mounted on the inside of the forearm cuff so that its pulse couples directly to the skin. All structural parts are printed in PETG; STEP/STL files are in `/hardware/cad/`.

#### Electrical wiring

All inter-board connections live on a self-designed PCB. The Arduino is mounted in pin-header sockets so it can be lifted off without de-soldering, and the IMU breakouts and Drake actuator connect to the PCB through pluggable jumper headers. The soldered traces remove the reliability issues of a breadboard prototype (loose contacts, unintentional shorts, electromagnetic noise pickup) while keeping every individually-mounted module serviceable.

- **Power:** 230 V mains → briidea AC safety switch with integrated E-stop → Mean Well RSP-320-24 (24 V / 13.4 A) → ODrive S1 (locking-connector DC input). Pressing the red E-stop cuts 230 V to the PSU; its 24 V output collapses to zero and the BLDC becomes immediately back-drivable. The Arduino draws its 5 V from the ODrive's regulated output to share a common ground.
- **UART:** ODrive TX (GPIO7) → Arduino RX1 (D0); ODrive RX (GPIO8) → Arduino TX1 (D1). 115 200 baud.
- **I²C bus** (SDA = pin 2, SCL = pin 3 on the Micro): MPU-6050 #1 (`0x68`, upper arm), MPU-6050 #2 (`0x69` — AD0 pulled HIGH, forearm), DRV2605L (`0x5A`).
- **Haptic PWM:** Arduino D13 (Timer4 OCR4A) → DRV2605L IN/TRIG pin.

Full schematic and Gerbers: `/hardware/pcb/`.

### 3.4 ODrive configuration

The ODrive's default behaviour — full motor and encoder calibration on every power-up — is incompatible with a mechanically constrained exoskeleton, where the linkage permits only ~100° of travel. Allowing the controller to attempt a full motor revolution would crash the structure. A one-time configuration is therefore applied through `odrivetool` and persisted to the ODrive's non-volatile memory.

#### 3.4.1 Power-rail protection

The DC-bus protection trip levels are set just outside the nominal 24 V operating point of the Mean Well PSU. The overvoltage trip catches regenerative voltage spikes when the BLDC decelerates the limb; the undervoltage trip catches brown-outs and a missing main supply.

```python
odrv0.config.dc_bus_overvoltage_trip_level  = 25     # V
odrv0.config.dc_bus_undervoltage_trip_level = 22     # V
```

#### 3.4.2 Thermistor disabled

No motor thermistor is wired in the prototype — at the assistive torque levels we run (≤ 1.5 Nm continuous, ≤ 4 Nm peak), the motor stator stays comfortably cool over 20-minute sessions. The ODrive's temperature input is disabled to avoid spurious thermal-foldback events.

#### 3.4.3 UART link to the Arduino

UART_A is enabled at 115 200 baud and routed to ODrive GPIO 7 (TX) and GPIO 8 (RX).

```python
odrv0.config.enable_uart_a       = True
odrv0.config.uart_a_baudrate     = 115200
odrv0.config.gpio7_mode          = GpioMode.UART_A   # TX
odrv0.config.gpio8_mode          = GpioMode.UART_A   # RX
```

#### 3.4.4 Encoder index search and persistent calibration

The AMT110 emits a single index pulse per shaft revolution. We use this pulse as the absolute angular reference. Crucially, the one-time motor and encoder-offset calibration are performed once with the motor uncoupled from the linkage, then flagged as `pre_calibrated` so they survive every subsequent boot. **This avoids the destructive full-revolution calibration sweep** that the ODrive would otherwise attempt on each startup.

```python
odrv0.axis0.encoder.config.use_index                  = True
odrv0.axis0.motor.config.pre_calibrated               = True
odrv0.axis0.encoder.config.pre_calibrated             = True
odrv0.axis0.config.startup_motor_calibration          = False
odrv0.axis0.config.startup_encoder_offset_calibration = False
odrv0.axis0.config.startup_encoder_index_search       = True
```

#### 3.4.5 Conservative limits during index search

The index search itself is potentially dangerous because the motor rotates without knowing where the mechanical end-stops are. Both `vel_limit` and `current_lim` are deliberately set very low.

```python
odrv0.axis0.config.vel_limit    = 2     # turns/s (very slow search)
odrv0.axis0.config.current_lim  = 2     # A   (low torque ceiling)
odrv0.save_configuration()
```

During assembly the motor shaft was physically rotated until the encoder index fell inside the operational arc of the exoskeleton (around the 50° flexion position). If the index sits outside that arc, the motor would never find it during startup.

#### 3.4.6 Runtime PID tuning via Arduino UART passthrough

The GUI exposes a dedicated **ODrive Tuning tab** that lets the therapist or engineer experiment with the controller gains without restarting the device. Each slider sends a high-level command (e.g. `SET_POS_GAIN:20.0`) to the Arduino, which forwards the equivalent ODrive ASCII command (`w axis0.controller.config.pos_gain 20.0`) over UART. Changes take effect immediately but are **not** written to non-volatile memory; every power-cycle restores the saved settings.

Exposed parameters:
- `pos_gain` — position controller P gain.
- `vel_gain` — velocity controller P gain.
- `vel_integrator_gain` — velocity controller I gain.
- `input_filter_bandwidth` — command shaping bandwidth.

### 3.5 Control logic

The therapy controller runs on the Arduino as a **non-blocking finite-state machine** (no `delay()` in the main loop). The machine progresses through five states: `WACHTEN_OP_INDEX`, `WACHTEN_OP_GESTREKT`, `NAAR_START_BEWEGEN`, `READY` and `THERAPIE_ACTIEF`. The hallmark feature is that the motor only contributes torque when the patient is actively trying to move — and the strength of that help is a live parameter set by the therapist.

#### 3.5.1 Assistive mode (velocity-triggered proportional assist)

The ODrive encoder velocity is read each control cycle and passed through a single-pole IIR low-pass filter (weight 0.80) to suppress noise. Three velocity regimes:

- **Forward intent** (v > 0.02 turns/s). The patient is actively initiating flexion. A proportional assistive torque is added:

  ```c
  assist = (v_filt - VELOCITY_THRESHOLD) * ASSIST_OPBOUW_FACTOR
  assist = clip(assist, 0, ASSIST_TORQUE)
  ```

  Both `ASSIST_OPBOUW_FACTOR` (ramp rate) and `ASSIST_TORQUE` (ceiling, default 0.60 Nm, max 4 Nm) are live GUI parameters.

- **Backward motion** (v < -0.02 turns/s). No assist — the motor must not push the arm back automatically — but a configurable viscous damping (`ASSIST_STREK_WEERSTAND × v`) supports eccentric activity on the way back.

- **Near rest** (|v| ≤ 0.02 turns/s). Small baseline damping (`BASIS_DAMPING × v`, default 0.05).

#### 3.5.2 Resistive mode (training)

For strength training the assist branch is disabled. Above the velocity threshold the heavy training damping is engaged:

```c
if |v_filt| > VELOCITY_THRESHOLD:
    damping = -TRAINING_WEERSTAND * v_filt   // default 2.50, range 0.50-4.00
else:
    damping = -BASIS_DAMPING    * v_filt   // 0.05
```

#### 3.5.3 Free-ride mode (internal — ROM measurement)

Both assist and damping are zero. The motor outputs no torque and the joint is fully back-drivable. The GUI engages this automatically when a ROM measurement is started.

#### 3.5.4 Safety: soft-limit virtual spring

A virtual spring is the **only** spring-like behaviour in the controller and is engaged exclusively when the joint angle crosses a soft limit:

```c
if angle < limit_min:
    spring = (limit_min - angle) * VIRTUELE_VEER_STIJFHEID
elif angle > limit_max:
    spring = (limit_max - angle) * VIRTUELE_VEER_STIJFHEID
else:
    // assist + damping computed by the active therapy mode
```

The default stiffness (0.10 Nm/°, range 0.05–0.50) is intentionally low: the soft limit is a guide reinforced by a simultaneous vibrotactile pulse (§3.7).

#### 3.5.5 Rate-limited torque update and hard safety guard

The commanded torque is rate-limited (|Δτ| ≤ 0.015 Nm per loop) and hard-clipped to ±`VEILIGHEIDS_KOPPEL_MAX` (default 1.50 Nm, max 4 Nm). A separate hard guard zeroes the torque whenever the measured velocity exceeds `MAX_SAFE_VELOCITY = 0.8 turns/s`, providing a software-level last line of defence on top of the hardware E-stop.

#### 3.5.6 Soft-start phase

The first impedance command applied at the moment of START used to produce an audible mechanical kick as the controller and the loosely-coupled human limb came into equilibrium. The firmware now introduces a soft-start phase the instant `THERAPIE_ACTIEF` is entered:

- **0 → 1.5 s:** only a transparent damping component (coefficient 0.25) is applied. No assist, no virtual spring.
- **1.5 → 3.0 s:** quadratic ease-in gain ramp from 0 to 1 applied to both assist and virtual-spring contributions. Damping remains full.
- **t > 3.0 s:** full therapy law.

#### 3.5.7 Vibration suppression (high-pass damping)

During demonstrations without a human arm in the brace, low-frequency oscillations of the unloaded mechanism appeared. The firmware applies a continuous high-pass damping term in addition to the standard control law:

```c
vel_hf            = fb.vel - v_filtered
vibration_damping = -HF_DAMPING_GAIN * vel_hf
```

The signal `(fb.vel - v_filtered)` is the high-frequency residual of the velocity feedback — large during oscillations, near zero during smooth voluntary motion. The gain (default 1.50 Nm·s/turn) is exposed as a **Schok-reductie** slider in the GUI.

### 3.6 Dual-IMU sensing

The two MPU-6050 IMUs serve two distinct and equally important functions. They are not redundant — each addresses a separate, well-documented problem in robotic upper-limb rehabilitation.

#### 3.6.1 Role 1 — Shoulder-compensation detection

This monitor is the most algorithmically sophisticated part of the system. The implementation went through several iterations before converging on a robust solution grounded in the clinical kinematic literature. The final algorithm is a **reach-based, ratio-modulated, zone-aware detector** — directly reflecting findings from Schwarz et al. (2020) [[15]](#references), Levin's RPSS [[16]](#references) and Cirstea & Levin (2000) [[17]](#references).

**Algorithm overview.** The detector operates as a per-reach evaluator rather than a continuous score. A *reach* is defined as any period during which the combined angular velocity (|ω_motor| + |ω_imu1|) exceeds **8 °/s for at least 0.2 s** (debounced start); the reach ends when the combined velocity falls below **4 °/s for 0.5 s** (debounced end). Within a reach, three quantities are tracked:

- `max_elbow_excursion`: maximum |Δmotor| from the reach-start angle.
- `max_shoulder_rise`: maximum upward elevation of the upper-arm IMU above the **session baseline** (the IMU1 angle captured when START MONITORING was pressed or after the most recent central tare), counted only while the upper arm is not net-descending.
- `net_motor_delta` and `net_imu1_delta`: signed change between the reach start and the current sample, used for direction-aware decisions.

**The score formula:**

```python
shoulder_score  = max(0, (max_shoulder_rise - 5°) * 100 / 40°)   # cap 100
elbow_required  = max(5°, max_shoulder_rise * zone_ratio)
effective_elbow = max(0, max_elbow_excursion - 3°)               # wobble
elbow_credit    = min(1, effective_elbow / elbow_required)
final_score     = shoulder_score * (1 - elbow_credit)
```

Three design choices distinguish this from naive ratio scoring:

**(a) Zone-based ratio (Levin RPSS [[16]](#references)).** The expected elbow/shoulder ratio depends on the functional workspace zone of the upper arm:

| Zone (imu1) | Ratio | Domain |
|---|---|---|
| **LOW** (< 30°) | 0.40 | Hand-to-mouth — shoulder should stay still |
| **MID** (30–90°) | 0.30 | Default reaching domain |
| **HIGH** (> 90°) | 0.50 | Above horizontal — elbow must take over |

**(b) Wobble subtraction.** The human elbow naturally exhibits 1–3° of incidental rotation during gross arm motion. To prevent this background wobble from counting as deliberate elbow use, the first **3° of elbow excursion is subtracted** before computing the credit.

**(c) Direction-aware finalisation (Cirstea & Levin 2000 [[17]](#references)).** When a reach ends with the upper-arm angle netto below its start angle (Δimu1 < -5°), the reach is treated as a return-to-rest motion and **not scored**. This avoids false alarms during the natural lowering phase that follows every outgoing reach.

**Live behaviour versus finalisation.** During an active reach the score updates in real time based on the current (instantaneous) effective_rise — the visible alarm subsides as soon as the patient lowers the shoulder, even mid-reach. At reach end, the finalisation step recomputes the score from the accumulated maxima and the direction test, and adds a coloured bar to the per-reach history chart. **Between reaches the level is forced back to GOED** so that a previous bad reach does not artificially keep the alarm panel red.

**Thresholds.** Base thresholds (warning at 25 %, alarm at 40 %) are exposed as a single **Gevoeligheid** slider that scales both proportionally:

| Score | Status | Visual |
|---|---|---|
| < warn | **GOED** | Green panel |
| warn–alarm | **WAARSCHUWING** | Amber panel |
| ≥ alarm | **COMPENSATIE** | Red panel with brief pulse; event counter increments on rising edge |

The session chart presents one coloured bar per completed reach (green/amber/red), with the score labelled above each bar. A CSV-export button writes the full sample log to disk for offline analysis.

#### 3.6.2 Role 2 — Full 3D arm-pose estimation

A single motor encoder only reports the relative angle between the two exoskeleton cuffs. It cannot distinguish an arm hanging vertically at 60° flexion from an arm held horizontally at the same 60°. Two IMUs fix this:

- The **upper-arm IMU** anchors the proximal segment in the global (gravity) frame. Its accelerometer-derived tilt reflects shoulder posture.
- The **forearm IMU + motor encoder** together anchor the distal segment.

Both angles use the **3D-vector gravity formulation** rather than the more common 2D `atan2(Y, Z)` form. The 2D form becomes numerically unstable near ±90° tilt; the 3D form is well-conditioned everywhere:

```c
angle = atan2( ay, sqrt(ax*ax + az*az) ) * 180 / PI
```

#### 3.6.3 Central tare — one calibration for every subsystem

To ensure that the 3D pose visualisation, the compensation monitor and the Mirror-Therapy overlay all use the same arm-orientation reference, a single pair of tare buttons is placed in the **main dashboard header** — always visible regardless of which tab is active:

- **↓ Tare verticaal** — captures the current pose as 0°/0° (arm hanging).
- **→ Tare horizontaal** — captures the current pose as 90°/0° (arm forward).

Pressing either button stores the per-IMU offsets and propagates them to every subsystem. If the compensation monitor is active when a tare is performed, its session baseline is also reset to the new orientation.

After taring, the dual-IMU pose stream feeds:

- A real-time 3D Matplotlib stick figure of the patient, with the active arm rendered shoulder → elbow → wrist.
- The Mirror-Therapy overlay (§3.8): a translucent mirrored arm on the contralateral side of the body, moving in lockstep with the affected arm.
- A UDP datagram stream (port 5005, format `"bovenarm_angle,motor_angle"`) to a Unity scene, allowing the patient to see their own arm inside an optional head-mounted display.

### 3.7 Vibrotactile feedback

The vibrotactile alert is delivered by the Drake LRA, driven through the DRV2605L in **PWM-input mode** (mode 3, LRA library 6). The DRV2605L receives its drive signal on its IN/TRIG pin from Arduino D13, which is hardware Timer4 / OCR4A. Using hardware PWM rather than software bit-banging guarantees the pulse timing is unaffected by the main control loop.

A naïve approach — apply a sustained PWM signal for the duration of a soft-limit overshoot — produces a mushy continuous buzz, because the LRA's internal mass keeps resonating. To produce a crisp end-stop click, the firmware uses a short fixed pulse with a 2.5 s cooldown:

```c
OCR4A = 255 * HAPTIC_KRACHT   // full drive, intensity from GUI slider
delay(50)
OCR4A = 0                     // hard cut
```

The intensity `HAPTIC_KRACHT` is exposed as a 0–1 slider in the GUI.

### 3.8 Python GUI

The therapist-facing interface is implemented in **CustomTkinter** with **Matplotlib** for live plotting. Source: `/software/gui_main.py`. The GUI parses a structured ASCII serial stream from the Arduino (`POS`, `VEL`, `TRQ`, `IMU1`, `IMU2`, `STATE`) and presents **nine tabs**:

| Tab | Description |
|---|---|
| **Project info** | Title, authors, summary. |
| **Instellingen** | Mode switch (Assistive / Resistive), ROM soft limits, assist parameters, resistive coefficient, safety torque ceiling, haptic intensity, and the **Schok-reductie** slider (vibration-suppression gain, §3.5.7). |
| **3D arm simulation** | 3D stick figure driven by dual-IMU pose, with an optional **Mirror-Therapy** toggle that renders a translucent mirrored arm on the contralateral side of the body — implementing the well-established mirror-therapy paradigm [[18]](#references), [[19]](#references). |
| **Positie controle** | Preset (0°, 50°, 100°) and slider-based commanded joint angles for setup. |
| **ROM test** | Switches to free-ride mode and records the patient's minimum/maximum reachable angle. |
| **Anti-compensatie** | The per-reach monitor of §3.6.1: status panel, live compensation score with progress bar, live **SCHOUDER ↑** and **ELLEBOOG Δ** mini-metrics, session statistics (Reaches Totaal, Goede Reaches, Compensaties) and the per-reach history bar chart. |
| **Game modus** | Two gamified modes: **Blokken Vangen** (Fitts-style hold) and **Ghost Arm (Ritme)** (rhythm task with adjustable tempo). |
| **Live grafieken** | Angle, velocity, acceleration and commanded torque versus time. |
| **ODrive Tuning** | Runtime PID adjustment (§3.4.6): `pos_gain`, `vel_gain`, `vel_integrator_gain`, `input_filter_bandwidth`. |

The **central tare buttons** (`↓ Tare verticaal` and `→ Tare horizontaal`) are placed in the always-visible header above the tab strip.

### 3.9 Notable troubleshooting and iterations

- **Breadboard → custom PCB.** Early prototyping used a breadboard, which suffered from intermittent contact issues and noise pickup on the I²C lines. Migrating to a self-designed PCB eliminated the loose-wire failures.
- **No more `delay()` in the firmware.** Early iterations used `delay()` for the haptic pulse, which froze the main loop for 50 ms and caused the ODrive to time out. The final firmware uses no `delay()` inside the therapy loop; the haptic pulse runs on Timer4 in the background.
- **Velocity-filter weight.** The first version of the assistive law used raw encoder velocity, which made the assist torque flicker unpleasantly. An IIR weight of 0.80 was the empirical sweet spot between latency and smoothness.
- **2D vs 3D IMU angle formula.** The first IMU implementation used `atan2(Y, Z)`, which collapsed catastrophically near 90° tilt. Switching to the 3D form `atan2(Y, √(X²+Z²))` made the angle robust across the entire patient working envelope [[12]](#references).
- **Soft-limit virtual-spring stiffness.** Initially set too high (0.30 Nm/°), the virtual spring felt like a hard wall and ejected the limb past the limit on the rebound. Reducing to 0.10 Nm/° and pairing it with the haptic pulse produced a far more natural boundary cue.
- **Index-pulse placement.** In one assembly attempt the encoder index ended up just outside the operational arc, which meant the ODrive could never finish its startup index search. Re-clocking the motor shaft so the index sits around 50° of flexion solved the issue.
- **Compensation algorithm — three generations.** The first implementation was a sliding-window max-min on raw IMU samples that triggered alarms continuously and required the user to "pump down" the score with repeated good motions. A second attempt used a geometric-coherence test (`imu2 ≈ imu1 + motor`), but the accelerometer-derived pitch is corrupted by tangential acceleration during dynamic motion, producing false alarms on clean elbow flexions. The final reach-based, ratio-modulated, zone-aware detector (§3.6.1), grounded in Schwarz et al. (2020) [[15]](#references), Levin's RPSS [[16]](#references) and Cirstea & Levin (2000) [[17]](#references), proved both more robust and more interpretable.
- **Soft-start at therapy activation.** Without the soft-start phase (§3.5.6), the first impedance command applied at the moment of START produced an audible mechanical kick. Splitting the activation into a 1.5 s damping-only phase followed by a quadratic gain ramp eliminated this.
- **Vibration suppression.** During demonstrations without a human arm in the brace, low-frequency oscillations of the unloaded mechanism appeared. A high-pass damping term added to the torque command (§3.5.7) provides automatic suppression that does not affect normal operation.
- **Continuous-motion smoothness metric.** A Spectral Arc Length (SPARC) display was briefly investigated as a candidate clinical smoothness marker. SPARC is well-validated for *discrete* reaches, but proved unreliable when applied to a continuous sliding window over mixed motion / rest data; the display was removed in favour of the per-reach compensation chart, which is also a smoothness proxy by construction.

---

## 4. Discussion

### 4.1 Addressing the medical challenge

The primary objective — providing low-impedance, back-drivable assistance that scales with the patient's own movement intent — was clearly demonstrated. Because the assist law triggers on filtered velocity rather than tracking a pre-recorded reference trajectory, the device behaves like a **cooperative partner**: it does nothing when the patient is at rest, gradually contributes torque when the patient initiates motion, and never overrides the patient's direction. This matches the assist-as-needed paradigm advocated by Marchal-Crespo and Reinkensmeyer [[3]](#references) and the broader meta-analysis of Mehrholz et al. [[4]](#references).

The **soft-start phase** (§3.5.6) and the **high-pass vibration-suppression term** (§3.5.7) added late in development make the device behave gracefully both at session start and during demonstrations with a loosely-coupled or absent arm. These two additions were not in the initial design but proved essential for comfortable, predictable behaviour outside ideal lab conditions.

### 4.2 Haptic and postural feedback

In informal testing, the active vibrotactile pulse at the soft-limit boundaries was reliably perceivable through the forearm cuff and through a thin shirt. Subjects identified boundary contact without ever looking at the GUI — exactly the off-loading of the visual channel reported by Bark et al. [[6]](#references). Combining the brief kinaesthetic push (virtual spring) with the discrete vibrotactile click produced a **bimodal boundary experience** that none of our subjects mistook for a system fault.

The dual-IMU compensation monitor reliably flagged shoulder-elevation events during deliberate "cheat" attempts (lifting the shoulder by 10° or more during attempted elbow flexion) and remained quiet during clean elbow movements with the upper arm held still. The graduated, ratio-based credit means that a small natural shoulder co-activation — for example a slight upward drift during a forward reach — is not penalised, while a stuck elbow paired with an actively moving shoulder is detected even when the per-reach delta is small. Direction-aware finalisation prevents alarms during the natural lowering phase. The single **Gevoeligheid** slider lets the therapist adapt the strictness to individual patients.

The Mirror-Therapy overlay in the 3D tab was perceived as a meaningful visual cue in informal demonstrations; a clinically meaningful evaluation would require structured testing with stroke survivors.

### 4.3 Constraints and observed limitations

- **IMU drift over long sessions.** The MPU-6050 lacks a magnetometer. Our implementation sidesteps this by using the gravity vector directly — robust against drift but insensitive to rotations about the vertical axis. A 9-DOF replacement (e.g. Bosch BNO085) with on-board sensor fusion would lift this constraint entirely.
- **Mechanical compliance.** Some 3D-printed adapter parts flex slightly under peak assistive torque. Critical load-bearing parts would benefit from CNC-machined aluminium.
- **ODrive index-search latency.** After power-up the ODrive needs 2–5 s of slow motion to locate the encoder index pulse. The Arduino state machine handles this gracefully but a brief delay before the first session is unavoidable.
- **Compensation thresholds derived from healthy data.** The zone-based ratios (0.40 / 0.30 / 0.50) and the wobble subtraction (3°) are derived from healthy-reaching kinematic studies [[15]](#references), not from a clinical population. For deployment, these values would need calibration against a cohort of stroke survivors.
- **Trunk-compensation not detected.** The current dual-IMU setup catches shoulder elevation but not forward trunk leaning, which is the second common compensation mode catalogued by Subramanian et al. [[20]](#references). A third IMU on the sternum is the natural extension.
- **Validation scope.** All testing was performed with healthy volunteers in a controlled lab environment. Clinical validation with stroke survivors is required before any deployment beyond engineering proof-of-concept.

### 4.4 Comparison with the originality benchmark

Within the assessment framework of this course, the system integrates a non-trivial set of subsystems that go well beyond the minimum-implementation example of "a Drake actuator on a sensor": FOC torque control with runtime PID tuning, dual-IMU posture sensing with two independent functional roles, hardware-PWM vibrotactile actuation, a real-time GUI with gamified modes, a Mirror-Therapy visualisation and a UDP bridge to Unity. The most novel sub-result is the **simultaneous use of the same IMU pair for both a literature-grounded compensation monitor** (Schwarz / Levin / Cirstea-Levin) **and 3D pose visualisation**, exploiting one sensor stream for two clinically distinct purposes.

---

## 5. Conclusion and future work

### 5.1 Summary

HapticElbow is a **low-cost open-source elbow exoskeleton** aimed at upper-limb neurorehabilitation. Its central technical contribution is a velocity-triggered proportional assist law (rather than a virtual spring-damper around a reference trajectory), implemented via current-mode FOC on the ODrive S1, combined with a dual-IMU sensing subsystem that simultaneously detects shoulder compensation and reconstructs the patient's 3D arm pose. The compensation detector is a per-reach evaluator with ratio, wobble and zone modulation, directly grounded in the published kinematic literature (Schwarz 2020 [[15]](#references), Levin's RPSS [[16]](#references), Cirstea & Levin 2000 [[17]](#references)). Two safety additions — a soft-start phase and a high-pass vibration-suppression term — make the device comfortable to use even before the arm is rigidly coupled to the brace. Vibrotactile soft-limit alerts via a Drake LRA, a non-blocking Arduino firmware and a Python GUI with gamified modes and a Mirror-Therapy overlay complete the system. The total component cost is approximately **€540** and every artefact required to rebuild the device is in this repository.

### 5.2 Future work

- **Trunk IMU for richer compensation detection.** Adding a third MPU-6050 on the sternum would extend the monitor to forward-leaning trunk compensation, the second common pattern after shoulder elevation [[20]](#references).
- **EMG-based intent detection.** Surface EMG over biceps and triceps would shift detection upstream, allowing the device to assist *before* motion begins. Reducing the cortex-to-actuator delay is expected to improve the neuroplastic benefit [[2]](#references).
- **9-DOF IMU upgrade.** The Bosch BNO085 has on-board AHRS sensor fusion and would eliminate yaw drift.
- **Clinical-grade mechanical revision.** Replacing the structural 3D-printed parts with CNC-machined aluminium would remove the residual compliance.
- **Wireless / tetherless operation.** Migrating Layer 1 from a desktop PC to a small SBC (Raspberry Pi 4/5) mounted on the harness would untether the patient, enabling bedside or home use.
- **Structured patient study.** A natural next step is a feasibility study with stroke survivors under ethical approval, measuring ROM recovery, compensation incidence and session adherence over multi-week protocols.

### 5.3 Lessons learned

- **Never use `delay()` inside a robot control loop** — a 10 ms block is enough to stall the ODrive serial channel.
- An LRA driven without DRV2605L LRA-mode and hardware PWM produces barely perceptible vibration. Get this right early.
- The AMT110 index location is a mechanical design constraint. Verify it falls inside the operational arc before final assembly.
- Use the 3D gravity-vector form of the IMU tilt angle from day one — the 2D form fails near 90° and the failure looks like a glitch, not a singularity.
- **Separate the therapy controller from the safety supervisor.** Hard guards (velocity ceiling, torque clip, rate-limit, vibration-suppression damping) belong in the safety layer and must run regardless of which therapy mode is active.
- **Ground rehabilitation algorithms in published clinical kinematics**, not in ad-hoc heuristics. The compensation detector required three redesigns before the literature-anchored approach (§3.6.1) produced stable, interpretable behaviour.
- Add a soft-start phase to any actively-driven physical-interaction robot. The patient's body is never as rigidly coupled as the bench test suggests.

---

## References

[1] World Health Organization, "The top 10 causes of death," Geneva, Switzerland, Dec. 2020. [Online]. Available: https://www.who.int/news-room/fact-sheets/detail/the-top-10-causes-of-death

[2] J. A. Kleim and T. A. Jones, "Principles of experience-dependent neural plasticity: Implications for rehabilitation after brain damage," *J. Speech Lang. Hear. Res.*, vol. 51, no. 1, pp. S225–S239, Feb. 2008, doi: [10.1044/1092-4388(2008/018)](https://doi.org/10.1044/1092-4388(2008/018)).

[3] L. Marchal-Crespo and D. J. Reinkensmeyer, "Review of control strategies for robotic movement training after neurologic injury," *J. NeuroEng. Rehabil.*, vol. 6, no. 1, p. 20, Jun. 2009, doi: [10.1186/1743-0003-6-20](https://doi.org/10.1186/1743-0003-6-20).

[4] J. Mehrholz, M. Pohl, T. Platz, J. Kugler, and B. Elsner, "Electromechanical and robot-assisted arm training for improving activities of daily living, arm function, and arm muscle strength after stroke," *Cochrane Database Syst. Rev.*, no. 9, Art. no. CD006876, 2018, doi: [10.1002/14651858.CD006876.pub5](https://doi.org/10.1002/14651858.CD006876.pub5).

[5] R. Sigrist, G. Rauter, R. Riener, and P. Wolf, "Augmented visual, auditory, haptic, and multimodal feedback in motor learning: A review," *Psychon. Bull. Rev.*, vol. 20, no. 1, pp. 21–53, Feb. 2013, doi: [10.3758/s13423-012-0333-8](https://doi.org/10.3758/s13423-012-0333-8).

[6] K. Bark, J. W. Wheeler, G. Lee, J. Redmond, and A. M. Okamura, "Comparison of skin stretch and vibrotactile stimulation for feedback of proprioceptive information," in *Proc. IEEE Symp. Haptic Interfaces for Virtual Environment and Teleoperator Syst.*, 2009, pp. 71–78, doi: [10.1109/HAPTIC.2009.4810812](https://doi.org/10.1109/HAPTIC.2009.4810812).

[7] M. F. Levin, J. A. Kleim, and S. L. Wolf, "What do motor recovery and compensation mean in patients following stroke?" *Neurorehabil. Neural Repair*, vol. 23, no. 4, pp. 313–319, May 2009, doi: [10.1177/1545968308328727](https://doi.org/10.1177/1545968308328727).

[8] A. Filippeschi, N. Schmitz, M. Miezal, G. Bleser, E. Ruffaldi, and D. Stricker, "Survey of motion tracking methods based on inertial sensors: A focus on upper limb human motion," *Sensors*, vol. 17, no. 6, p. 1257, Jun. 2017, doi: [10.3390/s17061257](https://doi.org/10.3390/s17061257).

[9] H. I. Krebs, N. Hogan, M. L. Aisen, and B. T. Volpe, "Robot-aided neurorehabilitation," *IEEE Trans. Rehabil. Eng.*, vol. 6, no. 1, pp. 75–87, Mar. 1998, doi: [10.1109/86.662623](https://doi.org/10.1109/86.662623).

[10] T. Nef, M. Mihelj, and R. Riener, "ARMin: A robot for patient-cooperative arm therapy," *Med. Biol. Eng. Comput.*, vol. 45, no. 9, pp. 887–900, Sep. 2007, doi: [10.1007/s11517-007-0226-6](https://doi.org/10.1007/s11517-007-0226-6).

[11] S. O. H. Madgwick, A. J. L. Harrison, and R. Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm," in *Proc. IEEE Int. Conf. Rehabil. Robot. (ICORR)*, 2011, pp. 1–7, doi: [10.1109/ICORR.2011.5975346](https://doi.org/10.1109/ICORR.2011.5975346).

[12] M. Pedley, "Tilt sensing using a three-axis accelerometer," Freescale Semiconductor Application Note AN3461, Rev. 6, Mar. 2013. [Online]. Available: https://www.nxp.com/docs/en/application-note/AN3461.pdf

[13] ODrive Robotics, "ODrive S1 datasheet," v0.6.x, 2024. [Online]. Available: https://docs.odriverobotics.com/v/latest/hardware/s1-datasheet.html

[14] Texas Instruments, "DRV2605L 2-to-5.2 V haptic driver for LRA and ERM with internal memory and library effects," datasheet SLOS854B, 2014 (revised 2016). [Online]. Available: https://www.ti.com/lit/ds/symlink/drv2605l.pdf

[15] A. Schwarz, J. M. Veerbeek, J. P. O. Held, J. C. Buurke, and A. R. Luft, "Measures of interjoint coordination post-stroke across different upper limb movement tasks," *Front. Bioeng. Biotechnol.*, vol. 8, art. 620805, Jan. 2021, doi: [10.3389/fbioe.2020.620805](https://doi.org/10.3389/fbioe.2020.620805).

[16] M. F. Levin, J. Desrosiers, D. Beauchemin, N. Bergeron, and A. Rochette, "Development and validation of a scale for rating motor compensations used for reaching in patients with hemiparesis: the Reaching Performance Scale," *Phys. Ther.*, vol. 84, no. 1, pp. 8–22, Jan. 2004, doi: [10.1093/ptj/84.1.8](https://doi.org/10.1093/ptj/84.1.8).

[17] M. C. Cirstea and M. F. Levin, "Compensatory strategies for reaching in stroke," *Brain*, vol. 123, no. 5, pp. 940–953, May 2000, doi: [10.1093/brain/123.5.940](https://doi.org/10.1093/brain/123.5.940).

[18] V. S. Ramachandran and D. Rogers-Ramachandran, "Synaesthesia in phantom limbs induced with mirrors," *Proc. R. Soc. B Biol. Sci.*, vol. 263, no. 1369, pp. 377–386, Apr. 1996, doi: [10.1098/rspb.1996.0058](https://doi.org/10.1098/rspb.1996.0058).

[19] H. Thieme, N. Morkisch, J. Mehrholz, M. Pohl, J. Behrens, B. Borgetto, and C. Dohle, "Mirror therapy for improving motor function after stroke," *Cochrane Database Syst. Rev.*, no. 7, Art. no. CD008449, 2018, doi: [10.1002/14651858.CD008449.pub3](https://doi.org/10.1002/14651858.CD008449.pub3).

[20] S. K. Subramanian, J. Yamanaka, G. Chilingaryan, and M. F. Levin, "Validity of movement pattern kinematics as measures of arm motor impairment poststroke," *Stroke*, vol. 41, no. 10, pp. 2303–2308, Oct. 2010, doi: [10.1161/STROKEAHA.110.593368](https://doi.org/10.1161/STROKEAHA.110.593368).

---

*This README documents the complete design and implementation of HapticElbow. All hardware files, firmware and software are released under the licence specified in this repository.*
