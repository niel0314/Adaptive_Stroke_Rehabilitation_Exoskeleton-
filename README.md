# Adaptive Stroke Rehabilitation Exoskeleton
**Authors:** Senne Peeters and Niel Boon 
## Introduction

Neurological injuries, most prominently strokes, are a leading cause of long-term motor disability worldwide. The World Health Organization estimates that 15 million people suffer a stroke each year, with approximately one third left with permanent impairment [1]. A large fraction of survivors lose proximal control of the upper limb, which stops them from doing essential activities of daily living such as reaching, lifting, and self-care. Recovery is driven by neuroplasticity, where the central nervous system reorganizes and forms new synaptic connections in response to repetitive, task-specific, voluntary practice [2]. 

While electromechanical and robot-assisted arm training significantly improves arm function after a stroke, this benefit is conditional on the patient remaining an active agent in the movement [3], [4]. Purely passive limb manipulation produces only marginal cortical reorganization. This active-participation requirement is the foundation of the assist-as-needed (AAN) paradigm now widely advocated in rehabilitation robotics [3].

### Why a Haptic Interface
Motor learning is driven by sensorimotor feedback. Augmented haptic and multimodal feedback during motor training improves learning retention compared with visual-only protocols [5]. Two complementary haptic modalities are relevant for a rehabilitation device:
* **Kinaesthetic feedback:** Torques and forces transmitted through the mechanical linkage. In our system, this is realized by a back-drivable brushless motor driven in current (torque) mode, delivering smooth, low-impedance assistance without locking the joint.
* **Vibrotactile feedback:** High-frequency skin vibrations that convey discrete events. Vibrotactile cues at the forearm during elbow tasks reduce path errors and accelerate motor learning by offloading information from the visual channel [6].

### The Gap Addressed
A well-documented issue in upper-limb rehabilitation is the compensatory movement strategy. When elbow flexors are too weak, patients reflexively recruit proximal muscles, elevating the shoulder or rotating the trunk [7]. Neurologically, this is counter-productive: the self defeating pattern is rehearsed instead of the target movement, and any robotic system that simply assists the joint torque without monitoring posture will reinforce it. 

Wearable inertial measurement units (IMUs) have been validated as effective low-cost tools for tracking limb-segment orientation [8]. Mounting one IMU on the upper arm and one on the forearm enables real-time discrimination between true elbow flexion and proximal compensation. We use this insight as a core design principle: the prototype not only assists movement but also flags when the assistance is being recruited by the wrong muscle group.

Established robotic platforms (such as MIT-Manus [8], ARMin [9], and the Armeo family) demonstrate robust clinical efficacy but are prohibitively expensive (€50,000 to €150,000), tied to dedicated clinical infrastructure, and largely closed-source. Open-source alternatives like the EduExo kit lower the entry cost dramatically but typically rely on rigid stepper-motor or gear-train actuation that is neither back-drivable nor torque-controlled. 

Our contribution targets the intersection that none of the above covers simultaneously: precise, back-drivable torque control, integrated vibrotactile limit alerts, real-time compensation monitoring, and a gamified patient-facing interface. All hardware files, firmware, and software are released openly so the device can be replicated for around €540.

---

## Supplies (Bill of Materials)

### Actuation and Motor Control
| Component | Description | Source | Cost |
| :--- | :--- | :--- | :--- |
| **ODrive S1** | Single-axis FOC controller, 12-50 V input, 2 kW continuous. | ODrive Europe | ~€155 |
| **BLDC motor (D5312s 330KV)** | High-pole-count outrunner; smooth low-speed torque, fully back-drivable. | ODrive Robotics | ~€85 |
| **AMT110 incremental encoder** | Configurable 48-2048 CPR, dedicated index pulse for ODrive homing. | Digi-Key | ~€30 |
| **Mean Well RSP-320-24 PSU** | 24 V / 13.4 A / 320 W enclosed switching PSU with active PFC. | Farnell | ~€65 |
| **230 V AC safety switch** | Pre-wired with integrated E-stop mushroom button. | Amazon DE | ~€25 |

### Microcontroller and Communication
| Component | Description | Source | Cost |
| :--- | :--- | :--- | :--- |
| **Arduino Micro (ATmega32U4)** | Controller of the whole set-up. | Arduino Official | ~€25 |
| **Custom PCB** | Carries the Arduino, DRV2605L, ODrive UART terminals, and headers. | Self-fabricated | ~€25 |
| **Micro-USB & Jumpers** | Data cable for PC link; M-M/M-F jumpers for off-board modules. | Any | ~€11 |

### Sensors and Haptic Feedback
| Component | Description | Source | Cost |
| :--- | :--- | :--- | :--- |
| **MPU-6050 IMU ×2** | 6-DOF (3-axis accelerometer + 3-axis gyroscope) on I²C. | Adafruit #3886 | €18 |
| **DRV2605L haptic driver** | I²C-controlled LRA/ERM driver (PWM input mode used). | Adafruit #2305 | ~€9 |
| **Drake LRA haptic actuator** | Linear Resonant Actuator with 15 ms rise/fall time. | Drake / TacHammer | ~€20 |

### Mechanical Structure
| Component | Description | Source | Cost |
| :--- | :--- | :--- | :--- |
| **Custom exoskeleton frame** | Printed in PETG; motor axis coaxial with the anatomical elbow joint. | Self-designed | ~€60 |
| **Bearings, Fasteners, Velcro** | Deep-groove ball bearing, M3/M4 hardware, 50 mm Velcro. | Local store | ~€12 |

---

## Methods

### System Architecture
The system is organized into a three-layer distributed control stack:
* **Layer 1 (Python GUI, PC):** Handles therapy-mode selection, ROM and assist parameters, real-time graphs, and a UDP stream to Unity. It communicates with Layer 2 over USB serial.
* **Layer 2 (Arduino Micro):** Runs a non-blocking finite-state machine, two-sensor I²C polling, vibrotactile trigger logic, and relays computed torque set-points to the ODrive.
* **Layer 3 (ODrive S1):** Handles closed-loop field-oriented current control at ~8 kHz with encoder feedback. If Arduino communication stalls, the ODrive holds its last commanded torque safely.

### Actuator and Microcontroller Logic
We selected an ODrive S1 and a BLDC motor over a geared servo because conventional geared motors are stiff and not back-drivable, which poses a safety risk in physical human–robot interaction. The ODrive controls motor current (torque), allowing the joint to behave as an active element without gearbox backlash. 

The therapy controller runs on the Arduino as a non-blocking finite-state machine. It utilizes three primary modes:
* **Assistive Mode:** The system interprets positive filtered velocity above a threshold as active intent, applying a proportional assistive torque that ramps up with patient effort.
* **Resistive Mode:** For strength training, the assist branch is disabled and heavy training damping is engaged.
* **Safety Soft-Limits:** A virtual spring engages exclusively when the joint angle crosses a soft limit. Once a limit is crossed, the motor stops helping or resisting and pushes the limb back into the safe arc.

### Dual-IMU Posture Sensing
Mounting one IMU on the upper arm and one on the forearm enables real-time discrimination between true elbow flexion and proximal compensation.
* **Compensation Detection:** The upper-arm IMU tracks the orientation of the proximal segment relative to gravity. A continuous monitor computes range and drift; if the score exceeds 18°, a compensation event is logged.
* **3D Arm-Pose Estimation:** The upper-arm IMU anchors the proximal segment globally, while the forearm IMU combined with the motor encoder anchors the distal segment. This reconstructs the patient's exact 3D arm pose for visualizations.

### Vibrotactile Feedback
The vibrotactile alert is delivered by the Drake LRA, driven through the DRV2605L in PWM-input mode. ERM motors have 50 to 100 ms mechanical rise and fall times, making them unsuitable for discrete "click" events, whereas the LRA provides a smaller then 15 ms response. Using hardware PWM rather than software bit-banging guarantees the pulse timing is unaffected by the main control loop.

### Notable Troubleshooting and Iterations
* **Breadboard to Custom PCB:** Early prototyping on a breadboard suffered from intermittent contacts and noise on I²C lines. Migrating to a custom PCB eliminated loose-wire failures while preserving modularity.
* **Non-blocking Firmware:** Early iterations used `delay()` for haptic pulses, which froze the main loop for 50 ms and caused ODrive timeouts. The final firmware avoids `delay()` entirely, utilizing Timer4 in the background.
* **IMU Formula Upgrade:** The first IMU implementation used a 2D approach, which collapsed catastrophically near 90° tilt. Switching to the 3D form made calculations robust across the patient's entire working envelope.

---

## Discussion

The primary objective of providing low-impedance, back-drivable assistance that scales with the patient's movement intent was successfully demonstrated. Because the assist law triggers on filtered velocity rather than tracking a pre-recorded reference trajectory, the device behaves like a cooperative partner, aligning with the assist-as-needed paradigm.

During informal testing, subjects identified boundary contact from the vibrotactile pulse without needing to look at the GUI, successfully off-loading the visual channel. Combining the brief kinaesthetic push with a discrete vibrotactile click produced a bimodal boundary experience that users recognized as intentional rather than a system fault. The dual-IMU compensation monitor also robustly flagged deliberate "cheat" attempts while remaining quiet during clean elbow movements.

**Constraints and Observed Limitations:**
* **IMU Drift:** The MPU-6050 lacks a magnetometer. Using the gravity vector sidesteps drift but makes the system insensitive to rotations about the vertical axis.
* **Mechanical Compliance:** Some 3D-printed parts flex slightly under peak torque, softening the perceived stiffness of the soft-limit spring.
* **ODrive Search Latency:** After power-up, the ODrive requires 2 to 5s of slow motion to locate the encoder index pulse, introducing a brief but unavoidable delay.

---

## Conclusion and Future Work

HapticElbow is a low-cost (~€540) open-source elbow exoskeleton aimed at upper-limb neurorehabilitation. Its central technical contribution is a velocity-triggered proportional assist law implemented via current-mode FOC on the ODrive S1, combined with dual-IMU sensing that simultaneously detects shoulder compensation and reconstructs 3D arm pose. 

**Directions for Future Development:**
* **EMG-Based Intent Detection:** Adding surface EMG would shift detection upstream, allowing the device to assist before physical motion begins, potentially improving neuroplastic benefits.
* **9-DOF IMU Upgrade:** The Bosch BNO085 features on-board AHRS sensor fusion, which would eliminate yaw drift and enable quaternion-based tracking.
* **Clinical-Grade Mechanical Revision:** Replacing 3D-printed load-bearing parts with CNC-machined aluminum would remove residual compliance.
* **Wireless Operation:** Migrating the PC software to a Raspberry Pi mounted on the harness would untether the patient for bedside or home use.
* **Clinical Studies:** Conducting feasibility studies with stroke survivors under ethical approval.

**Lessons Learned for Follow-On Work:**
* Never use `delay()` inside a robot control loop, a 10 ms block is enough to stall the ODrive serial channel.
* Use the 3D gravity-vector form of the IMU tilt angle immediately; the 2D form fails near 90°.
* Separate the therapy controller from the safety supervisor so hard guards run regardless of which therapy mode is active.

---

## References

[1]. World Health Organization, "The top 10 causes of death," Geneva, Switzerland, Dec. 2020.

[2]. J. A. Kleim and T. A. Jones, "Principles of experience-dependent neural plasticity: Implications for rehabilitation after brain damage," J. Speech Lang. Hear. Res., vol. 51, no. 1, pp. S225–S239, Feb. 2008.

[3]. L. Marchal-Crespo and D. J. Reinkensmeyer, "Review of control strategies for robotic movement training after neurologic injury," J. NeuroEng. Rehabil., vol. 6, no. 1, p. 20, Jun. 2009.

[4]. J. Mehrholz, M. Pohl, T. Platz, J. Kugler, and B. Elsner, "Electromechanical and robot-assisted arm training for improving activities of daily living, arm function, and arm muscle strength after stroke," Cochrane Database Syst. Rev., no. 9, Art. no. CD006876, 2018.

[5]. R. Sigrist, G. Rauter, R. Riener, and P. Wolf, "Augmented visual, auditory, haptic, and multimodal feedback in motor learning: A review," Psychon. Bull. Rev., vol. 20, no. 1, pp. 21–53, Feb. 2013.

[6]. K. Bark, J. W. Wheeler, G. Lee, J. Redmond, and A. M. Okamura, "Comparison of skin stretch and vibrotactile stimulation for feedback of proprioceptive information," in Proc. IEEE Symp. Haptic Interfaces for Virtual Environment and Teleoperator Syst., 2009, pp. 71–78.

[7]. M. F. Levin, J. A. Kleim, and S. L. Wolf, "What do motor recovery and compensation mean in patients following stroke?" Neurorehabil. Neural Repair, vol. 23, no. 4, pp. 313–319, May 2009.

[8]. H. I. Krebs, N. Hogan, M. L. Aisen, and B. T. Volpe, "Robot-aided neurorehabilitation," IEEE Trans. Rehabil. Eng., vol. 6, no. 1, pp. 75–87, Mar. 1998.

[9]. T. Nef, M. Mihelj, and R. Riener, "ARMin: A robot for patient-cooperative arm therapy," Med. Biol. Eng. Comput., vol. 45, no. 9, pp. 887–900, Sep. 2007.

[10]. S. O. H. Madgwick, A. J. L. Harrison, and R. Vaidyanathan, "Estimation of IMU and MARG orientation using a gradient descent algorithm," in Proc. IEEE Int. Conf. Rehabil. Robot. (ICORR), 2011, pp. 1–7.

[11]. M. Pedley, "Tilt sensing using a three-axis accelerometer," Freescale Semiconductor Application Note AN3461, Rev. 6, Mar. 2013.

[12]. ODrive Robotics, "ODrive S1 datasheet," v0.6.x, 2024.

[13]. Texas Instruments, "DRV2605L 2-to-5.2 V haptic driver for LRA and ERM with internal memory and library effects," datasheet SLOS854B, 2014 (revised 2016).t SLOS854B, 2014 (revised 2016). : 355]
