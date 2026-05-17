# AeroTHON 2026 Flight Dynamics & Simulation Suite 🚁

**Mission:** Rapid Payload Delivery & Surveillance (Mission 2)

This repository contains the complete MATLAB mathematical modeling, control systems validation, and mission simulation suite for the SkyScan Uncrewed Aircraft System (UAS). These scripts validate aerodynamic stability, propulsion selection, and autonomous flight algorithms prior to hardware-in-the-loop (HITL) testing.

---

## 🚁 UAS Specifications

These parameters are mathematically derived from CAD models and propulsion datasheets, and are dynamically loaded into the simulations.

| Parameter | Value | Notes |
| :--- | :--- | :--- |
| **Frame Type** | X-Quadrotor | Custom 200mm arm length (center-to-motor) |
| **Takeoff Weight (AUW)** | 1.8 kg | Includes 100g payload (200g margin to 2kg limit) |
| **Propulsion** | T-Motor AIR2216 KV880 | Paired with T1045 (10x4.5) Propellers |
| **Battery** | 4S 3300mAh LiPo | 14.8V Nominal |
| **Thrust-to-Weight** | 2.87 : 1 | High authority for rapid maneuvers |
| **Hover Throttle** | ~68% | 450g thrust per motor required |
| **Hover Endurance** | ~11.7 minutes | Derived from power consumption models |

---

## 📊 Simulation Suite

This suite addresses the AeroTHON 2026 Rulebook requirements for **Computational Analysis**, **Stability Analysis**, **Safety**, and **Innovation** entirely in MATLAB.

### 1. Stability & Dynamics
* **`sim1_roll.m` (Roll Hover Stability):** Validates tuning of the PID controller. Proves the drone settles within 1.5s with zero steady-state error.
* **`sim2_4axis.m` (Full 4-Axis Stability):** Simulates simultaneous step responses for Roll, Pitch, Yaw, and Altitude.
* **`sim8_bode.m` (Frequency Response):** Proves robustness against vibrations with sufficient phase and gain margins.

### 2. Autonomous Flight Algorithms
* **`sim3_trajectory_3d.m` (Mission 2 3D Trajectory):** Maps the exact rulebook waypoints (takeoff, 5m QR scan, 3m corridor navigation, 10m delivery zone identification, and 5m payload drop) into a time-based reference path. Generates a 6-panel tracking error plot and a 3D animated flight video.

### 3. Payload Mechanics
* **`sim9_cg_shift.m` (Payload Drop CG Shift):** Models the upward Center of Gravity shift at the exact moment the 100g payload is released, proving flight stability is maintained mid-mission.

### 4. Aircraft Performance & Safety
* **`sim4_battery.m` (Battery & Endurance):** Integrates power consumption over a 15-minute simulated flight profile, tracking State of Charge (SOC) to ensure mission feasibility before the 20% RTL threshold.
* **`sim5_wind.m` (Wind Disturbance Rejection):** Models a 5 m/s wind gust hitting the drone during hover to prove the flight controller can recover.
* **`sim6_motor_failure.m` (Motor Failure Analysis):** Simulates a sudden loss of Motor 1 and maps the required compensation RPM from the remaining three motors.
* **`sim10_thrust_curve.m` (Propulsion Margins):** Validates that the T-Motor AIR2216 operates efficiently at the 1.8kg AUW hover point.

### 5. Innovation
* **`sim7_pid_vs_lqr.m` (PID vs. LQR Control):** A comparative analysis showing standard PID response vs. an optimal Linear Quadratic Regulator (LQR) derived from a state-space model.

---

## 🛠️ How to Run the Simulations

1. Clone this repository to your local machine.
2. Open MATLAB and navigate to the `src` folder.
3. **CRITICAL:** You must run the parameters file first to load the drone's physics into the workspace:
   ```matlab
   drone_params
