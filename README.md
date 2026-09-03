### ASU RISE Lab FURI UAV Flight Analysis

**Researcher:** Matthew Herdegen
**Institution:** Arizona State University
**Lab:** RISE Lab
**Program:** FURI
**Focus:** UAV Flight Stability, Rotor Force Estimation, and Near-Surface Aerodynamics

---

## Overview

This project uses **ROS 2**, **MAVROS**, and a simulated multirotor UAV to execute waypoint-based flight missions and analyze vehicle performance.

The goal of the project is to develop a simulation and analysis framework that can quantify UAV behavior during:

* Free-flight hover
* Ground-effect operation
* Near-wall flight
* Ground and wall interaction
* Low-altitude translation
* Takeoff and landing

The system is divided into two main components:

1. **ROS 2 / MAVROS Flight Control**

   * Connects to the simulated UAV
   * Arms the vehicle
   * Sets the flight mode
   * Sends position commands
   * Executes waypoint-based missions

2. **MATLAB Flight Analysis**

   * Processes recorded flight data
   * Converts rotor RPM into estimated thrust
   * Evaluates vehicle stability
   * Measures waypoint tracking accuracy
   * Estimates rotor-induced moments
   * Compares different flight conditions

---

## Repository Structure

```text
rise-furi-uav/
│
├── ros2/
│   └── offboard_control.py
│
├── matlab/
│   └── rise_flight_analysis.m
│
├── data/
│   └── flight_data.csv
│
├── rise_analysis_results/
│   ├── flight_summary.csv
│   ├── window_metrics.csv
│   ├── phase_metrics.csv
│   └── generated plots
│
└── README.md
```

---

## Flight Mission

The current mission is designed around a simple waypoint sequence:

```python
waypoints = [
    [0, 0, 2],
    [2, 2, 2],
    [0, 0, 2],
    [0, 0, 0.1]
]
```

The intended flight sequence is:

```text
Takeoff
   ↓
Climb to 2 m
   ↓
Fly to [2, 2, 2]
   ↓
Return to [0, 0, 2]
   ↓
Descend
   ↓
Land
```

Position setpoints are intended to be published at approximately **10 Hz**.

---

# ROS 2 / MAVROS Flight Control

The ROS 2 node communicates with the simulated vehicle through MAVROS.

The controller is responsible for:

* Monitoring MAVROS connection status
* Monitoring vehicle armed state
* Monitoring vehicle flight mode
* Reading vehicle position
* Sending position setpoints
* Arming the UAV
* Changing the flight mode
* Executing the waypoint mission

The current node uses:

```python
self.timer = self.create_timer(0.1, self.send_setpoint)
```

which corresponds to a setpoint publication frequency of:

```text
10 Hz
```

---

## Current Flight-Control Improvements

The initial flight-control script requires several corrections and additions before it can be used as a complete autonomous waypoint controller.

### Position Subscriber

Incorrect:

```python
self.pos_sub = self.create_subscription(
    PoseStamped. "mavros/local_position/pose",
    self.pos_cb,
    10
)
```

Correct:

```python
self.pos_sub = self.create_subscription(
    PoseStamped,
    "mavros/local_position/pose",
    self.pos_cb,
    10
)
```

---

### Position Callback

The position callback should store the current UAV pose:

```python
def pos_cb(self, msg):
    self.current_pose = msg
```

This allows the controller to determine the distance between the aircraft and the active waypoint.

---

### ROS 2 Spin

Incorrect:

```python
rclpy.spin(node)
```

Correct:

```python
rclpy.spin(takeoff_landing_node)
```

---

### Waypoint Advancement

The waypoint list must be paired with a waypoint index.

For example:

```python
self.current_waypoint = 0
```

The position error can then be calculated using:

```python
distance = math.sqrt(
    (x_cmd - x)**2 +
    (y_cmd - y)**2 +
    (z_cmd - z)**2
)
```

When the UAV reaches a selected tolerance, the controller can advance to the next waypoint.

Example:

```python
if distance < 0.15:
    self.current_waypoint += 1
```

---

## MAVROS Setpoint Topic

The message type used by the publisher must match the selected MAVROS topic.

For example, a standard position command commonly uses a `PoseStamped` message with a position setpoint interface.

The correct MAVROS topic and vehicle mode should be verified for the specific:

* ArduPilot configuration
* MAVROS version
* Gazebo simulation
* Vehicle model

used by the project.

---

# MATLAB Flight Analysis

The MATLAB analysis program is:

```text
rise_flight_analysis.m
```

Run the analysis using:

```matlab
results = rise_flight_analysis("data/flight_data.csv");
```

The program analyzes:

* Motor RPM
* Rotor angular velocity
* Individual rotor thrust
* Total rotor thrust
* Vertical thrust
* Net vertical force
* Thrust-to-weight ratio
* Estimated vertical acceleration
* Rotor RPM imbalance
* Roll stability
* Pitch stability
* Yaw behavior
* Angular-rate stability
* Position tracking
* Estimated rotor moments
* Flight phases
* Oscillation frequencies
* User-selected analysis windows

---

# Rotor Force Estimation

Rotor thrust is estimated using:

```text
T_i = kT * ω_i²
```

where:

* `T_i` = thrust from rotor `i`
* `kT` = motor/propeller thrust coefficient
* `ω_i` = rotor angular velocity

Rotor angular velocity is calculated from RPM using:

```text
ω = RPM * 2π / 60
```

Therefore:

```text
T_i = kT * (RPM_i * 2π / 60)²
```

---

## Total Thrust

For a quadrotor:

```text
T_total = T_1 + T_2 + T_3 + T_4
```

---

## Vertical Thrust

During roll and pitch maneuvers, part of the total thrust acts horizontally.

The approximate vertical thrust is:

```text
T_vertical = T_total * cos(roll) * cos(pitch)
```

---

## Vehicle Weight

Vehicle weight is calculated as:

```text
W = m * g
```

where:

```text
g = 9.80665 m/s²
```

---

## Net Vertical Force

The estimated net vertical force is:

```text
Fz_net = T_vertical - m*g
```

Interpretation:

| Result       | Meaning                          |
| ------------ | -------------------------------- |
| `Fz_net > 0` | Net upward force                 |
| `Fz_net ≈ 0` | Approximate vertical equilibrium |
| `Fz_net < 0` | Net downward force               |

---

## Estimated Vertical Acceleration

Using Newton's Second Law:

```text
F = m*a
```

the acceleration estimate is:

```text
az_est = Fz_net / m
```

---

# Thrust-to-Weight Ratio

The thrust-to-weight ratio is calculated using:

```text
TWR = T_total / (m*g)
```

A value near:

```text
TWR = 1
```

indicates that total rotor thrust is approximately equal to vehicle weight.

---

# Rotor Thrust Calibration

The thrust coefficient `kT` must be calibrated for the specific motor and propeller combination.

The MATLAB configuration contains:

```matlab
cfg.kT_N_per_rad_s2 = 1.20e-5;
```

This value is only a placeholder.

If measured thrust and RPM are known:

```text
kT = T_measured / ω²
```

where:

```text
ω = RPM * 2π / 60
```

For better accuracy, multiple RPM and thrust measurements should be collected and fitted to:

```text
T = kT * ω²
```

The RPM-based force calculations should not be treated as quantitative experimental measurements until `kT` has been properly calibrated.

---

# Recommended Flight Data

At minimum, the analysis requires:

```text
time_s
rpm1
rpm2
rpm3
rpm4
```

For full flight analysis, the recommended data set is:

```text
time_s

x
y
z

vx
vy
vz

roll
pitch
yaw

p
q
r

rpm1
rpm2
rpm3
rpm4

x_cmd
y_cmd
z_cmd
```

If an external force or wrench estimate is available, also record:

```text
Fx_ext
Fy_ext
Fz_ext
```

---

## Example CSV Header

```csv
time_s,x,y,z,vx,vy,vz,roll,pitch,yaw,p,q,r,rpm1,rpm2,rpm3,rpm4,x_cmd,y_cmd,z_cmd,Fx_ext,Fy_ext,Fz_ext
```

Example data:

```csv
12.40,1.20,0.95,2.03,0.31,0.28,0.01,1.2,-0.8,43.1,0.02,-0.03,0.01,5120,5090,5150,5110,2.0,2.0,2.0,0.0,0.0,0.2
```

---

# Quaternion Support

If attitude is recorded using quaternions instead of Euler angles, the MATLAB code can use:

```text
qx
qy
qz
qw
```

These are automatically converted to:

```text
roll
pitch
yaw
```

for stability analysis.

---

# MATLAB Configuration

Before running the analysis, update the configuration section.

Example:

```matlab
cfg.mass_kg = 2.00;

cfg.kT_N_per_rad_s2 = 1.20e-5;

cfg.armLength_m = 0.23;
```

These values should match the simulated UAV.

---

# Flight Stability Analysis

The MATLAB program evaluates several indicators of vehicle stability.

## Roll RMS

Roll RMS measures the magnitude of lateral attitude motion:

```text
Roll RMS = sqrt(mean(roll²))
```

A lower value during steady hover generally indicates better lateral stability.

---

## Pitch RMS

Pitch RMS measures longitudinal attitude motion:

```text
Pitch RMS = sqrt(mean(pitch²))
```

---

## Attitude Standard Deviation

Roll and pitch standard deviation measure attitude variation about the mean.

These metrics can help identify:

* Controller oscillation
* Aerodynamic disturbance
* Rotor imbalance
* Ground-effect disturbances
* Wall-effect disturbances

---

# Angular-Rate Stability

If body angular rates are available:

```text
p = roll rate
q = pitch rate
r = yaw rate
```

the MATLAB analysis calculates RMS angular rates.

High RMS rates during hover may indicate:

* Aggressive controller corrections
* Oscillation
* Aerodynamic disturbances
* Rotor imbalance
* Near-surface aerodynamic effects

---

# Frequency-Domain Stability Analysis

The MATLAB program uses a Fast Fourier Transform to identify dominant roll and pitch oscillation frequencies.

Example output:

```text
Roll dominant frequency  = 1.7 Hz
Pitch dominant frequency = 2.1 Hz
```

Periodic behavior may originate from:

* Flight-controller response
* Structural vibration
* Rotor interaction
* Ground-effect flow
* Wall-induced aerodynamic disturbances

---

# Motor RPM Imbalance

Rotor imbalance is calculated using:

```text
RPM Imbalance =
(max RPM - min RPM) / mean RPM * 100
```

A high RPM imbalance means the flight controller is commanding significantly different rotor speeds.

Possible causes include:

* Vehicle center-of-gravity offset
* Aerodynamic disturbances
* Rotor mismatch
* Vehicle asymmetry
* Ground effect
* Wall effect
* Controller compensation

---

# Rotor Moment Estimation

For a quadrotor, individual rotor thrust values can be used to estimate vehicle moments.

The default MATLAB model assumes:

```text
Motor 1 = Front Left
Motor 2 = Front Right
Motor 3 = Rear Right
Motor 4 = Rear Left
```

The body coordinate system is assumed to be:

```text
+x = Forward
+y = Right
+z = Up
```

Rotor moments are calculated using:

```text
M = r × F
```

The analysis estimates:

* Roll moment
* Pitch moment
* Yaw moment

---

# Yaw Torque

Rotor reaction torque can be approximated using:

```text
Q_i = kQ * ω_i²
```

where `kQ` is the rotor drag or torque coefficient.

Rotor spin direction must also be defined.

Example:

```matlab
cfg.rotorYawDirection = [1 -1 1 -1];
```

where the signs represent opposite rotor rotation directions.

---

# Position Tracking

If commanded position is recorded, the three-dimensional tracking error is:

```text
Position Error =
sqrt(
    (x - x_cmd)² +
    (y - y_cmd)² +
    (z - z_cmd)²
)
```

The MATLAB program calculates:

* Position RMSE
* Maximum position error

These measurements quantify how effectively the UAV maintains the commanded trajectory.

---

# Automatic Flight-Phase Detection

When position and velocity data are available, MATLAB attempts to classify each portion of the flight as:

* Ground
* Climb
* Hover
* Transit
* Descent

This allows stability metrics to be compared between different portions of the mission.

---

# Time-Window Analysis

Specific flight periods can also be selected manually.

Example:

```matlab
cfg.windows_s = [
     5  12
    20  27
    35  42
];

cfg.windowNames = [
    "Free_Flight"
    "Ground_Effect"
    "Wall_Effect"
];
```

The same stability and force metrics are then calculated independently for each condition.

---

# Ground-Effect Analysis

Ground effect occurs when rotor wake interacts with a nearby horizontal surface.

A potential effect is reduced rotor power or RPM required to maintain hover.

A useful experiment is to maintain constant:

* Vehicle mass
* Controller gains
* Motor model
* Propeller model
* Desired position
* Simulation settings

while changing only the UAV's height above the ground.

Useful comparison variables include:

* Mean rotor RPM
* Total estimated thrust
* Thrust-to-weight ratio
* Altitude error
* Roll RMS
* Pitch RMS
* RPM imbalance
* Position RMSE
* Oscillation frequency

A normalized RPM comparison can be calculated using:

```text
RPM Reduction (%) =
(RPM_free - RPM_ground) / RPM_free * 100
```

---

# Wall-Effect Analysis

Operation near a vertical wall can create asymmetric rotor inflow.

This may cause different rotor loading on the wall-side and opposite-side motors.

Useful variables include:

* Wall-side motor RPM
* Opposite-side motor RPM
* Roll angle
* Roll rate
* Estimated roll moment
* Lateral position error
* Position RMSE

Motor groups can be compared as:

```text
Wall-Side Motors
        vs.
Opposite-Side Motors
```

This provides a measure of additional control effort caused by wall proximity.

---

# External Wrench Analysis

If the simulation provides an independent vehicle force estimate, it can be compared with the RPM-based model.

The residual is:

```text
F_residual = F_measured - F_RPM_model
```

The residual may include:

* Ground effect
* Wall effect
* Aerodynamic disturbance
* Propulsion-model error
* Unmodeled vehicle dynamics
* Simulation noise

This comparison may help separate rotor-generated forces from near-surface aerodynamic forces.

---

# Recommended Stability Metrics

| Metric                         | Purpose                           |
| ------------------------------ | --------------------------------- |
| Mean Motor RPM                 | Rotor operating condition         |
| Mean Total Thrust              | Estimated rotor force             |
| Thrust-to-Weight Ratio         | Vertical force margin             |
| RPM Imbalance                  | Asymmetric motor demand           |
| Roll RMS                       | Lateral attitude stability        |
| Pitch RMS                      | Longitudinal attitude stability   |
| Roll Rate RMS                  | Lateral dynamic activity          |
| Pitch Rate RMS                 | Longitudinal dynamic activity     |
| Position RMSE                  | Station-keeping performance       |
| Altitude Variation             | Vertical stability                |
| Dominant Oscillation Frequency | Periodic instability              |
| Roll Moment                    | Lateral control effort            |
| Pitch Moment                   | Longitudinal control effort       |
| External Force Residual        | Estimated aerodynamic disturbance |

---

# Suggested Test Matrix

Potential simulation conditions include:

| Test                        | Purpose                       |
| --------------------------- | ----------------------------- |
| Free Hover                  | Baseline                      |
| Ground-Effect Hover         | Ground proximity              |
| Near-Wall Hover             | Wall proximity                |
| Ground + Wall               | Combined surface interaction  |
| Corner Hover                | Multiple surface interaction  |
| Wall Approach               | Transient wall effect         |
| Wall Departure              | Recovery behavior             |
| Low-Altitude Forward Flight | Ground-effect translation     |
| Low-Altitude Lateral Flight | Lateral near-surface behavior |

Whenever possible, maintain constant:

* Vehicle mass
* Motor model
* Propeller model
* Controller gains
* Simulation timestep
* Sampling frequency
* Initial conditions
* Test duration

Only the environmental parameter being studied should be changed.

---

# Nondimensional Surface Distance

Surface effects may be compared using rotor dimensions.

For ground effect:

```text
z/R
```

or:

```text
z/D
```

where:

* `z` = rotor height above the ground
* `R` = rotor radius
* `D` = rotor diameter

For wall effect:

```text
d/R
```

or:

```text
d/D
```

where:

* `d` = distance from the UAV or rotor to the wall

Nondimensional distance makes results easier to compare between different UAV and rotor sizes.

---

# Generated MATLAB Outputs

The MATLAB script creates:

```text
rise_analysis_results/
```

Typical output files include:

```text
flight_summary.csv
window_metrics.csv
phase_metrics.csv
force_comparison.csv

trajectory_3d.png
rpm_and_thrust.png
attitude_stability.png
force_margin_and_motor_balance.png
position_error.png
estimated_moments.png
roll_spectrum.png
pitch_spectrum.png
```

---

# Flight Summary Metrics

The generated flight summary may include:

* Flight duration
* Sampling frequency
* Mean total thrust
* Total thrust standard deviation
* Mean thrust-to-weight ratio
* Minimum thrust-to-weight ratio
* Maximum thrust-to-weight ratio
* Mean net vertical force
* Mean RPM imbalance
* Maximum RPM imbalance
* Roll RMS
* Pitch RMS
* Peak roll angle
* Peak pitch angle
* Dominant roll frequency
* Dominant pitch frequency
* Position RMSE
* Maximum position error

---

# Example Research Questions

The framework can be used to investigate questions such as:

* Does ground effect reduce the RPM required for hover?
* How does wall distance affect rotor loading?
* Does wall proximity increase roll instability?
* How does ground distance affect altitude stability?
* Does near-surface operation increase flight-controller effort?
* Are there characteristic oscillation frequencies near surfaces?
* How does position-hold accuracy change with surface distance?
* Does one side of the vehicle require increased rotor thrust near a wall?
* Can external aerodynamic force be identified from the difference between measured wrench and RPM-based thrust?

---

# Current Limitations

The rotor model:

```text
T = kT * ω²
```

is a simplified aerodynamic relationship.

It does not independently account for:

* Ground effect
* Wall effect
* Rotor-to-rotor interaction
* Changing inflow conditions
* Air-density variation
* Motor transients
* Battery-voltage variation
* Propeller deformation
* Structural vibration
* Sensor noise
* RPM measurement error

Therefore, RPM-derived thrust should initially be treated as a baseline estimate.

The model should ideally be validated against:

* Gazebo rotor-force data
* Simulation wrench measurements
* Propulsion-model parameters
* Experimental thrust-stand measurements

---

# Future Development

Potential future improvements include:

* [ ] Automatic ROS 2 data logging
* [ ] ROS bag to CSV conversion
* [ ] Direct MATLAB ROS bag analysis
* [ ] Automatic waypoint detection
* [ ] Automatic takeoff detection
* [ ] Automatic landing detection
* [ ] Automatic hover-window detection
* [ ] Ground-distance logging
* [ ] Wall-distance logging
* [ ] Ground-effect coefficient estimation
* [ ] Wall-effect coefficient estimation
* [ ] Motor-pair asymmetry analysis
* [ ] External wrench estimation
* [ ] Rotor thrust coefficient identification
* [ ] Controller response analysis
* [ ] Settling-time calculations
* [ ] Overshoot calculations
* [ ] Disturbance-rejection testing
* [ ] Monte Carlo simulation
* [ ] Automated comparison between flights
* [ ] Automated FURI plots and tables
* [ ] Experimental UAV validation

---

# Long-Term Research Goal

The long-term goal of this project is to quantify how near-surface aerodynamic environments influence multirotor UAV performance.

The research combines:

* ROS 2
* MAVROS
* ArduPilot
* UAV simulation
* Rotor RPM measurements
* Position measurements
* Vehicle attitude
* Angular rates
* Rotor-force estimation
* External wrench estimation
* MATLAB data analysis

to evaluate:

* Rotor loading
* Flight stability
* Control effort
* Position-hold accuracy
* Aerodynamic disturbances
* Ground effect
* Wall effect
* Near-surface flight behavior

The resulting framework can be used to characterize UAV performance under different near-surface operating conditions and support future experimental validation.

---

## Author

**Matthew Herdegen**
Arizona State University
RISE Lab
FURI Research Project
