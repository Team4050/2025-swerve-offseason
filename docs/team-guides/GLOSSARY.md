# Swerve Drive & Robotics Glossary

A comprehensive reference for terminology used in our documentation and codebase.

---

## A

**Absolute Encoder**
An encoder that provides position information without needing a reference point. CANcoders are absolute encoders that know wheel angle even after power loss.

**Acceleration Feedforward (kA)**
A gain that compensates for the inertia of the mechanism, providing extra voltage during acceleration.

**Alliance**
Blue or Red team grouping in FRC matches. Affects autonomous strategy and field coordinate transformations.

**AprilTag**
Fiducial markers placed around the FRC field at known locations. Used by vision systems to determine robot position.

**Autonomous (Auto)**
The 15-second period at the start of each match where the robot operates without driver input.

**Azimuth**
Another term for the steer/rotation angle of a swerve module.

---

## B

**Brownout**
When battery voltage drops too low (typically <6.8V), causing the roboRIO to reset. Usually caused by excessive current draw.

**Bus Utilization**
Percentage of CAN bus bandwidth in use. High utilization (>80%) can cause communication issues.

---

## C

**CANcoder**
CTRE's absolute magnetic encoder, used for measuring swerve module wheel angles.

**CANivore**
CTRE's dedicated CAN FD interface that offloads CAN traffic from the roboRIO. Requires Phoenix Pro license.

**CAN Bus**
Controller Area Network - the communication system connecting motor controllers, sensors, and the roboRIO.

**CAN ID**
Unique identifier for each device on the CAN bus. Must be unique across all devices.

**Chassis Speeds**
Robot velocity represented as X velocity, Y velocity, and rotational velocity (vx, vy, omega).

**Closed-Loop Control**
Control system that uses feedback (sensors) to correct errors. Opposite of open-loop.

**Command**
WPILib's unit of robot action - contains code to execute, requirements, and end conditions.

**Coupling Ratio**
Mechanical relationship where steering a module affects drive wheel position. Phoenix automatically compensates for this.

---

## D

**Deadband**
Joystick input threshold below which inputs are ignored, preventing tiny unwanted movements.

**Derivative (kD)**
PID term that responds to rate of change of error. Reduces overshoot and oscillation.

**Differential Drive**
Traditional drive system with left and right wheels (tank drive). Different from swerve.

**Drivetrain Constants**
Robot-wide configuration like CAN bus name, Pigeon ID, and support for Pro features.

---

## E

**Encoder Offset**
Calibration value that sets the "zero" position of each swerve module's CANcoder.

---

## F

**Feedforward (FF)**
Control term that predicts required output based on desired setpoint, rather than reacting to error.

**Field-Centric**
Drive mode where robot motion is relative to the field (forward always downfield) rather than robot orientation.

**Field2d**
WPILib class for visualizing robot position on a 2D field representation.

**Firmware**
Software running on motor controllers and sensors. Should be updated to match Phoenix version.

**Fused CANcoder**
Phoenix Pro feature that combines CANcoder data with TalonFX internal sensor for lower latency. Falls back to Remote CANcoder without Pro.

---

## G

**Gear Ratio**
Mechanical reduction between motor and output. Higher ratio = more torque, less speed.

---

## H

**Heading**
Robot's rotation angle, typically measured from the field X-axis by the Pigeon 2.0.

**Holonomic**
Able to move in any direction while independently controlling rotation. Swerve is holonomic; differential drive is not.

**Hoot File**
Phoenix signal log file that can be replayed for simulation and debugging.

---

## I

**IMU (Inertial Measurement Unit)**
Sensor measuring rotation, acceleration, and orientation. Pigeon 2.0 is our IMU.

**Integral (kI)**
PID term that accumulates error over time. Eliminates steady-state error but can cause instability if too high.

**Inversion**
Reversing the direction of a motor or sensor. Critical for ensuring all modules work together correctly.

---

## K

**Kalman Filter**
Statistical algorithm that fuses multiple sensor measurements (odometry + vision) for optimal state estimation.

**kP, kI, kD**
PID controller gains: Proportional, Integral, Derivative.

**kS, kV, kA**
Feedforward gains: Static friction, Velocity, Acceleration.

---

## M

**Module**
A single swerve wheel assembly with drive motor, steer motor, and encoder.

**Module Constants**
Configuration specific to one module: motor IDs, encoder offset, position on robot.

**Motion Magic**
CTRE's trapezoidal motion profiling for smooth movement to position setpoints.

**Module State**
Desired or actual state of a swerve module: speed (m/s) and angle (Rotation2d).

---

## N

**Neutral Mode**
Motor behavior when not commanded: Brake (resists movement) or Coast (free spin).

**NetworkTables**
WPILib's pub-sub communication system for dashboard data and inter-process communication.

---

## O

**Odometry**
Calculating robot position by integrating wheel velocities and IMU heading over time.

**Open-Loop Control**
Control without feedback - directly setting motor voltage without checking result.

**Optimization (Module State)**
Preventing swerve modules from rotating >90° by reversing wheel speed instead.

---

## P

**PathPlanner**
Popular FRC tool for visually creating and following robot trajectories.

**Phoenix**
CTRE's motor controller software ecosystem (Phoenix 5, Phoenix 6).

**Phoenix Pro**
Licensed features including CANivore support, fused sensors, and advanced control modes.

**Phoenix Tuner X**
CTRE's configuration and diagnostic tool for Phoenix 6 devices.

**PID Controller**
Proportional-Integral-Derivative controller - standard feedback control algorithm.

**Pigeon 2.0**
CTRE's IMU sensor providing heading, pitch, roll, and acceleration data.

**Pose2d**
Robot position and orientation on field: X, Y coordinates and heading angle.

**Proportional (kP)**
PID term proportional to current error. Primary correction force.

---

## R

**Robot-Centric**
Drive mode where forward/back/left/right are relative to robot orientation, not field.

**RobotContainer**
Class that holds subsystems and binds commands to controller buttons.

**roboRIO**
National Instruments controller that runs robot code.

---

## S

**Signal**
Data value from a Phoenix device (position, velocity, current, etc.).

**SignalLogger**
Phoenix 6's built-in logging system for recording all device signals.

**Simulation**
Running robot code on development computer with physics simulation instead of real hardware.

**Slip Current**
Current threshold at which wheels begin to slip. Used to prevent loss of traction.

**Stator Current**
Current through motor windings. Limited to prevent motor damage and brownouts.

**Status Signal**
Phoenix 6's API for efficiently reading device data with latency compensation.

**Subsystem**
WPILib component representing a robot mechanism with state and control methods.

**Supply Current**
Current from battery to motor controller. Different from stator current due to controller efficiency.

**Swerve Drive**
Holonomic drive system with independently steerable and driven wheels.

**SwerveRequest**
Phoenix 6 API for commanding swerve drivetrain behavior (field-centric, brake, etc.).

**SysId**
WPILib tool for characterizing robot mechanisms and calculating optimal control gains.

---

## T

**Talon FX**
CTRE's integrated motor controller for Falcon 500 and Kraken motors.

**Telemetry**
Data sent from robot to driver station for monitoring and debugging.

**Teleop (Teleoperated)**
Match period where drivers control robot (follows autonomous, 2:15 duration).

**Trajectory**
Time-parameterized path defining robot position, velocity, and acceleration over time.

**Translation**
Linear movement (X/Y) without rotation.

**TunerConstants**
Phoenix-generated file containing all swerve configuration (gains, IDs, dimensions).

---

## V

**Vendor Library**
Third-party software library (CTRE Phoenix, REVLib, PathPlanner) added to robot project.

**Vision Measurement**
Robot pose estimate from camera/AprilTag detection, fused with odometry.

---

## W

**Waypoint**
Intermediate point along a path that the robot passes through.

**Wheel Base**
Front-to-back distance between wheels.

**Track Width**
Left-to-right distance between wheels.

**WPILib**
Official FRC software library providing robot framework, math utilities, and control algorithms.

---

## Acronyms

**API** - Application Programming Interface
**CAN** - Controller Area Network
**CCW** - Counter-Clockwise
**CTRE** - Cross The Road Electronics (Phoenix manufacturer)
**CW** - Clockwise
**FD** - Flexible Datarate (high-speed CAN variant)
**FF** - Feedforward
**FOC** - Field-Oriented Control
**FRC** - FIRST Robotics Competition
**GUI** - Graphical User Interface
**IMU** - Inertial Measurement Unit
**PID** - Proportional-Integral-Derivative
**PWM** - Pulse Width Modulation
**SysId** - System Identification
**USB** - Universal Serial Bus

---

## Units (Phoenix 6)

Phoenix 6 uses WPILib's Units library for type safety:

**Distance:**

- `Inches.of(2)` → wheel radius
- `Meters.of(0.5)` → robot dimensions

**Velocity:**

- `MetersPerSecond.of(4.58)` → max speed
- `RotationsPerSecond.of(10)` → motor speed

**Current:**

- `Amps.of(120)` → current limits

**Voltage:**

- `Volts.of(12)` → voltage compensation

**Angle:**

- `Rotations.of(0.25)` → encoder offset
- `Degrees.of(90)` → heading target

---

## Common Abbreviations in Code

```java
kP, kI, kD          // PID gains
kS, kV, kA          // Feedforward gains
FL, FR, BL, BR      // Front-Left, Front-Right, Back-Left, Back-Right
vx, vy, omega       // X velocity, Y velocity, rotational velocity
```
