# Swerve Drive Quick Reference Card

## 🚨 Emergency Troubleshooting

### Robot Won't Move

1. Check Driver Station: Green lights for "Communications" and "Robot Code"?
2. Battery voltage >12V?
3. All devices visible in Tuner X?
4. Emergency stop pressed?
5. Default command set in RobotContainer?

### Wheels Oscillate/Vibrate

1. Reduce steer kP by 25% in TunerConstants.java
2. Increase steer kD by 50%
3. Redeploy and test

### Robot Spins Uncontrollably

1. Reset heading: `drivetrain.seedFieldCentric()`
2. Check field-centric mode is enabled
3. Verify Pigeon 2.0 working in Tuner X

### One Module Not Working

1. Swap CAN IDs with working module in TunerConstants.java
2. If problem follows module → hardware issue
3. If problem stays → configuration issue

---

## 🎮 Default Control Scheme

**Left Stick:** Strafe (X/Y translation)
**Right Stick:** Rotation
**A Button:** X-brake mode
**B Button:** Reset field-centric heading

---

## 🔧 Key Configuration Files

### TunerConstants.java

Location: `src/main/java/frc/robot/generated/`

**Steer Gains:**

```java
.withKP(100)    // Position control
.withKI(0)      // Usually 0
.withKD(0.5)    // Damping
.withKS(0.1)    // Static friction
.withKV(2.66)   // Velocity feedforward
```

**Drive Gains:**

```java
.withKP(0.1)    // Velocity error correction
.withKI(0)      // Usually 0
.withKD(0)      // Usually 0
.withKS(0)      // Static friction
.withKV(0.124)  // Velocity feedforward
```

**Other Settings:**

- `kSlipCurrent`: Current limit (default: 120A)
- `kSpeedAt12Volts`: Max speed (default: 4.58 m/s)
- `kPigeonId`: Pigeon 2.0 CAN ID (default: 2)

---

## 📊 Tuning Quick Guide

### Symptoms → Solutions

| Symptom              | Problem           | Solution                   |
| -------------------- | ----------------- | -------------------------- |
| Wheels vibrate       | Steer kP too high | Decrease kP by 25%         |
| Slow wheel rotation  | Steer kP too low  | Increase kP by 50%         |
| Wheels overshoot     | Need damping      | Add/increase kD            |
| Velocity lags        | Drive kP too low  | Increase kP by 25%         |
| Velocity oscillates  | Drive kP too high | Decrease kP by 25%         |
| Robot drifts         | Odometry drift    | Add vision pose estimation |
| Not driving straight | Wrong wheel size  | Verify `kWheelRadius`      |

---

## 🎯 Validation Checklist

Before competition:

```
□ All wheels point forward when enabled
□ Robot drives straight (no curve)
□ Spins in place smoothly
□ No oscillation during figure-8
□ Heading holds during translation
□ No brownouts under full acceleration
□ Battery >12.5V
□ All devices show in Tuner X (no faults)
```

---

## 💻 Common Commands

### Build & Deploy

```bash
./gradlew build
./gradlew deploy
```

### Simulation

```bash
./gradlew simulateJava
```

### View Logs

```
In Driver Station: View -> Log Viewer
Extract from robot: Tuner X -> Tools -> Log Extractor
```

---

## 🔍 Diagnostic Tools

### Phoenix Tuner X

- **Device List:** See all CAN devices, check faults
- **Control Tab:** Manually test motors
- **Plots:** Graph signals in real-time
- **Configs:** View/edit device configurations

### SmartDashboard / Shuffleboard

- View Field2d for pose visualization
- Monitor odometry frequency
- Check command execution

### AdvantageScope

- Replay matches
- Detailed signal analysis
- Trajectory visualization

---

## 📈 Typical Gain Values

### Well-Tuned Robot (Reference)

**Steer:**

- kP: 80-150
- kI: 0
- kD: 0.5-2.0
- kS: 0.05-0.2 V
- kV: 1.5-3.0 V/(rot/s)

**Drive:**

- kP: 0.05-0.3
- kI: 0
- kD: 0
- kS: 0-0.3 V
- kV: 0.1-0.15 V/(m/s)

**Heading:**

- kP: 5-12
- kI: 0
- kD: 0.2-0.8

---

## 🎬 SysId Procedure

1. Deploy code with SysId routines
2. Run characterization commands:
   - Translation: Quasistatic Forward/Reverse, Dynamic Forward/Reverse
   - Steer: Quasistatic Forward/Reverse, Dynamic Forward/Reverse
   - Rotation: Quasistatic Forward/Reverse, Dynamic Forward/Reverse
3. Extract logs (Tuner X -> Log Extractor)
4. Open in WPILib SysId tool
5. Analyze and apply gains

---

## 🏁 Autonomous Quick Start

### PathPlanner Setup

1. Install PathPlanner (VS Code vendor library)
2. Open PathPlanner GUI
3. Set robot dimensions and max speeds
4. Create path
5. Load in code: `AutoBuilder.followPath(PathPlannerPath.fromPathFile("PathName"))`

### Reset Odometry

```java
@Override
public void autonomousInit() {
    drivetrain.resetPose(path.getPreviewStartingHolonomicPose());
    autoCommand.schedule();
}
```

---

## 🔑 Key Files Reference

| File                           | Purpose             | Modify?              |
| ------------------------------ | ------------------- | -------------------- |
| `TunerConstants.java`          | Gains, IDs, configs | ✓ Yes (carefully)    |
| `CommandSwerveDrivetrain.java` | Main subsystem      | ✓ Yes (add features) |
| `RobotContainer.java`          | Button bindings     | ✓ Yes                |
| `Constants.java`               | Team constants      | ✓ Yes                |
| SwerveDrivetrain base          | Phoenix core logic  | ✗ No (library)       |
