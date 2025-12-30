# Swerve Drive Troubleshooting Guide

## Table of Contents

1. [Diagnostic Approach](#diagnostic-approach)
2. [Common Issues by Symptom](#common-issues-by-symptom)
3. [Systematic Debugging Process](#systematic-debugging-process)
4. [Tools and Techniques](#tools-and-techniques)
5. [Code-Specific Issues](#code-specific-issues)
6. [Prevention Strategies](#prevention-strategies)

---

## Diagnostic Approach

### The Troubleshooting Mindset

When debugging swerve drive issues, follow this hierarchy:

```
1. OBSERVE: What exactly is happening? (Not what you think might be wrong)
2. ISOLATE: Can you reproduce it? Under what conditions?
3. HYPOTHESIZE: What could cause this behavior?
4. TEST: Change ONE thing, verify result
5. DOCUMENT: Record what you tried and what happened
```

### Information Gathering Checklist

Before diving into fixes, gather this information:

```
□ Exact symptom description (video is ideal)
□ When does it occur? (Always, sometimes, specific conditions?)
□ Recent changes to code or hardware
□ Any error messages or console output
□ Driver Station log (if applicable)
□ Phoenix Tuner X device list status
□ Battery voltage at time of issue
□ Whether issue occurs in simulation
```

---

## Common Issues by Symptom

### Category 1: Robot Won't Move At All

#### Symptom: Enable robot, no motor movement

```
Checklist:
□ Driver Station shows "Communications" and "Robot Code" green?
□ Battery voltage >12V?
□ All motor controllers visible in Tuner X?
□ Any red fault indicators in Tuner X?
□ Command getting to drivetrain? (Add print statement)
□ Emergency stop pressed? (physical or software)
```

**Most Common Causes:**

##### 1. Motor Controllers Not Configured

```java
// Check: Do you see this in console during startup?
// "Applying configuration to device X..."

// If missing, verify in RobotContainer:
private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
// ^ This MUST be called to initialize motors
```

**Fix:** Ensure drivetrain is constructed in `RobotContainer` constructor, not lazily.

##### 2. No Command Scheduled

```java
// Check if default command is set:
public RobotContainer() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(() -> drive.with...)
    );
}

// Debug: Add to drivetrain.periodic():
System.out.println("Current command: " + getCurrentCommand());
```

**Fix:** Verify default command is set and joystick inputs are being read.

##### 3. Motor Controllers in Wrong Mode

```java
// Check neutral mode:
driveInitialConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
steerInitialConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
```

**Fix:** Ensure neutral mode is set to `Brake`, not `Coast` (for testing).

#### Symptom: Some motors move, others don't

```
Checklist:
□ Check CAN IDs - any duplicates?
□ Verify each motor in Tuner X individually
□ Look for fault codes on non-working motors
□ Swap non-working motor with working one (hardware vs. config issue?)
```

**Most Common Causes:**

##### 1. CAN ID Conflicts

```java
// Check TunerConstants.java for duplicates:
private static final int kFrontLeftDriveMotorId = 1;
private static final int kFrontLeftSteerMotorId = 0;  // Duplicate?
```

**Fix:** Ensure all 13 devices have unique IDs (4 drive + 4 steer + 4 CANcoder + 1 Pigeon2).

##### 2. Inverted Motor Outputs

```java
// Check motor inversions:
driveInitialConfigs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
// or
.Inverted = InvertedValue.CounterClockwise_Positive;
```

**Fix:** Test each motor individually in Tuner X control panel, adjust inversion if needed.

---

### Category 2: Erratic or Jittery Movement

#### Symptom: Wheels oscillate/vibrate rapidly

This is the #1 most common swerve issue - **over-tuned steer PID gains**.

```
Checklist:
□ Can you hear a buzzing/hunting sound from modules?
□ Do wheels twitch even when robot is stationary?
□ Does it get worse at higher speeds?
□ Is oscillation frequency > 5 Hz?
```

**Diagnosis:**

```bash
# In Tuner X:
1. Plot "Steer Motor Position" vs "Steer Target Position"
2. Look for high-frequency oscillation around target
3. Check "Steer Motor Voltage" - should be smooth, not spiky
```

**Fix:**

```java
// In TunerConstants.java:
// 1. Reduce kP by 25-50%
.withKP(100)  // Try 50-75 instead

// 2. Increase kD
.withKD(0.5)  // Try 1.0-2.0

// 3. Reduce kV if still oscillating
.withKV(2.66)  // Try 2.0
```

Refer to **Tuning Guide** for detailed PID tuning process.

#### Symptom: Robot "jumps" or "twitches" when changing direction

```
Checklist:
□ Does it happen when reversing drive direction?
□ Does it happen when rotating?
□ Are all modules doing it, or just some?
```

**Most Common Causes:**

##### 1. Module Optimization Disabled

Phoenix should automatically prevent >90° rotations, but check:

```java
// Module state optimization is automatic in Phoenix 6
// If not working, check for manual state application bypassing API

// DON'T do this (bypasses optimization):
module.getDriveMotor().setControl(driveRequest.withVelocity(speed));
module.getSteerMotor().setControl(steerRequest.withPosition(angle));

// DO use SwerveRequest API instead:
setControl(new SwerveRequest.FieldCentric().with...);
```

##### 2. Encoder Offset Issues

```java
// Check encoder offsets are correct:
// 1. Enable robot
// 2. All wheels should point same direction
// 3. If one is backwards, offset is ~180° off

// Fix in TunerConstants.java:
private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.XXX);
// Adjust by ±0.5 if wheel is backwards
```

#### Symptom: Robot spins uncontrollably

```
Checklist:
□ Is robot in field-centric mode?
□ Has Pigeon 2.0 been reset recently?
□ Are you testing on carpet vs. smooth floor?
□ Does it happen immediately or after moving?
```

**Most Common Causes:**

##### 1. Field-Centric Not Seeded

```java
// If robot heading is unknown, field-centric will be wrong
// Fix: Seed or reset at startup

@Override
public void autonomousInit() {
    drivetrain.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
    // Or:
    drivetrain.seedFieldCentric();  // Uses current heading as 0°
}
```

##### 2. Heading Controller Unstable

```java
// If using FieldCentricFacingAngle with bad gains
// Check: Plot heading error over time

// Fix: Re-tune heading PID (start with kP = 5.0, kD = 0.2)
// See "Phase 4" in Tuning Guide
```

##### 3. Gyro Drift

```java
// Pigeon 2.0 accumulating drift during match
// Check: Does heading change when robot is stationary?

// Short-term fix: Reset heading between matches
// Long-term fix: Mount Pigeon 2.0 more rigidly, check for vibration
```

---

### Category 3: Inaccurate Movement

#### Symptom: Robot doesn't drive straight

```
Checklist:
□ Which direction does it curve? (Consistent?)
□ Same curvature at all speeds?
□ Does it happen in robot-centric mode too?
□ Are all wheels the same diameter/wear?
```

**Most Common Causes:**

##### 1. Incorrect Wheel Radius

```java
// If wheels are worn or wrong size:
private static final Distance kWheelRadius = Inches.of(2);
// Measure actual wheel diameter and update

// Symptom: Robot moves faster/slower than commanded
// Test: Command 1 m/s, measure actual speed
// If actual < commanded: increase radius
// If actual > commanded: decrease radius
```

##### 2. Drive Motor Feedforward (kV) Mismatch

```java
// If different modules have different kV (due to wear, friction):
// All modules share same gains, so use average

// Check in Tuner X: Plot velocity error for each module
// If one consistently lags, may need mechanical inspection
```

##### 3. Module Inversion Issues

```java
// Check TunerConstants.java:
private static final boolean kInvertLeftSide = false;
private static final boolean kInvertRightSide = true;

// Test: Command pure forward (no rotation)
// Wheels should all spin same direction
// If not, adjust inversions
```

#### Symptom: Rotation rate is wrong (too fast/slow)

```
Checklist:
□ Does robot rotate faster or slower than joystick input suggests?
□ Is MaxAngularRate set correctly?
□ Check module positions (X/Y coordinates)
```

**Most Common Causes:**

##### 1. Incorrect Module Positions

```java
// Module positions affect rotation radius
// Check TunerConstants.java:
private static final Distance kFrontLeftXPos = Inches.of(10.5);
private static final Distance kFrontLeftYPos = Inches.of(10.5);

// Measure actual distances from robot center to module centers
// Common mistake: Using module corner instead of wheel center
```

##### 2. MaxAngularRate Calculation

```java
// In RobotContainer or Constants:
private static final double MaxAngularRate = Math.PI * 1.5; // rad/s

// If too fast: decrease multiplier (1.5 → 1.0)
// If too slow: increase multiplier (1.5 → 2.0)
```

#### Symptom: Autonomous paths don't follow correctly

```
Checklist:
□ Does path preview look correct in PathPlanner?
□ Is odometry accurate? (Check Field2d visualization)
□ Are units correct? (meters vs. inches)
□ Is robot starting pose initialized correctly?
```

**Most Common Causes:**

##### 1. Odometry Not Reset at Start

```java
@Override
public void autonomousInit() {
    // CRITICAL: Reset odometry to path starting pose
    drivetrain.resetPose(autoPath.getStartingPose());
}
```

##### 2. PathPlanner Configuration Mismatch

```java
// Check AutoBuilder configuration:
AutoBuilder.configure(
    drivetrain::getPose,              // Must match odometry
    drivetrain::resetPose,            // Must actually reset
    drivetrain::getChassisSpeeds,     // Must return current speeds
    (speeds, feedforwards) -> { ... },
    new PPHolonomicDriveController(...), // PID gains here
    new RobotConfig(...),             // Max speeds must match robot
    () -> shouldFlipPath(),           // Alliance flipping
    drivetrain
);
```

##### 3. Units Inconsistency

```java
// Common mistake: Mixing meters and inches
// Phoenix uses meters for distance, radians for angles
// Ensure all conversions are correct:

Distance wheelRadius = Inches.of(2);  // ✓ Correct
LinearVelocity maxSpeed = MetersPerSecond.of(4.5);  // ✓ Correct

// Don't mix units manually without conversion
```

---

### Category 4: Performance Issues

#### Symptom: CAN bus errors or timeouts

```
Checklist:
□ Error messages in Driver Station log?
□ Yellow/red indicators in Tuner X?
□ Using CANivore or roboRIO CAN?
□ How many devices on bus?
□ CAN termination resistors present?
```

**Most Common Causes:**

##### 1. CAN Bus Overload

```java
// Too many signals at high update rates
// Check StatusSignal update frequencies:

// In TunerConstants - don't reduce these without reason:
driveMotor.getVelocity().setUpdateFrequency(100);  // Hz
steerMotor.getPosition().setUpdateFrequency(100);

// But can reduce non-critical signals:
driveMotor.getSupplyVoltage().setUpdateFrequency(4);  // Only need occasionally
```

**Fix:** Use CANivore for swerve devices (Pro license required).

##### 2. Bad CAN Wiring

```
Physical check:
- CANH (yellow) and CANL (green) not swapped
- Termination resistors at both ends of bus (120Ω each)
- No loose connectors
- No damaged/pinched wires
```

##### 3. Device Faults

```bash
# In Tuner X:
1. Check device list for red fault indicators
2. Click faulted device
3. View "Faults" tab
4. Common faults:
   - Hardware fault: Device damage
   - Bootup fault: Configuration issue
   - Sensor fault: Encoder disconnected
```

#### Symptom: Loop overruns / "Robot loop took too long"

```
Checklist:
□ Is simulation mode slower than real robot?
□ Are you logging excessive data?
□ Any blocking calls in periodic()?
□ Large autonomous paths loading?
```

**Most Common Causes:**

##### 1. Synchronous CAN Calls in Periodic

```java
// DON'T do this in periodic:
double velocity = driveMotor.getVelocity().getValue();  // Blocks!

// DO use cached signals:
StatusSignal<Double> velocitySignal = driveMotor.getVelocity();
// Later, non-blocking:
double velocity = velocitySignal.getValueAsDouble();

// Or use Phoenix's built-in caching:
// SwerveDrivetrain handles this automatically
```

##### 2. Excessive Logging

```java
// Logging every signal every loop = slow
// Fix: Reduce logging frequency

// Instead of:
for (int i = 0; i < 1000; i++) {
    SmartDashboard.putNumber("Value" + i, values[i]);  // Slow!
}

// Do:
if (loopCount % 10 == 0) {  // Only every 10 loops
    SmartDashboard.putNumber("ImportantValue", value);
}
```

##### 3. Path Loading in Periodic

```java
// DON'T load paths in periodic:
public void periodic() {
    PathPlannerPath path = PathPlanner.loadPath("AutoPath");  // Blocks!
}

// DO load in constructor or init:
public RobotContainer() {
    autoPath = PathPlanner.loadPath("AutoPath");  // Load once
}
```

---

### Category 5: Simulation vs. Real Robot Differences

#### Symptom: Works in simulation, fails on real robot

```
Checklist:
□ Are you using correct CAN bus name?
□ Is simulation using default parameters?
□ Are sensors reading correctly on real robot?
□ Battery voltage difference?
```

**Most Common Causes:**

##### 1. CAN Bus Configuration

```java
// Simulation often uses hoot replay:
public static final CANBus kCANBus = new CANBus("", "./logs/example.hoot");

// Real robot needs actual bus name:
// public static final CANBus kCANBus = new CANBus("");  // roboRIO
// public static final CANBus kCANBus = new CANBus("canivore_name");
```

##### 2. Sensor Availability

```java
// Simulation doesn't require real hardware
// Real robot: Check all devices in Tuner X

// Add validation in code:
@Override
public void robotInit() {
    if (isReal()) {
        // Verify critical devices
        if (!drivetrain.verifyDevices()) {
            System.err.println("ERROR: Swerve devices not found!");
        }
    }
}
```

##### 3. Physics Differences

```java
// Simulation uses ideal physics:
private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);

// Real robot has more friction, mass variation
// Gains tuned in sim may need adjustment on real robot
// Always re-tune on actual hardware
```

#### Symptom: Works on real robot, crashes in simulation

```
Checklist:
□ Are you calling sim-only methods?
□ Is simulation loop running? (m_simNotifier)
□ Are simulation parameters set?
```

**Fix:**

```java
// Guard real-robot-only code:
if (isReal()) {
    // Real robot code
}

// Guard simulation-only code:
@Override
public void simulationPeriodic() {
    // Simulation update
    updateSimState(kSimLoopPeriod, RobotController.getBatteryVoltage());
}
```

---

## Systematic Debugging Process

### Process 1: Isolate the Problem Module/Motor

When a specific module acts differently:

```bash
Step 1: Test in Tuner X Control Panel
  1. Open Tuner X
  2. Select suspected motor controller
  3. Go to "Control" tab
  4. Manually command voltage (start low, ±1V)
  5. Verify motor spins expected direction
  6. Try different speeds, check for stuttering

Step 2: Check Configuration
  1. In Tuner X, click device
  2. Go to "Configs" tab
  3. Compare configs to other (working) modules
  4. Look for differences in:
     - Inversion settings
     - Current limits
     - PID gains (should be identical across all)
     - Neutral mode

Step 3: Swap Configurations
  1. Temporarily swap CAN IDs of working and non-working module
  2. If problem follows module: hardware issue
  3. If problem stays with CAN ID: configuration issue
```

### Process 2: Binary Search for Code Issues

When robot behavior changed after recent code changes:

```bash
Step 1: Revert to Known Good Commit
  git log --oneline  # Find last working commit
  git checkout <commit_hash>
  # Test - does issue still exist?

Step 2: Binary Search
  # If issue is gone, problem introduced between commits
  git bisect start
  git bisect bad <current_commit>
  git bisect good <last_working_commit>
  # Test each bisect point until finding culprit

Step 3: Analyze Culprit Commit
  git diff <good_commit> <bad_commit>
  # Review what changed, identify root cause
```

### Process 3: Logging and Data Analysis

Capture detailed data for offline analysis:

```java
// Enable comprehensive logging:
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain {
    private int logCounter = 0;

    @Override
    public void periodic() {
        logCounter++;

        // Log key signals (rate-limit to avoid slowdown)
        if (logCounter % 5 == 0) {
            SwerveModuleState[] states = getState().ModuleStates;
            for (int i = 0; i < 4; i++) {
                SmartDashboard.putNumber("Module" + i + "/TargetSpeed",
                    states[i].speedMetersPerSecond);
                SmartDashboard.putNumber("Module" + i + "/TargetAngle",
                    states[i].angle.getDegrees());

                // Add actual values from motors for comparison
            }

            SmartDashboard.putNumber("Drive/Heading",
                getState().Pose.getRotation().getDegrees());
            SmartDashboard.putNumber("Drive/OdometryPeriod",
                getState().OdometryPeriod);
        }
    }
}
```

**Analysis:**

- Export data from Driver Station log
- Import to Excel/Python/MATLAB
- Plot target vs. actual values
- Look for:
  - Steady-state errors (PID tuning)
  - Oscillations (over-tuned)
  - Phase lag (feedforward issues)

---

## Tools and Techniques

### Tool 1: Phoenix Tuner X

**Device List View:**

```
Use For:
- Verify all devices present
- Check for faults (red indicators)
- Validate firmware versions
- View quick status (temperature, voltage, current)
```

**Device Detail View:**

```
Use For:
- Plot signals in real-time
- Manually control motors (testing)
- View/edit configurations
- Check fault history
- Run self-tests
```

**Signal Plotter:**

```
Best Practices:
1. Plot target vs. actual (e.g., TargetVelocity vs. ActualVelocity)
2. Plot motor voltage (should be smooth, not choppy)
3. Plot current (watch for spikes)
4. Use cursors to measure response times
5. Export plots for documentation
```

### Tool 2: WPILib Tools

**Driver Station Log Viewer:**

```
Access: Open "Driver Station Log Viewer" from WPILib tools

Use For:
- Viewing console output after match
- Checking for Java exceptions
- Reviewing command scheduling events
- Correlating events with time stamps
```

**AdvantageScope:**

```
Superior to built-in tools:
1. Records all SmartDashboard/NetworkTables data
2. Visualizes pose and trajectories
3. Plots multiple signals with correlation
4. Replay robot matches frame-by-frame
5. Export data for analysis

Setup:
- Install AdvantageScope from GitHub
- Log data via DataLogManager or AdvantageKit
- Open .wpilog files in AdvantageScope
```

**Glass / Shuffleboard:**

```
Real-time visualization:
- Field2d widget for pose
- Gyro/accelerometer widgets
- Custom widgets for specific data
```

### Tool 3: Version Control (Git)

**Bisecting to Find Regressions:**

```bash
# Automated bisect:
git bisect start
git bisect bad HEAD
git bisect good v1.0-working
git bisect run ./test_script.sh  # Automates testing
```

**Reviewing Changes:**

```bash
# See what changed in specific file:
git diff v1.0-working:TunerConstants.java HEAD:TunerConstants.java

# See all changes to file over time:
git log -p -- src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
```

### Tool 4: Custom Diagnostic Commands

Create test commands for systematic validation:

```java
// In CommandSwerveDrivetrain.java:

/** Tests each module individually */
public Command testModuleSequence() {
    return Commands.sequence(
        runOnce(() -> System.out.println("Testing Front Left...")),
        applyRequest(() -> testSingleModule(0)).withTimeout(2),
        runOnce(() -> System.out.println("Testing Front Right...")),
        applyRequest(() -> testSingleModule(1)).withTimeout(2),
        // ... etc
    );
}

private SwerveRequest testSingleModule(int moduleIndex) {
    // Custom request that only controls one module
    // Useful for isolating problems
    return new SwerveRequest() { /* implementation */ };
}

/** Spins each module to verify encoder offsets */
public Command calibrateModules() {
    return Commands.sequence(
        runOnce(() -> System.out.println("Point all modules forward")),
        applyRequest(() -> new SwerveRequest.Idle()).withTimeout(3),
        runOnce(() -> {
            // Print current module angles
            for (int i = 0; i < 4; i++) {
                System.out.println("Module " + i + ": " +
                    getState().ModuleStates[i].angle.getDegrees() + "°");
            }
        })
    );
}
```

**Use in testing:**

```java
// Bind to controller for quick access:
controller.back().onTrue(drivetrain.testModuleSequence());
controller.start().onTrue(drivetrain.calibrateModules());
```

---

## Code-Specific Issues

### Issue: NullPointerException in Drivetrain

```
Stack Trace:
  at CommandSwerveDrivetrain.periodic(CommandSwerveDrivetrain.java:123)
  at edu.wpi.first.wpilibj2.command.SubsystemBase.periodic()
```

**Common Causes:**

##### 1. Uninitialized SwerveRequest

```java
// BAD: Request not initialized
private SwerveRequest.FieldCentric drive;

@Override
public void periodic() {
    setControl(drive);  // NullPointerException!
}

// GOOD: Initialize in constructor
private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric();
```

##### 2. Accessing State Before Initialization

```java
// BAD: Calling getState() too early
public CommandSwerveDrivetrain(...) {
    super(...);
    Pose2d pose = getState().Pose;  // May be null if called before super() completes!
}

// GOOD: Wait until after construction
public void initializePose() {
    Pose2d pose = getState().Pose;  // Safe
}
```

### Issue: "StatusCode" Error Codes

Phoenix methods return `StatusCode` - check these for errors!

```java
// BAD: Ignoring return value
drivetrain.setControl(request);

// GOOD: Check for errors
StatusCode status = drivetrain.setControl(request);
if (!status.isOK()) {
    System.err.println("Drivetrain control failed: " + status);
}

// BETTER: Handle specific errors
if (status == StatusCode.InvalidDevice) {
    System.err.println("Invalid device - check CAN IDs");
} else if (status == StatusCode.NotSupported) {
    System.err.println("Feature requires Phoenix Pro license");
}
```

### Issue: Configuration Not Applied

```java
// Configurations must be explicitly applied!

// BAD: Creating config but not applying
TalonFXConfiguration config = new TalonFXConfiguration();
config.Slot0.kP = 0.5;
// Motor still has default config!

// GOOD: Apply configuration
TalonFXConfiguration config = new TalonFXConfiguration();
config.Slot0.kP = 0.5;
motor.getConfigurator().apply(config);  // This actually applies it

// BETTER: Check status
StatusCode status = motor.getConfigurator().apply(config);
if (!status.isOK()) {
    System.err.println("Failed to apply config: " + status);
}
```

### Issue: Commands Not Running

```java
// Debug command execution:

// 1. Check if command is scheduled
public void periodic() {
    Command currentCommand = getCurrentCommand();
    System.out.println("Current command: " +
        (currentCommand != null ? currentCommand.getName() : "NONE"));
}

// 2. Check command requirements
public Command exampleCommand() {
    return run(() -> { /* do something */ })
        .withName("ExampleCommand")  // Name for debugging
        .until(() -> conditionMet);  // Explicit end condition
}

// 3. Use command logging
CommandScheduler.getInstance().onCommandInitialize(command ->
    System.out.println("Started: " + command.getName()));
CommandScheduler.getInstance().onCommandInterrupt(command ->
    System.out.println("Interrupted: " + command.getName()));
CommandScheduler.getInstance().onCommandFinish(command ->
    System.out.println("Finished: " + command.getName()));
```

---

## Prevention Strategies

### 1. Pre-Deployment Checklist

```
Before deploying code:
□ Code compiles without warnings
□ Simulation runs without errors
□ All unit tests pass (if applicable)
□ Version control committed with descriptive message
□ Tuner constants backed up
□ Team members notified of changes
```

### 2. Defensive Programming

```java
// Validate inputs
public void setMaxSpeed(double speed) {
    if (speed < 0 || speed > MAX_PHYSICALLY_POSSIBLE_SPEED) {
        System.err.println("Invalid speed: " + speed);
        return;
    }
    this.maxSpeed = speed;
}

// Add assertions for critical assumptions
assert(moduleCount == 4) : "Swerve must have exactly 4 modules";

// Graceful degradation
public Pose2d getPose() {
    SwerveState state = getState();
    if (state == null || state.Pose == null) {
        System.err.println("WARNING: Invalid state, returning last known pose");
        return lastKnownPose;  // Fallback
    }
    return state.Pose;
}
```

### 3. Automated Testing

```java
// Create unit tests for critical functions:
@Test
public void testModuleOptimization() {
    SwerveModuleState desiredState = new SwerveModuleState(
        1.0, Rotation2d.fromDegrees(270));
    SwerveModuleState currentState = new SwerveModuleState(
        0.0, Rotation2d.fromDegrees(45));

    SwerveModuleState optimized =
        SwerveModuleState.optimize(desiredState, currentState);

    // Should rotate <90° by reversing speed
    assertTrue(Math.abs(optimized.angle.minus(currentState.angle).getDegrees()) < 90);
}
```

### 4. Code Reviews

```
Review checklist:
□ Is the change necessary?
□ Are there potential side effects?
□ Are error cases handled?
□ Is code well-commented?
□ Are magic numbers explained/constants?
□ Does it follow team code style?
```

### 5. Incremental Changes

```
Best Practice: Change one thing at a time

Bad Example:
- Updated TunerConstants PID gains
- Refactored CommandSwerveDrivetrain
- Added new autonomous routine
- Changed control scheme
→ If something breaks, which change caused it?

Good Example:
Commit 1: Update steer kP based on tuning
Commit 2: Test and verify improvement
Commit 3: Update drive kV based on SysId
Commit 4: Test and verify improvement
→ Easy to identify what changed and roll back if needed
```

---

## Emergency Procedures

### During Competition - Quick Fixes

**Robot completely non-functional:**

```bash
1. Revert to last known-good code
   git checkout last-working-tag
   gradle deploy

2. Power cycle everything
   - Robot breaker
   - Driver station
   - Radio

3. Check battery voltage (>12.5V required)

4. Re-verify all devices in Tuner X
```

**Specific module not working:**

```bash
1. Quick swap CAN IDs with working module (software)
2. If that works: configuration issue, fix config
3. If that doesn't work: hardware issue, need repair
```

**Driving poorly but functional:**

```bash
1. Switch to robot-centric mode (if field-centric having issues)
2. Reduce max speed/acceleration (Constants.java)
3. Use conservative control request types:
   .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
   .withSteerRequestType(SteerRequestType.MotionMagic)
```
