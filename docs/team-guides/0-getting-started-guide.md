# Getting Started: From Generated Code to Driving Robot

**A practical walkthrough for Team 4050**

You've just used Phoenix Tuner X to generate your swerve drive code. Now what? This guide will walk you through everything you need to do to get your robot driving smoothly, from that first deployment to competition-ready performance.

---

## What You're Starting With

After running Tuner X's code generation, you should have:

- **TunerConstants.java** in `src/main/java/frc/robot/generated/`
- **CommandSwerveDrivetrain.java** that extends the generated base class
- A basic **RobotContainer.java** with drivetrain instantiation

This is your foundation. Let's build on it!

---

## Phase 1: First Deploy and Basic Driving (Day 1)

### Step 1: Verify Your Hardware Configuration

Before deploying anything, open `TunerConstants.java` and double-check these critical values:

```java
// CAN IDs - must match your actual hardware!
private static final int kFrontLeftDriveMotorId = 1;
private static final int kFrontLeftSteerMotorId = 0;
private static final int kFrontLeftEncoderId = 0;
// ... repeat for all modules

// CAN Bus name
public static final CANBus kCANBus = new CANBus(""); // "" = roboRIO bus
// Or if using CANivore: new CANBus("canivore_name")

// Pigeon 2.0
private static final int kPigeonId = 2;
```

**Pro tip:** Open Phoenix Tuner X and verify all 13 devices show up on the CAN bus (4 drive motors, 4 steer motors, 4 CANcoders, 1 Pigeon 2.0). If any are missing, you'll have problems!

### Step 2: Set Up Basic Teleop Control

In your `RobotContainer.java`, you need to wire up joystick control. Here's the pattern:

```java
public class RobotContainer {
    // Create max speed constant (from TunerConstants)
    private static final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private static final double MaxAngularRate = Math.PI * 1.5; // 1.5 rotations per second

    // Your controller
    private final CommandXboxController m_driverController = new CommandXboxController(0);

    // Drivetrain subsystem
    private final CommandSwerveDrivetrain m_drivetrain = TunerConstants.createDrivetrain();

    // Field-centric drive request
    private final SwerveRequest.FieldCentric m_driveRequest =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Default command: field-centric drive
        m_drivetrain.setDefaultCommand(
            m_drivetrain.applyRequest(() ->
                m_driveRequest
                    .withVelocityX(-m_driverController.getLeftY() * MaxSpeed)
                    .withVelocityY(-m_driverController.getLeftX() * MaxSpeed)
                    .withRotationalRate(-m_driverController.getRightX() * MaxAngularRate)
            )
        );

        // X button: X-formation brake
        m_driverController.x().whileTrue(
            m_drivetrain.applyRequest(() -> new SwerveRequest.SwerveDriveBrake())
        );

        // A button: Reset field-centric (zero heading to current direction)
        m_driverController.a().onTrue(
            m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric())
        );
    }

    public Command getAutonomousCommand() {
        return Commands.print("No auto configured yet!");
    }
}
```

**What's happening here:**

- **Field-centric drive:** The left stick controls direction relative to the field (pushing forward always moves downfield, regardless of robot orientation)
- **Rotation:** Right stick X-axis controls how fast the robot spins
- **Deadband:** 10% deadband prevents stick drift from causing unwanted movement
- **OpenLoopVoltage:** Direct voltage control - fastest response, good for driver control

### Step 3: Deploy and Test Drive

1. **Deploy the code:** Hit F5 in VS Code or run `./gradlew deploy`
2. **Enable in teleop mode**
3. **Try driving:**
   - Left stick forward/back/left/right → robot should translate
   - Right stick left/right → robot should rotate
   - Wheels should point in the direction of intended travel

**What to look for on first drive:**

✅ **Good signs:**

- Robot moves in the direction you push the stick
- Wheels optimize (never rotate more than 90°)
- Field-centric works (forward stick = downfield, even when robot is sideways)

⚠️ **Warning signs:**

- Robot drifts when sticks are neutral → Check deadband or current limits
- Wheels "jitter" or oscillate → Steer gains need tuning (see Phase 2)
- Robot moves in wrong direction → Check motor inversions in TunerConstants
- Field-centric doesn't work → Zero heading with A button while facing downfield

### Step 4: Calibrate Your Wheel Offsets

This is **critical** for accurate wheel pointing. With the robot disabled:

1. Manually rotate all wheels to point straight forward (parallel to robot centerline)
2. In Tuner X, read the absolute position of each CANcoder
3. Update `TunerConstants.java` with these offsets:

```java
private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.123456);
private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.234567);
// etc...
```

4. Re-deploy and verify: disabled robot should show all wheels pointing forward

---

## Phase 2: Tuning for Performance (Days 2-3)

Your robot drives, but it probably doesn't drive _well_ yet. Time to tune!

### The Big Picture

You have three control loops to tune, in this order:

1. **Steer motors** (wheel angle control) - Most critical
2. **Drive motors** (wheel speed control) - Important for autonomous
3. **Heading controller** (rotation control) - Nice for advanced features

### Quick Tuning: Steer Motors

The generated code has default steer gains that usually work okay, but you can improve them:

**In TunerConstants.java, find:**

```java
private static final Slot0Configs steerGains = new Slot0Configs()
    .withKP(100)    // Start here
    .withKI(0)      // Leave at 0
    .withKD(0.5)    // Can increase for damping
    .withKS(0.1)    // Static friction
    .withKV(2.66)   // Velocity feedforward
    .withKA(0);     // Acceleration feedforward
```

**Tuning procedure (manual method):**

1. Deploy and enable in teleop
2. Drive the robot and watch wheel behavior
3. If wheels oscillate/jitter → **Reduce kP** (try 80, then 60)
4. If wheels are slow to reach target angle → **Increase kP** (try 120, then 140)
5. If wheels overshoot and bounce back → **Increase kD** (try 1.0, then 2.0)

**Goal:** Wheels should snap to the correct angle quickly without oscillating.

### Optional: SysId for Drive Motors

For the best autonomous performance, characterize your drive motors using WPILib's SysId tool:

**Quick steps:**

1. In your `CommandSwerveDrivetrain`, you already have SysId routines built-in!
2. Add these to your auto chooser for testing:

```java
// In RobotContainer
private void configureBindings() {
    // ... existing bindings ...

    // SysId bindings (for tuning only!)
    m_driverController.back().and(m_driverController.y()).whileTrue(
        m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kForward)
    );
    m_driverController.back().and(m_driverController.a()).whileTrue(
        m_drivetrain.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)
    );
    m_driverController.back().and(m_driverController.b()).whileTrue(
        m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kForward)
    );
    m_driverController.back().and(m_driverController.x()).whileTrue(
        m_drivetrain.sysIdDynamic(SysIdRoutine.Direction.kReverse)
    );
}
```

3. Run the routines, analyze in SysId tool, update kS and kV in drive gains

**Reality check:** The default drive gains usually work fine for basic driving. SysId matters most when you're doing precise autonomous paths or want maximum performance.

---

## Phase 3: Setting Up Autonomous (Days 3-5)

### Install PathPlanner

PathPlanner is the standard tool for FRC autonomous planning:

1. Download PathPlanner from GitHub releases
2. Create a `pathplanner` folder in your `deploy` directory
3. Launch PathPlanner and point it to your robot project

### Configure PathPlanner Integration

Add this to your `RobotContainer` constructor:

```java
public RobotContainer() {
    configurePathPlanner();
    configureBindings();
}

private void configurePathPlanner() {
    // Configure AutoBuilder
    AutoBuilder.configure(
        m_drivetrain::getPose,              // Supplier of current pose
        m_drivetrain::resetPose,            // Consumer to reset pose
        m_drivetrain::getChassisSpeeds,     // Supplier of current speeds
        (speeds, feedforwards) -> m_drivetrain.setControl(
            new SwerveRequest.ApplyChassisSpeeds()
                .withSpeeds(speeds)
                .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesX())
                .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesY())
        ),
        new PPHolonomicDriveController(
            new PIDConstants(5.0, 0, 0),    // Translation PID
            new PIDConstants(5.0, 0, 0)     // Rotation PID
        ),
        new RobotConfig(
            Units.lbsToKilograms(110),      // Robot mass (with battery/bumpers)
            Units.lbsToKilograms(5.5),      // Robot MOI (tune if needed)
            new ModuleConfig(
                Units.inchesToMeters(2),    // Wheel radius
                MaxSpeed,                    // Max speed
                1.2,                        // Wheel COF
                new DCMotor(1, 1, 1, 1, 1, 1), // Drive motor (use real values)
                60,                          // Drive current limit
                1                            // Drive motors per module
            ),
            Units.inchesToMeters(21)        // Drivebase radius (center to module)
        ),
        () -> {
            // Flip path for red alliance
            var alliance = DriverStation.getAlliance();
            return alliance.isPresent() && alliance.get() == Alliance.Red;
        },
        m_drivetrain
    );
}
```

**Note:** The exact RobotConfig parameters depend on your physical robot. Measure your robot's dimensions and mass!

### Create Your First Path

In PathPlanner:

1. **New Path:** Click "+" to create a path
2. **Name it:** Something like "SimpleForward"
3. **Add waypoints:**
   - Starting pose: Where your robot starts (usually near alliance wall)
   - Ending pose: Where you want to go
4. **Set constraints:**
   - Max velocity: Start conservative (2 m/s)
   - Max acceleration: Start conservative (2 m/s²)
5. **Set rotation:**
   - You can set heading at each waypoint
   - Or use "Follow Path" rotation mode
6. **Save** and close PathPlanner

### Load and Run the Path

Back in your code:

```java
public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("SimpleForward");
}
```

Deploy, enable in autonomous mode, and watch it go!

**Troubleshooting your first auto:**

- **Robot doesn't move:** Check that path is saved in `deploy/pathplanner/autos/`
- **Robot goes wrong direction:** Check alliance flipping logic
- **Path is jerky:** Reduce max velocity/acceleration constraints
- **Robot doesn't face correct direction:** Tune rotation PID in PPHolonomicDriveController

---

## Phase 4: Adding Telemetry and Logging (Ongoing)

Good telemetry makes debugging so much easier!

### Basic Dashboard Integration

Add this to your `CommandSwerveDrivetrain.periodic()`:

```java
@Override
public void periodic() {
    // Log pose
    SmartDashboard.putNumber("Pose X", getState().Pose.getX());
    SmartDashboard.putNumber("Pose Y", getState().Pose.getY());
    SmartDashboard.putNumber("Heading", getState().Pose.getRotation().getDegrees());

    // Log speeds
    ChassisSpeeds speeds = getState().Speeds;
    SmartDashboard.putNumber("Velocity X", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Velocity Y", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Omega", Math.toDegrees(speeds.omegaRadiansPerSecond));
}
```

### Field Visualization

Create a Field2d object for visualization in Glass/Shuffleboard:

```java
// In CommandSwerveDrivetrain
private final Field2d m_field = new Field2d();

public CommandSwerveDrivetrain(...) {
    // ... existing constructor code ...
    SmartDashboard.putData("Field", m_field);
}

@Override
public void periodic() {
    m_field.setRobotPose(getState().Pose);
}
```

Now open Glass or Shuffleboard and add the "Field" widget. You'll see your robot's position and orientation!

### Enable Phoenix Signal Logging

In your `Robot.java` `robotInit()`:

```java
@Override
public void robotInit() {
    m_robotContainer = new RobotContainer();

    // Enable Signal Logger
    if (isReal()) {
        SignalLogger.setPath("/media/sda1/");  // USB stick
    } else {
        SignalLogger.setPath("./logs/");       // Sim logs
    }
    SignalLogger.start();
}
```

This automatically logs all Phoenix device signals to a file. After a match, pull the USB stick and analyze in Tuner X!

---

## Phase 5: Pre-Competition Checklist

You're almost ready to compete! Go through this checklist:

### Hardware Verification

- [ ] All 13 CAN devices visible in Tuner X (4 drive + 4 steer + 4 CANcoder + 1 Pigeon2)
- [ ] No faults on any device
- [ ] CANcoder offsets calibrated (wheels straight when disabled)
- [ ] Pigeon 2.0 mounted rigidly (no vibration)
- [ ] All motors spin in correct direction
- [ ] Current limits configured (60A steer, 80A drive)

### Software Verification

- [ ] Field-centric control works correctly
- [ ] Brake mode (X button) creates X-formation
- [ ] Autonomous paths load without errors
- [ ] Telemetry appears on dashboard
- [ ] Signal Logger writes to USB drive
- [ ] No console errors or warnings

### Driver Practice

- [ ] Drivers comfortable with field-centric control
- [ ] Practiced resetting heading (A button)
- [ ] Tested maximum speed (adjust MaxSpeed if too fast/slow)
- [ ] Practiced defensive brake position (X button)
- [ ] Practiced emergency stop (disable button)

### Autonomous Testing

- [ ] All autonomous routines tested on practice field
- [ ] Starting positions marked and verified
- [ ] Alliance color flipping works correctly
- [ ] Paths stay within field boundaries
- [ ] Timing validated (auto period = 15 seconds)

---

## Common "Gotchas" and How to Fix Them

### Robot Spins Uncontrollably

**Problem:** Robot rotates when you don't want it to.

**Fixes:**

- Increase rotational deadband: `.withRotationalDeadband(MaxAngularRate * 0.15)`
- Check for stuck joystick axis
- Verify joystick axis mapping (might be using wrong axis)

### Wheels Point Wrong Direction

**Problem:** Wheels don't point where you expect.

**Fixes:**

- Re-calibrate CANcoder offsets
- Check motor inversions in TunerConstants
- Verify steer motor CAN IDs are correct

### Robot Drifts During Autonomous

**Problem:** Robot doesn't follow path accurately.

**Fixes:**

- Tune drive motor kV using SysId
- Increase PathPlanner translation PID gains
- Check wheel tread wear (replace if needed)
- Verify battery is fully charged (>12.5V)

### Field-Centric Doesn't Work

**Problem:** Field-centric mode doesn't maintain field orientation.

**Fixes:**

- Zero heading with robot facing downfield: `m_drivetrain.seedFieldCentric()`
- Check Pigeon 2.0 is working in Tuner X
- Verify Pigeon 2.0 mounting (must be level and rigid)
- Check for gyro drift (might need to re-zero periodically)

### Robot Oscillates/Jitters

**Problem:** Wheels or robot shake/oscillate.

**Fixes:**

- Reduce steer kP gain
- Increase steer kD gain
- Check for mechanical slop in modules
- Verify no loose wiring causing intermittent connections

---

## What's Next?

You now have a fully functional swerve drive! Here are some next-level features to consider:

### Advanced Control Features

- **Heading Lock:** Drive while maintaining current heading (prevents rotation from stick drift)
- **Snap-to-Angle:** Quick button bindings to face specific angles (0°, 90°, 180°, 270°)
- **Slow Mode:** Reduced speed mode for precise positioning
- **Auto-Align:** Use vision to automatically align to scoring positions

### Vision Integration

- **PhotonVision Setup:** Add camera for AprilTag detection
- **Pose Estimation:** Fuse vision measurements with odometry
- **Auto-Target:** Automatically aim at game pieces or scoring locations

### Performance Optimization

- **CANivore:** Upgrade to CANivore for 1ms CAN bus updates
- **Slip Detection:** Monitor and compensate for wheel slip
- **Battery Voltage Compensation:** Maintain consistent performance throughout match
- **Path Optimization:** Fine-tune PathPlanner configurations for faster cycle times

### Competition Features

- **Alliance Selection:** Automatically configure for red/blue alliance
- **Match Logging:** Enhanced logging with match number and event name
- **Autonomous Selector:** Dashboard-based autonomous routine selection
- **Pre-Match Checks:** Automated system health checks before matches
