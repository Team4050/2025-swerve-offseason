# Swerve Drive Optimization & Advanced Features

## Table of Contents

1. [Vision Integration for Pose Estimation](#vision-integration-for-pose-estimation)
2. [Advanced Control Features](#advanced-control-features)
3. [Performance Optimization](#performance-optimization)
4. [Software Architecture Improvements](#software-architecture-improvements)
5. [Competition-Ready Features](#competition-ready-features)
6. [Next-Level Capabilities](#next-level-capabilities)

---

## Vision Integration for Pose Estimation

### Why Vision?

**Problem:** Wheel odometry drifts over time due to:

- Wheel slip
- Floor surface variations
- Carpet wear
- Collisions with other robots

**Solution:** Fuse vision-based pose estimation with odometry using Kalman filtering.

**Benefits:**

- Accurate field-relative positioning
- Enables precise autonomous anywhere on field
- Corrects for odometry drift
- Supports dynamic target tracking

---

### AprilTag System Overview

**AprilTags** are fiducial markers placed around the FRC field. Each tag has a unique ID and known position.

```
FRC Field Layout:
- 16+ AprilTags at known locations
- Tags identify: scoring locations, loading zones, field landmarks
- PhotonVision/Limelight detects tags and calculates robot pose
```

### Hardware Options

#### Option 1: Limelight (Commercial Solution)

```
Pros:
✓ Pre-configured for FRC
✓ Built-in processing
✓ Easy setup
✓ Proven reliability

Cons:
✗ Expensive ($400-500)
✗ Less customizable
```

#### Option 2: PhotonVision (Open Source)

```
Pros:
✓ Free (open source)
✓ Highly customizable
✓ Active development
✓ Runs on Raspberry Pi or Orange Pi

Cons:
✗ Requires more setup
✗ Need to source hardware
✗ More troubleshooting needed
```

**Recommendation for Team 4050:** Start with PhotonVision for cost-effectiveness and learning opportunity.

---

### Implementation: PhotonVision + AprilTags

#### Step 1: Hardware Setup

**Required Hardware:**

- Raspberry Pi 4 (4GB+ recommended) or Orange Pi 5
- USB camera (Logitech C920 or equivalent)
- Ethernet cable or WiFi connection to robot
- Rigid camera mount

**Camera Placement:**

- Height: ~12-18" above ground
- Angle: Slightly upward (10-20°)
- Location: Front or back of robot, unobstructed view
- Multiple cameras: Increase field of view coverage

#### Step 2: Software Installation

```bash
# On Raspberry Pi:
# 1. Download PhotonVision image from:
#    https://photonvision.org/docs/installation/sw_install/

# 2. Flash to SD card using Raspberry Pi Imager

# 3. Boot Pi, connect to robot network

# 4. Access web interface:
#    http://photonvision.local:5800
```

#### Step 3: Camera Calibration

```
In PhotonVision UI:
1. Go to "Settings" tab
2. Select your camera
3. Run "Camera Calibration"
4. Use provided calibration board pattern
5. Capture 12+ images at various angles and distances
6. Run calibration algorithm
7. Save results
```

**Critical:** Accurate calibration is essential for accurate pose estimation!

#### Step 4: AprilTag Pipeline Configuration

```
In PhotonVision UI:
1. Create new pipeline: "AprilTag3D"
2. Select pipeline type: "AprilTag"
3. Enable "Multi-Target"
4. Set "Tag Family": "36h11" (FRC standard)
5. Enable "3D mode" for pose estimation
6. Save pipeline
```

#### Step 5: Robot-to-Camera Transform

Measure camera position relative to robot center:

```java
// In Constants.java or VisionConstants.java:
public static final class VisionConstants {
    // Camera mounted 0.3m forward, 0.0m sideways, 0.5m up from robot center
    // Angled upward 20 degrees
    public static final Transform3d ROBOT_TO_CAMERA = new Transform3d(
        new Translation3d(0.3, 0.0, 0.5),  // X, Y, Z in meters
        new Rotation3d(0, Math.toRadians(-20), 0)  // Roll, Pitch, Yaw
    );

    // If multiple cameras, define each:
    public static final Transform3d ROBOT_TO_FRONT_CAMERA = ...
    public static final Transform3d ROBOT_TO_BACK_CAMERA = ...
}
```

**Measuring Tips:**

- Use CAD model if available
- Physically measure with ruler/calipers
- Verify angles with protractor or level

#### Step 6: Code Integration

**Add PhotonLib dependency:**

```bash
# In VS Code Command Palette:
# "WPILib: Manage Vendor Libraries"
# "Install new libraries (online)"
# URL: https://maven.photonvision.org/repository/internal/org/photonvision/photonlib-json/1.0/photonlib-json-1.0.json
```

**Create Vision Subsystem:**

```java
package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import java.util.Optional;

import static frc.robot.Constants.VisionConstants.*;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera;
    private final PhotonPoseEstimator poseEstimator;
    private final AprilTagFieldLayout fieldLayout;

    public VisionSubsystem() {
        // Initialize camera (name must match PhotonVision settings)
        camera = new PhotonCamera("FrontCamera");

        // Load AprilTag field layout
        try {
            fieldLayout = AprilTagFieldLayout.loadFromResource(
                AprilTagFields.k2024Crescendo.m_resourceFile
            );
        } catch (Exception e) {
            throw new RuntimeException("Failed to load AprilTag field layout", e);
        }

        // Configure pose estimator
        poseEstimator = new PhotonPoseEstimator(
            fieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,  // Best accuracy
            camera,
            ROBOT_TO_CAMERA
        );

        poseEstimator.setMultiTagFallbackStrategy(
            PoseStrategy.LOWEST_AMBIGUITY
        );
    }

    /**
     * Get estimated robot pose from vision
     * @return Optional containing estimated pose and timestamp, or empty if no tags visible
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        return poseEstimator.update();
    }

    /**
     * Check if any AprilTags are currently visible
     */
    public boolean hasTargets() {
        return camera.getLatestResult().hasTargets();
    }

    /**
     * Get number of tags currently visible
     */
    public int getTagCount() {
        return camera.getLatestResult().getTargets().size();
    }
}
```

**Integrate with Drivetrain:**

Modify `CommandSwerveDrivetrain.java`:

```java
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    // ... existing code ...

    private final VisionSubsystem vision;

    public CommandSwerveDrivetrain(VisionSubsystem visionSubsystem, ...) {
        super(...);
        this.vision = visionSubsystem;
    }

    @Override
    public void periodic() {
        // Update odometry with vision measurements
        Optional<EstimatedRobotPose> visionPose = vision.getEstimatedGlobalPose();

        if (visionPose.isPresent()) {
            EstimatedRobotPose estimate = visionPose.get();

            // Add vision measurement to pose estimator
            // Standard deviations represent measurement uncertainty
            addVisionMeasurement(
                estimate.estimatedPose.toPose2d(),
                estimate.timestampSeconds,
                calculateVisionStdDevs(estimate)
            );
        }
    }

    /**
     * Calculate standard deviations for vision measurement
     * Lower = more trust in measurement
     */
    private Matrix<N3, N1> calculateVisionStdDevs(EstimatedRobotPose estimate) {
        // Base standard deviations (meters and radians)
        double xyStdDev = 0.5;
        double thetaStdDev = 0.5;

        // Increase uncertainty based on distance to tags
        int numTags = estimate.targetsUsed.size();
        double avgDistance = estimate.targetsUsed.stream()
            .mapToDouble(target -> target.getBestCameraToTarget().getTranslation().getNorm())
            .average()
            .orElse(5.0);

        // More tags = more confidence, closer = more confidence
        if (numTags >= 2) {
            xyStdDev = 0.5;
            thetaStdDev = 0.5;
        } else if (numTags == 1) {
            xyStdDev = 1.0;
            thetaStdDev = 2.0;
        }

        // Scale by distance (farther = less confident)
        xyStdDev *= (1 + (avgDistance / 5.0));
        thetaStdDev *= (1 + (avgDistance / 5.0));

        return VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
    }
}
```

**Wire up in RobotContainer:**

```java
public class RobotContainer {
    private final VisionSubsystem vision = new VisionSubsystem();
    private final CommandSwerveDrivetrain drivetrain =
        TunerConstants.createDrivetrain(vision);  // Pass vision subsystem

    // ... rest of code
}
```

#### Step 7: Testing and Tuning

**Test Procedure:**

```
1. Place robot at known position on field
2. Enable robot
3. Check SmartDashboard for vision pose vs. odometry pose
4. Drive around, observe pose correction when tags visible
5. Tune standard deviations if:
   - Vision pose jumps erratically: Increase std devs (less trust)
   - Odometry drift not corrected: Decrease std devs (more trust)
```

**Logging for Analysis:**

```java
@Override
public void periodic() {
    // Log both poses for comparison
    SmartDashboard.putNumberArray("Vision/Pose", new double[] {
        visionPose.getX(), visionPose.getY(), visionPose.getRotation().getDegrees()
    });
    SmartDashboard.putNumberArray("Odometry/Pose", new double[] {
        getState().Pose.getX(), getState().Pose.getY(),
        getState().Pose.getRotation().getDegrees()
    });
    SmartDashboard.putNumber("Vision/TagCount", vision.getTagCount());
}
```

---

## Advanced Control Features

### Feature 1: Heading Lock / Field-Centric Facing Angle

**What it does:** Driver controls translation (X/Y), robot automatically maintains or seeks a heading.

**Use cases:**

- Keep robot facing scoring target while driving
- Maintain orientation while strafing
- Quick snap to cardinal directions

**Implementation:**

Already built into Phoenix! Use `FieldCentricFacingAngle` request:

```java
// In RobotContainer:
private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
    new SwerveRequest.FieldCentricFacingAngle()
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(MaxAngularRate * 0.1);

private void configureBindings() {
    // Face scoring target while driving
    controller.rightBumper().whileTrue(
        drivetrain.applyRequest(() ->
            driveToAngle
                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withTargetDirection(getTargetDirection())  // Calculate based on position
        )
    );
}

/**
 * Calculate direction to scoring target based on robot position
 */
private Rotation2d getTargetDirection() {
    Pose2d robotPose = drivetrain.getPose();
    Translation2d targetPosition = new Translation2d(8.0, 4.0);  // Example target

    Translation2d delta = targetPosition.minus(robotPose.getTranslation());
    return new Rotation2d(delta.getX(), delta.getY());
}
```

**Advanced: Dynamic Target Tracking:**

```java
public Command trackScoringTarget() {
    return drivetrain.applyRequest(() -> {
        // Continuously update target based on robot position and target location
        Pose2d currentPose = drivetrain.getPose();
        Translation2d targetPos = getClosestScoringPosition(currentPose);
        Rotation2d targetHeading = calculateHeading(currentPose, targetPos);

        return driveToAngle
            .withVelocityX(-controller.getLeftY() * MaxSpeed)
            .withVelocityY(-controller.getLeftX() * MaxSpeed)
            .withTargetDirection(targetHeading);
    });
}
```

### Feature 2: Path Following During Teleop

**What it does:** Follow pre-planned paths during teleop for consistent positioning.

**Use cases:**

- Navigate to scoring positions
- Return to loading zone
- Defensive positioning

**Implementation:**

```java
public Command driveToScoringPosition() {
    return Commands.either(
        // If we have a path to scoring position
        AutoBuilder.pathfindToPose(
            new Pose2d(2.0, 5.5, Rotation2d.fromDegrees(0)),  // Target pose
            new PathConstraints(
                3.0, 2.0,   // Max vel/accel
                5.0, 4.0    // Max angular vel/accel
            ),
            0.0  // End velocity
        ),
        // Fallback: manual control
        drivetrain.applyRequest(() -> drive.with...),
        () -> vision.hasTargets()  // Only pathfind if we can localize
    );
}

// Bind to button:
controller.a().onTrue(driveToScoringPosition());
```

**PathPlanner Pathfinding:**

- Automatically generates path from current position to target
- Avoids obstacles (if configured)
- Respects robot constraints

### Feature 3: Snap to Angles

**What it does:** Quickly rotate to cardinal directions (0°, 90°, 180°, 270°).

**Use cases:**

- Quick orientation for scoring
- Defense positioning
- Recovery from being pushed

**Implementation:**

```java
// In RobotContainer:
private void configureBindings() {
    // D-pad snap to angles
    controller.povUp().whileTrue(
        drivetrain.applyRequest(() ->
            driveToAngle
                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(0))  // Forward
        )
    );

    controller.povRight().whileTrue(
        drivetrain.applyRequest(() ->
            driveToAngle
                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromDegrees(90))  // Right
        )
    );

    // Similar for povDown() -> 180° and povLeft() -> 270°
}
```

### Feature 4: Speed Modes

**What it does:** Multiple speed/sensitivity profiles for different scenarios.

**Use cases:**

- Precise mode for scoring
- Turbo mode for fast traversal
- Defensive mode (slower, more stable)

**Implementation:**

```java
public class RobotContainer {
    // Speed multipliers
    private static final double NORMAL_SPEED_MULTIPLIER = 1.0;
    private static final double PRECISE_SPEED_MULTIPLIER = 0.3;
    private static final double TURBO_SPEED_MULTIPLIER = 1.5;  // If safe

    private double currentSpeedMultiplier = NORMAL_SPEED_MULTIPLIER;

    private void configureBindings() {
        // Hold for precise mode
        controller.leftBumper().whileTrue(
            Commands.runOnce(() -> currentSpeedMultiplier = PRECISE_SPEED_MULTIPLIER)
        ).onFalse(
            Commands.runOnce(() -> currentSpeedMultiplier = NORMAL_SPEED_MULTIPLIER)
        );

        // Hold for turbo mode
        controller.rightTrigger().whileTrue(
            Commands.runOnce(() -> currentSpeedMultiplier = TURBO_SPEED_MULTIPLIER)
        ).onFalse(
            Commands.runOnce(() -> currentSpeedMultiplier = NORMAL_SPEED_MULTIPLIER)
        );

        // Default command uses current multiplier
        drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-controller.getLeftY() * MaxSpeed * currentSpeedMultiplier)
                     .withVelocityY(-controller.getLeftX() * MaxSpeed * currentSpeedMultiplier)
                     .withRotationalRate(-controller.getRightX() * MaxAngularRate * currentSpeedMultiplier)
            )
        );
    }
}
```

### Feature 5: Trajectory Setpoint Visualization

**What it does:** Shows where robot is trying to go in real-time.

**Use cases:**

- Debugging path following
- Driver feedback
- Tuning PID controllers

**Implementation:**

```java
// In CommandSwerveDrivetrain:
private final Field2d field = new Field2d();

public CommandSwerveDrivetrain(...) {
    super(...);
    SmartDashboard.putData("Field", field);
}

@Override
public void periodic() {
    // Show current pose
    field.setRobotPose(getState().Pose);

    // Show target pose (if following trajectory)
    // Access from PathPlanner or custom trajectory follower
    // field.getObject("target").setPose(targetPose);
}
```

---

## Performance Optimization

### Optimization 1: CANivore + Phoenix Pro

**What it does:** Offloads CAN traffic from roboRIO to dedicated CAN FD bus.

**Benefits:**

- 10x faster signal updates (250 Hz → 1000+ Hz)
- Reduced roboRIO CPU load
- Time-synchronized odometry updates
- Better path following accuracy

**Cost:** Phoenix Pro license (~$150/device or $1000 season pass)

**Implementation:**

```java
// In TunerConstants.java:
// Change from:
public static final CANBus kCANBus = new CANBus("");  // roboRIO

// To:
public static final CANBus kCANBus = new CANBus("canivore1");  // CANivore name
```

**Hardware setup:**

1. Connect CANivore to roboRIO via USB
2. Connect swerve devices to CANivore CAN bus (not roboRIO)
3. Name CANivore "canivore1" in Tuner X
4. Apply Pro licenses to devices

**Expected improvements:**

- Odometry update rate: 250 Hz
- Pose estimation accuracy: ±5mm
- Path following accuracy: ±2cm

### Optimization 2: Custom Odometry Thread

**What it does:** Moves odometry updates to higher-priority thread.

**Benefits:**

- More consistent update timing
- Lower latency
- Smoother control

**Implementation:**

Phoenix 6 swerve already does this automatically! Odometry runs on separate thread at max CAN bus speed.

**Verify it's working:**

```java
@Override
public void periodic() {
    double odometryFrequency = 1.0 / getState().OdometryPeriod;
    SmartDashboard.putNumber("Odometry Hz", odometryFrequency);
    // Should see 250 Hz (roboRIO) or 1000+ Hz (CANivore Pro)
}
```

### Optimization 3: Signal Update Frequency Tuning

**What it does:** Reduce update rate of non-critical signals to free CAN bandwidth.

**Implementation:**

```java
// In TunerConstants or CommandSwerveDrivetrain:
public void optimizeCANUsage() {
    // Critical signals - keep at high rate
    for (SwerveModule module : getModules()) {
        module.getDriveMotor().getVelocity().setUpdateFrequency(100);  // Hz
        module.getSteerMotor().getPosition().setUpdateFrequency(100);
        module.getCANcoder().getPosition().setUpdateFrequency(100);
    }

    // Non-critical signals - reduce rate
    for (SwerveModule module : getModules()) {
        module.getDriveMotor().getSupplyVoltage().setUpdateFrequency(4);
        module.getDriveMotor().getSupplyCurrent().setUpdateFrequency(4);
        module.getDriveMotor().getMotorVoltage().setUpdateFrequency(4);
        // Similar for steer motors...
    }

    // Apply optimized update frequencies
    BaseStatusSignal.setUpdateFrequencyForAll(100,
        /* List all critical signals here */
    );
}
```

### Optimization 4: Reduce Logging Overhead

**What it does:** Minimize CPU/network usage from excessive logging.

**Implementation:**

```java
@Override
public void periodic() {
    // Rate-limit logging
    if (loopCounter % 5 == 0) {  // Only every 5 loops (10 Hz instead of 50 Hz)
        SmartDashboard.putNumber("Drive/Speed", getCurrentSpeed());
        SmartDashboard.putNumber("Drive/Heading", getHeading().getDegrees());
    }
    loopCounter++;

    // Disable non-essential logging during competition
    if (DriverStation.isFRCMatch()) {
        // Minimal logging only
    } else {
        // Full diagnostic logging
    }
}
```

---

## Software Architecture Improvements

### Improvement 1: State Machine for Autonomous

**What it does:** Robust autonomous routines with error recovery.

**Implementation:**

```java
package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

public class StateMachineAuto {
    private enum State {
        DRIVE_TO_GAMEPIECE,
        INTAKE,
        DRIVE_TO_SCORE,
        SCORE,
        DONE
    }

    private State currentState = State.DRIVE_TO_GAMEPIECE;

    public Command buildAuto(CommandSwerveDrivetrain drivetrain) {
        return Commands.sequence(
            // State 1: Drive to game piece
            Commands.either(
                driveToGamePiece(drivetrain),
                Commands.none(),  // Skip if already have piece
                this::needsGamePiece
            ),

            // State 2: Intake
            intakeCommand().withTimeout(2.0),

            // State 3: Drive to scoring
            driveToScore(drivetrain),

            // State 4: Score
            scoreCommand(),

            // Done
            Commands.runOnce(() -> currentState = State.DONE)
        );
    }

    private boolean needsGamePiece() {
        // Check sensor, vision, or state
        return !hasGamePiece();
    }

    // ... implement other methods
}
```

### Improvement 2: Subsystem Manager

**What it does:** Centralized subsystem state management and coordination.

**Implementation:**

```java
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RobotStateManager extends SubsystemBase {
    // Singleton pattern
    private static RobotStateManager instance;

    public static RobotStateManager getInstance() {
        if (instance == null) {
            instance = new RobotStateManager();
        }
        return instance;
    }

    // Robot states
    public enum RobotMode {
        IDLE,
        INTAKING,
        SCORING,
        CLIMBING
    }

    private RobotMode currentMode = RobotMode.IDLE;
    private Pose2d targetPose = null;
    private boolean hasGamePiece = false;

    // Getters/setters
    public RobotMode getMode() { return currentMode; }
    public void setMode(RobotMode mode) { this.currentMode = mode; }

    public boolean hasGamePiece() { return hasGamePiece; }
    public void setHasGamePiece(boolean has) { this.hasGamePiece = has; }

    public Optional<Pose2d> getTargetPose() {
        return Optional.ofNullable(targetPose);
    }
    public void setTargetPose(Pose2d pose) { this.targetPose = pose; }

    @Override
    public void periodic() {
        // Update dashboard
        SmartDashboard.putString("Robot Mode", currentMode.toString());
        SmartDashboard.putBoolean("Has Game Piece", hasGamePiece);
    }
}
```

### Improvement 3: Advanced Logging with AdvantageKit

**What it does:** Deterministic replay and comprehensive logging.

**Benefits:**

- Replay matches exactly as they occurred
- Analyze issues post-match
- Compare different configurations

**Implementation:**

See AdvantageKit documentation: https://github.com/Mechanical-Advantage/AdvantageKit

**Quick start:**

```java
// In Robot.java:
import org.littletonrobotics.junction.LoggedRobot;

public class Robot extends LoggedRobot {
    @Override
    public void robotInit() {
        Logger.recordMetadata("ProjectName", "2025-Swerve");
        Logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);

        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter());
            Logger.addDataReceiver(new NT4Publisher());
        } else {
            Logger.addDataReceiver(new WPILOGWriter());
        }

        Logger.start();

        // ... rest of initialization
    }
}
```

### Improvement 4: Unit Testing

**What it does:** Automated testing of critical functions.

**Implementation:**

```java
// In src/test/java/frc/robot/:
import org.junit.jupiter.api.Test;
import static org.junit.jupiter.api.Assertions.*;

public class DrivetrainTest {
    @Test
    public void testModuleOptimization() {
        // Test that module state optimization works correctly
        SwerveModuleState desiredState = new SwerveModuleState(
            1.0, Rotation2d.fromDegrees(270)
        );
        SwerveModuleState currentState = new SwerveModuleState(
            0.0, Rotation2d.fromDegrees(45)
        );

        SwerveModuleState optimized =
            SwerveModuleState.optimize(desiredState, currentState);

        // Should reverse speed and rotate <90°
        assertTrue(optimized.speedMetersPerSecond < 0);
        double angleDiff = Math.abs(
            optimized.angle.minus(currentState.angle).getDegrees()
        );
        assertTrue(angleDiff < 90);
    }

    @Test
    public void testPoseCalculation() {
        // Test odometry calculations
        // ...
    }
}
```

---

## Competition-Ready Features

### Feature: Match Data Logging

**Implementation:**

```java
public class MatchLogger {
    private DataLog log;
    private IntegerLogEntry matchNumberEntry;
    private StringLogEntry eventNameEntry;

    public void initialize() {
        log = DataLogManager.getLog();
        matchNumberEntry = new IntegerLogEntry(log, "/match/number");
        eventNameEntry = new StringLogEntry(log, "/match/event");
    }

    @Override
    public void autonomousInit() {
        // Log match info
        matchNumberEntry.append(DriverStation.getMatchNumber());
        eventNameEntry.append(DriverStation.getEventName());

        // Log starting configuration
        log.log("/match/startPose", drivetrain.getPose().toString());
        log.log("/match/selectedAuto", autoChooser.getSelected().getName());
    }
}
```

### Feature: Driver Feedback System

**Implementation:**

```java
// Rumble controller based on robot state:
public void periodic() {
    // Rumble when game piece acquired
    if (hasGamePiece() && !previousHadGamePiece) {
        controller.setRumble(RumbleType.kBothRumble, 1.0);
        new Thread(() -> {
            try {
                Thread.sleep(500);
                controller.setRumble(RumbleType.kBothRumble, 0);
            } catch (InterruptedException e) {}
        }).start();
    }

    // Visual feedback via LEDs
    if (readyToScore()) {
        leds.setColor(Color.kGreen);
    } else if (hasGamePiece()) {
        leds.setColor(Color.kYellow);
    } else {
        leds.setColor(Color.kRed);
    }
}
```

### Feature: Automated Pre-Match Checks

**Implementation:**

```java
public class PreMatchCheck extends SubsystemBase {
    private boolean allChecksPass = false;

    public boolean runChecks() {
        List<String> failures = new ArrayList<>();

        // Check all devices present
        if (!drivetrain.verifyDevices()) {
            failures.add("Drivetrain devices missing");
        }

        // Check battery voltage
        if (RobotController.getBatteryVoltage() < 12.0) {
            failures.add("Battery voltage low: " +
                RobotController.getBatteryVoltage() + "V");
        }

        // Check vision system
        if (!vision.hasTargets()) {
            failures.add("No AprilTags visible");
        }

        // Report results
        if (failures.isEmpty()) {
            System.out.println("✓ All pre-match checks passed");
            allChecksPass = true;
        } else {
            System.err.println("✗ Pre-match check failures:");
            failures.forEach(f -> System.err.println("  - " + f));
            allChecksPass = false;
        }

        return allChecksPass;
    }
}
```

---

## Next-Level Capabilities

### Capability 1: Dynamic Obstacle Avoidance

**Implementation with PathPlanner:**

```java
// Define obstacle regions
List<Translation2d> obstaclePoints = List.of(
    new Translation2d(3.0, 2.0),
    new Translation2d(3.5, 2.5)
);

// PathPlanner can avoid these during pathfinding
AutoBuilder.pathfindToPose(
    targetPose,
    constraints,
    0.0
    // Obstacles handled automatically by PathPlanner
);
```

### Capability 2: Auto-Alignment to Scoring Positions

**Implementation:**

```java
public Command autoAlignToScore() {
    return Commands.sequence(
        // Use vision to find scoring target
        Commands.runOnce(() -> {
            Pose2d scoringPose = vision.getNearestScoringPose();
            drivetrain.setTargetPose(scoringPose);
        }),

        // Path to scoring position
        AutoBuilder.pathfindToPose(
            drivetrain.getTargetPose(),
            new PathConstraints(2.0, 1.5, 4.0, 3.0),
            0.0
        ),

        // Fine-tune with vision
        drivetrain.applyRequest(() ->
            driveToAngle
                .withVelocityX(0)
                .withVelocityY(0)
                .withTargetDirection(vision.getTargetHeading())
        ).until(() -> vision.isAligned()).withTimeout(2.0)
    );
}
```

### Capability 3: Predictive Target Tracking

**Track moving targets (e.g., moving opponent robot):**

```java
public Command trackMovingTarget() {
    return drivetrain.run(() -> {
        // Get target position and velocity
        Translation2d targetPos = getTargetPosition();
        Translation2d targetVel = getTargetVelocity();

        // Predict where target will be
        double timeToIntercept = calculateInterceptTime(targetPos, targetVel);
        Translation2d predictedPos = targetPos.plus(
            targetVel.times(timeToIntercept)
        );

        // Aim at predicted position
        Rotation2d aimDirection = calculateHeading(
            drivetrain.getPose().getTranslation(),
            predictedPos
        );

        drivetrain.setControl(
            driveToAngle
                .withVelocityX(-controller.getLeftY() * MaxSpeed)
                .withVelocityY(-controller.getLeftX() * MaxSpeed)
                .withTargetDirection(aimDirection)
        );
    });
}
```

### Capability 4: Field-Relative Autonomous Corrections

**Adjust autonomous on-the-fly based on field position:**

```java
@Override
public void autonomousPeriodic() {
    // Check if we're off-course
    Pose2d currentPose = drivetrain.getPose();
    Pose2d expectedPose = currentAutoPath.getExpectedPose(Timer.getFPGATimestamp());

    double error = currentPose.getTranslation()
        .getDistance(expectedPose.getTranslation());

    if (error > 0.5) {  // More than 0.5m off
        // Re-plan path from current position
        PathPlannerPath correctionPath = generatePathFromCurrent();
        AutoBuilder.followPath(correctionPath).schedule();
    }
}
```

---

## Prioritized Implementation Roadmap

### Phase 1: Foundation (Weeks 1-2)

```
□ Ensure swerve drive tuned and reliable
□ Implement basic autonomous (mobility, score preload)
□ Add Field2d visualization
□ Set up logging infrastructure
```

### Phase 2: Vision Integration (Weeks 3-4)

```
□ Install and configure PhotonVision
□ Calibrate camera
□ Integrate vision pose estimation
□ Test accuracy and tune standard deviations
```

### Phase 3: Advanced Control (Weeks 5-6)

```
□ Implement heading lock / facing angle
□ Add snap-to-angle functionality
□ Create speed mode system
□ Path following during teleop
```

### Phase 4: Competition Features (Weeks 7-8)

```
□ Build comprehensive autonomous routines
□ Implement driver feedback (rumble, LEDs)
□ Create pre-match check system
□ Optimize logging for competition
```

### Phase 5: Optimization (Ongoing)

```
□ Consider CANivore + Pro licenses
□ Fine-tune PID gains
□ Reduce CAN bus utilization
□ Profile and optimize CPU usage
```

### Phase 6: Advanced Capabilities (If Time Allows)

```
□ Auto-alignment to scoring
□ Dynamic obstacle avoidance
□ Predictive target tracking
□ Machine learning integration (?)
```
