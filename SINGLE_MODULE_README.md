# Single MK4i Swerve Module Test Setup

This project provides a simple, straightforward drivetrain implementation for testing a single SDS MK4i Level 2 swerve module with Kraken X60 motors and a CANcoder.

## Features

- Simple direct control of a single swerve module
- Xbox controller input for intuitive testing
- SmartDashboard telemetry for monitoring
- Easy calibration and tuning process
- Safety features and configurable speed limits

## Hardware Requirements

- 1x SDS MK4i Level 2 swerve module
- 2x Kraken X60 motors (drive and steer)
- 1x CANcoder for absolute position feedback
- Xbox controller for input
- CAN bus connection to all devices

## Quick Start

### 1. Hardware Setup

1. Install your Kraken X60 motors in the MK4i module
2. Connect all devices to the CAN bus
3. Note your CAN IDs for each device

### 2. Software Configuration

Open `Constants.java` and update the CAN IDs to match your hardware:

```java
// Update these to match your setup
public static final int DRIVE_MOTOR_ID = 1;    // Your drive motor CAN ID
public static final int STEER_MOTOR_ID = 2;    // Your steer motor CAN ID
public static final int CANCODER_ID = 11;      // Your CANcoder CAN ID
```

### 3. Motor Direction Testing

1. Deploy the code and enable the robot
2. Use the left stick Y-axis to test drive direction:
   - Forward stick should drive wheel forward
   - If backward, set `DRIVE_MOTOR_INVERTED = true` in Constants.java
3. Use the right stick X-axis to test steering:
   - Right stick should point wheel right, left stick should point wheel left
   - If wrong, set `STEER_MOTOR_INVERTED = true` in Constants.java

### 4. CANcoder Calibration

1. Manually point your module straight forward (0 degrees)
2. Press the **Y button** on the Xbox controller
3. Check the console output for a line like:
   ```
   Set ANGLE_OFFSET_RAD to: 1.2345
   ```
4. Update `ANGLE_OFFSET_RAD` in Constants.java with this value
5. Redeploy the code

### 5. Testing

Your module should now be properly calibrated! Use these controls:

- **Left Stick Y**: Drive forward/backward
- **Right Stick X**: Steer left/right (absolute positioning)
- **A Button**: Stop module
- **B Button**: Enable/disable module
- **Y Button**: Recalibrate angle offset

## SmartDashboard Monitoring

The following values are available on SmartDashboard:

- `Single/Current Angle (deg)`: Current module angle
- `Single/Current Speed (m/s)`: Current wheel speed
- `Single/Drive Motor Temp (C)`: Drive motor temperature
- `Single/Steer Motor Temp (C)`: Steer motor temperature
- `Single/CANcoder Angle (deg)`: Raw CANcoder reading
- `Single/Module Enabled`: Enable/disable the module

## Tuning

### Speed Limits

Adjust these in Constants.java for safe testing:

```java
public static final double MAX_DRIVE_SPEED_MPS = 2.0;  // Max wheel speed
public static final double DRIVE_SPEED_SCALE = 1.5;    // Controller scaling
```

### PID Tuning

If steering is not smooth, tune these PID gains in Constants.java:

```java
public static final double STEER_KP = 20.0;  // Increase for faster response
public static final double STEER_KI = 0.0;   // Add for steady-state accuracy
public static final double STEER_KD = 0.1;   // Increase to reduce oscillation
```

**Tuning Guidelines:**

- Start with KP only, increase until responsive but not oscillating
- Add small KD (0.1-1.0) if there's overshoot or oscillation
- Add small KI (0.1-0.5) only if there's steady-state error

### Control Sensitivity

Adjust these in Constants.java:

```java
public static final double STEER_ANGLE_RANGE_DEG = 90.0;  // +/- degrees at full stick
```

## Troubleshooting

### Module doesn't respond

- Check CAN IDs are correct
- Verify motors are enabled and not faulted
- Check that robot is enabled

### Wrong drive direction

- Set `DRIVE_MOTOR_INVERTED = true` in Constants.java

### Wrong steer direction

- Set `STEER_MOTOR_INVERTED = true` in Constants.java

### Steering not accurate

- Recalibrate angle offset (Y button → update Constants.java)
- Check CANcoder wiring and magnet alignment
- Try `CANCODER_INVERTED = true` if readings seem backward

### Jerky or unstable steering

- Reduce STEER_KP gain
- Increase STEER_KD gain
- Check for mechanical binding in the module

### Module overheating

- Reduce MAX_DRIVE_SPEED_MPS
- Check for mechanical resistance
- Ensure adequate cooling

## Advanced Usage

### Alternative Control Schemes

The `SingleModuleTestCommand.java` includes commented code for velocity-based steering control instead of position-based. Uncomment the alternative section to try it.

### Custom Commands

You can create custom test commands by calling:

```java
drivetrain.drive(speedMps, angleDeg);           // Simple drive command
drivetrain.setModuleState(swerveModuleState);   // SwerveModuleState command
drivetrain.getCurrentState();                   // Get current state
```

## Code Structure

- `DrivetrainSingle.java`: Main subsystem class
- `SingleModuleTestCommand.java`: Xbox controller command
- `Constants.java`: All configuration values
- `RobotContainer.java`: Subsystem and command setup

## Safety Notes

- Always start with low speed limits during initial testing
- Monitor motor temperatures during extended operation
- Ensure emergency stop is readily available
- Test in a safe area with adequate space

## Support

This code is designed to be simple and self-contained. All tuning parameters are clearly documented in the Constants.java file with setup instructions.
