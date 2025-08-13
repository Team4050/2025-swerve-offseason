package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SingleModuleConstants;

/**
 * Single swerve module drivetrain for testing MK4i L2 module. Uses Kraken X60 motors and CANcoder
 * with SDS MK4i L2 gearing.
 *
 * <p>This class allows testing a single swerve module with Xbox controller input. Tune the
 * constants as needed for your specific module setup.
 */
public class DrivetrainSingle extends SubsystemBase {

  // Calculate max theoretical wheel speed
  private static final double WHEEL_CIRCUMFERENCE =
      Math.PI * SingleModuleConstants.WHEEL_DIAMETER_M;
  private static final double THEORETICAL_MAX_SPEED_MPS =
      (SingleModuleConstants.MOTOR_FREE_RPM / 60.0)
          * WHEEL_CIRCUMFERENCE
          / SingleModuleConstants.DRIVE_GEAR_RATIO;

  // Configuration constants - now using shared constants file
  // Hardware
  private final TalonFX driveMotor = new TalonFX(SingleModuleConstants.DRIVE_MOTOR_ID);
  private final TalonFX steerMotor = new TalonFX(SingleModuleConstants.STEER_MOTOR_ID);
  private final CANcoder steerEncoder = new CANcoder(SingleModuleConstants.CANCODER_ID);

  // Control objects
  private final DutyCycleOut driveOutput = new DutyCycleOut(0);
  private final PositionVoltage steerOutput = new PositionVoltage(0).withEnableFOC(true);

  // Conversion factors
  private final double steerRotationsPerRad =
      SingleModuleConstants.STEER_GEAR_RATIO / (2.0 * Math.PI);
  private final double driveRotationsPerMeter =
      SingleModuleConstants.DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE;

  // State tracking
  private boolean isEnabled = true;
  private double targetSpeed = 0.0;
  private double targetAngle = 0.0;

  public DrivetrainSingle() {
    configureMotors();
    syncSteerMotorToAbsolute();

    // Set initial SmartDashboard values
    SmartDashboard.putNumber("Single/Target Speed (m/s)", 0.0);
    SmartDashboard.putNumber("Single/Target Angle (deg)", 0.0);
    SmartDashboard.putBoolean("Single/Module Enabled", true);
  }

  /**
   * Drive the module with direct speed and angle control.
   *
   * @param speedMps Drive speed in meters per second
   * @param angleDeg Target angle in degrees (0 = forward)
   */
  public void drive(double speedMps, double angleDeg) {
    if (!isEnabled) {
      stop();
      return;
    }

    // Store targets for telemetry
    targetSpeed = speedMps;
    targetAngle = angleDeg;

    // Limit speed for safety
    speedMps =
        Math.max(
            -SingleModuleConstants.MAX_DRIVE_SPEED_MPS,
            Math.min(SingleModuleConstants.MAX_DRIVE_SPEED_MPS, speedMps));

    // Create desired state
    SwerveModuleState desiredState =
        new SwerveModuleState(speedMps, Rotation2d.fromDegrees(angleDeg));

    // Set the module to this state
    setModuleState(desiredState);
  }

  /**
   * Drive using SwerveModuleState (for compatibility with swerve libraries).
   *
   * @param desiredState The desired module state
   */
  public void setModuleState(SwerveModuleState desiredState) {
    if (!isEnabled) {
      stop();
      return;
    }

    // Get current absolute angle from CANcoder
    double currentAngleRad = getCurrentAngleRad();

    // Simple optimization: if we need to turn more than 90 degrees,
    // reverse the wheel direction and turn to the opposite angle
    double targetAngleRad = desiredState.angle.getRadians();
    double speedMps = desiredState.speedMetersPerSecond;

    double angleDifference = Math.abs(targetAngleRad - currentAngleRad);
    if (angleDifference > Math.PI) {
      angleDifference = 2 * Math.PI - angleDifference;
    }

    if (angleDifference > Math.PI / 2) {
      // Reverse direction and use opposite angle
      targetAngleRad += Math.PI;
      speedMps = -speedMps;
    }

    // Normalize target angle to [0, 2π)
    while (targetAngleRad < 0) targetAngleRad += 2 * Math.PI;
    while (targetAngleRad >= 2 * Math.PI) targetAngleRad -= 2 * Math.PI;

    // Set drive speed (as percentage of max speed)
    double driveOutput = speedMps / THEORETICAL_MAX_SPEED_MPS;
    driveOutput = Math.max(-1.0, Math.min(1.0, driveOutput)); // Clamp to [-1, 1]
    driveMotor.setControl(this.driveOutput.withOutput(driveOutput));

    // Set steer angle using shortest path
    setSteerAngle(targetAngleRad);
  }

  /** Stop the module (zero drive speed, hold current steer position). */
  public void stop() {
    driveMotor.setControl(driveOutput.withOutput(0));
    // Hold current steer position
    double currentPosition = steerMotor.getPosition().getValueAsDouble();
    steerMotor.setControl(steerOutput.withPosition(currentPosition));
  }

  /**
   * Enable/disable the module.
   *
   * @param enabled Whether to enable the module
   */
  public void setEnabled(boolean enabled) {
    isEnabled = enabled;
    if (!enabled) {
      stop();
    }
  }

  /**
   * Check if the module is enabled.
   *
   * @return True if the module is enabled
   */
  public boolean isEnabled() {
    return isEnabled;
  }

  /**
   * Get the current angle of the module from the CANcoder.
   *
   * @return Current angle in radians
   */
  public double getCurrentAngleRad() {
    return (steerEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI)
        - SingleModuleConstants.ANGLE_OFFSET_RAD;
  }

  /**
   * Get the current angle of the module in degrees.
   *
   * @return Current angle in degrees
   */
  public double getCurrentAngleDeg() {
    return Math.toDegrees(getCurrentAngleRad());
  }

  /**
   * Get the current drive velocity in m/s.
   *
   * @return Drive velocity in meters per second
   */
  public double getDriveVelocityMps() {
    return driveMotor.getVelocity().getValueAsDouble() / driveRotationsPerMeter;
  }

  /**
   * Get the current module state.
   *
   * @return Current SwerveModuleState
   */
  public SwerveModuleState getCurrentState() {
    return new SwerveModuleState(getDriveVelocityMps(), new Rotation2d(getCurrentAngleRad()));
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with current values
    SmartDashboard.putNumber("Single/Current Angle (deg)", getCurrentAngleDeg());
    SmartDashboard.putNumber("Single/Current Speed (m/s)", getDriveVelocityMps());
    SmartDashboard.putNumber(
        "Single/Drive Motor Temp (C)", driveMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "Single/Steer Motor Temp (C)", steerMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(
        "Single/CANcoder Angle (deg)",
        steerEncoder.getAbsolutePosition().getValueAsDouble() * 360.0);

    // Update targets from SmartDashboard (for tuning)
    SmartDashboard.putNumber("Single/Target Speed (m/s)", targetSpeed);
    SmartDashboard.putNumber("Single/Target Angle (deg)", targetAngle);

    // Allow enable/disable from dashboard
    boolean dashboardEnabled = SmartDashboard.getBoolean("Single/Module Enabled", true);
    if (dashboardEnabled != isEnabled) {
      setEnabled(dashboardEnabled);
    }
  }

  /**
   * Set the steer motor to the target angle using shortest path.
   *
   * @param targetAngleRad Target angle in radians
   */
  private void setSteerAngle(double targetAngleRad) {
    // Get current motor position
    double currentMotorPosition = steerMotor.getPosition().getValueAsDouble();

    // Get current absolute angle
    double currentAngleRad = getCurrentAngleRad();

    // Calculate the shortest angular distance
    double deltaRad = targetAngleRad - currentAngleRad;

    // Wrap delta to [-π, π] for shortest path
    while (deltaRad > Math.PI) deltaRad -= 2.0 * Math.PI;
    while (deltaRad < -Math.PI) deltaRad += 2.0 * Math.PI;

    // Calculate target motor position
    double targetMotorPosition = currentMotorPosition + (deltaRad * steerRotationsPerRad);

    // Set the steer motor position
    steerMotor.setControl(steerOutput.withPosition(targetMotorPosition));
  }

  /** Configure motor controllers with appropriate settings. */
  private void configureMotors() {
    // Configure drive motor
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    driveConfig.MotorOutput.Inverted =
        SingleModuleConstants.DRIVE_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Apply drive configuration
    driveMotor.getConfigurator().apply(driveConfig);

    // Configure steer motor
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted =
        SingleModuleConstants.STEER_MOTOR_INVERTED
            ? InvertedValue.Clockwise_Positive
            : InvertedValue.CounterClockwise_Positive;

    // Configure PID gains for steer control
    steerConfig.Slot0.kP = SingleModuleConstants.STEER_KP;
    steerConfig.Slot0.kI = SingleModuleConstants.STEER_KI;
    steerConfig.Slot0.kD = SingleModuleConstants.STEER_KD;

    // Apply steer configuration
    steerMotor.getConfigurator().apply(steerConfig);

    // Configure CANcoder
    CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
    encoderConfig.MagnetSensor.SensorDirection =
        SingleModuleConstants.CANCODER_INVERTED
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;

    // Apply encoder configuration
    steerEncoder.getConfigurator().apply(encoderConfig);
  }

  /**
   * Sync the steer motor's internal position to match the absolute encoder. Call this on startup to
   * ensure accurate position control.
   */
  private void syncSteerMotorToAbsolute() {
    double absoluteAngleRad = getCurrentAngleRad();
    steerMotor.setPosition(absoluteAngleRad * steerRotationsPerRad);
  }

  /**
   * Reset the angle offset to the current CANcoder position. Use this to calibrate the module's
   * "zero" position.
   */
  public void calibrateAngleOffset() {
    // This would typically write to a config file or persistent storage
    // For now, just log the current position
    double currentPosition = steerEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI;
    System.out.println(
        "Current CANcoder position for calibration: "
            + Math.toDegrees(currentPosition)
            + " degrees");
    System.out.println("Set ANGLE_OFFSET_RAD to: " + currentPosition);
  }
}
