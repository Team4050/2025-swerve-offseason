package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Simple two-module swerve drivetrain for testbench. Uses Kraken X60 motors and CANcoders with SDS
 * MK4i L2 modules.
 */
public class Drivetrain2 extends SubsystemBase {

  // Configuration constants
  private static final class C {
    // CAN IDs - adjust for your setup
    static final int LEFT_DRIVE_ID = 1;
    static final int LEFT_STEER_ID = 2;
    static final int LEFT_CANCODER_ID = 11;
    static final int RIGHT_DRIVE_ID = 3;
    static final int RIGHT_STEER_ID = 4;
    static final int RIGHT_CANCODER_ID = 12;

    // Hardware specs
    static final double TRACK_WIDTH_M = Units.inchesToMeters(24.0); // More realistic for testbench
    static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0); // Correct for MK4i
    static final double DRIVE_GEAR_RATIO = 6.75; // MK4i L2
    static final double STEER_GEAR_RATIO = 150.0 / 7.0; // MK4i steer ratio
    static final double MOTOR_FREE_RPM = 6000.0; // Kraken X60

    // Steer PID gains - tune these for your modules
    static final double STEER_KP = 20.0; // Proportional gain
    static final double STEER_KI = 0.0; // Integral gain (start at 0)
    static final double STEER_KD = 0.1; // Derivative gain (small value to reduce oscillation)

    // Module angle offsets (set these after measuring your modules)
    static final double LEFT_ANGLE_OFFSET_RAD = 0.0;
    static final double RIGHT_ANGLE_OFFSET_RAD = 0.0;
  }

  // Calculate max wheel speed
  private static final double WHEEL_CIRCUMFERENCE = Math.PI * C.WHEEL_DIAMETER_M;
  private static final double MAX_WHEEL_SPEED_MPS =
      (C.MOTOR_FREE_RPM / 60.0) * WHEEL_CIRCUMFERENCE / C.DRIVE_GEAR_RATIO;

  // Kinematics setup for two modules
  private final Translation2d leftPos = new Translation2d(0.0, C.TRACK_WIDTH_M / 2.0);
  private final Translation2d rightPos = new Translation2d(0.0, -C.TRACK_WIDTH_M / 2.0);
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(leftPos, rightPos);

  // Hardware - direct motor/encoder control (no custom module class needed!)
  private final TalonFX leftDrive = new TalonFX(C.LEFT_DRIVE_ID);
  private final TalonFX leftSteer = new TalonFX(C.LEFT_STEER_ID);
  private final CANcoder leftEncoder = new CANcoder(C.LEFT_CANCODER_ID);
  private final TalonFX rightDrive = new TalonFX(C.RIGHT_DRIVE_ID);
  private final TalonFX rightSteer = new TalonFX(C.RIGHT_STEER_ID);
  private final CANcoder rightEncoder = new CANcoder(C.RIGHT_CANCODER_ID);

  // Control objects
  private final DutyCycleOut driveOutput = new DutyCycleOut(0);
  private final PositionVoltage steerOutput = new PositionVoltage(0).withEnableFOC(true);

  // Conversion factors
  private final double steerRotationsPerRad = C.STEER_GEAR_RATIO / (2.0 * Math.PI);

  private boolean leftDisabled = false;
  private boolean rightDisabled = false;

  public Drivetrain2() {
    configureMotors();
    syncSteerMotorsToAbsolute();
  }

  /**
   * Drive the robot with speed and rotation inputs.
   *
   * @param speed Forward speed in meters per second (+forward)
   * @param rotation Rotation rate in radians per second (+CCW)
   */
  public void drive(double speed, double rotation) {
    ChassisSpeeds chassisSpeeds = new ChassisSpeeds(speed, 0.0, rotation);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Limit speeds to achievable values
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, MAX_WHEEL_SPEED_MPS);

    setModuleState(leftDrive, leftSteer, leftEncoder, moduleStates[0], C.LEFT_ANGLE_OFFSET_RAD);
    setModuleState(rightDrive, rightSteer, rightEncoder, moduleStates[1], C.RIGHT_ANGLE_OFFSET_RAD);
  }

  public void stop() {
    leftDrive.setControl(driveOutput.withOutput(0));
    rightDrive.setControl(driveOutput.withOutput(0));
    // Hold current steer positions
    leftSteer.setControl(steerOutput.withPosition(leftSteer.getPosition().getValueAsDouble()));
    rightSteer.setControl(steerOutput.withPosition(rightSteer.getPosition().getValueAsDouble()));
  }

  /** Set a module to the desired state - simple direct control */
  private void setModuleState(
      TalonFX driveMotor,
      TalonFX steerMotor,
      CANcoder encoder,
      SwerveModuleState desiredState,
      double angleOffsetRad) {
    // Get current angle from CANcoder
    double currentAngleRad =
        (encoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI) - angleOffsetRad;

    // Since modules can spin 360°, we don't need optimization - just go to the target angle
    double targetAngleRad = desiredState.angle.getRadians();

    // Set drive speed
    double driveOutput = desiredState.speedMetersPerSecond / MAX_WHEEL_SPEED_MPS;
    driveOutput = Math.max(-1.0, Math.min(1.0, driveOutput));
    driveMotor.setControl(this.driveOutput.withOutput(driveOutput));

    // Set steer angle - calculate shortest path
    double currentMotorPosition = steerMotor.getPosition().getValueAsDouble();
    double deltaRad = targetAngleRad - currentAngleRad;

    // Wrap delta to [-π, π] for shortest path
    while (deltaRad > Math.PI) deltaRad -= 2.0 * Math.PI;
    while (deltaRad < -Math.PI) deltaRad += 2.0 * Math.PI;

    double targetMotorPosition = currentMotorPosition + (deltaRad * steerRotationsPerRad);
    steerMotor.setControl(steerOutput.withPosition(targetMotorPosition));
  }

  private void configureMotors() {
    // Configure drive motors
    TalonFXConfiguration driveConfig = new TalonFXConfiguration();
    driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Left drive not inverted, right drive inverted (typical for differential-style layout)
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    leftDrive.getConfigurator().apply(driveConfig);
    driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightDrive.getConfigurator().apply(driveConfig);

    // Configure steer motors
    TalonFXConfiguration steerConfig = new TalonFXConfiguration();
    steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    // Configure PID gains for steer control
    steerConfig.Slot0.kP = C.STEER_KP;
    steerConfig.Slot0.kI = C.STEER_KI;
    steerConfig.Slot0.kD = C.STEER_KD;

    leftSteer.getConfigurator().apply(steerConfig);
    rightSteer.getConfigurator().apply(steerConfig);
  }

  private void syncSteerMotorsToAbsolute() {
    // Sync left steer
    double leftAbsoluteRad =
        (leftEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI)
            - C.LEFT_ANGLE_OFFSET_RAD;
    leftSteer.setPosition(leftAbsoluteRad * steerRotationsPerRad);

    // Sync right steer
    double rightAbsoluteRad =
        (rightEncoder.getAbsolutePosition().getValueAsDouble() * 2.0 * Math.PI)
            - C.RIGHT_ANGLE_OFFSET_RAD;
    rightSteer.setPosition(rightAbsoluteRad * steerRotationsPerRad);
  }
}
