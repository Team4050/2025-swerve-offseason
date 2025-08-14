package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class DriveDouble extends SubsystemBase {
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;

  // Kinematics for two modules (side by side)
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  public DriveDouble() {
    // Initialize modules
    frontLeftModule =
        new SwerveModule(
            "FL",
            Constants.Drivetrain.FL_DRIVE_ID,
            Constants.Drivetrain.FL_STEER_ID,
            Constants.Drivetrain.FL_ANGLE_ID,
            Constants.Drivetrain.FL_OFFSET);

    frontRightModule =
        new SwerveModule(
            "FR",
            Constants.Drivetrain.FR_DRIVE_ID,
            Constants.Drivetrain.FR_STEER_ID,
            Constants.Drivetrain.FR_ANGLE_ID,
            Constants.Drivetrain.FR_OFFSET);

    // Setup kinematics for two modules
    kinematics =
        new SwerveDriveKinematics(
            Constants.Drivetrain.FL_POSITION, Constants.Drivetrain.FR_POSITION);

    // Initialize odometry
    odometry =
        new SwerveDriveOdometry(
            kinematics,
            new Rotation2d(), // TODO: Replace with gyro when available
            getModulePositions());

    // Reset steer encoders to absolute positions on startup
    resetSteerEncoders();
  }

  @Override
  public void periodic() {
    // Update odometry
    odometry.update(
        new Rotation2d(), // TODO: Replace with gyro when available
        getModulePositions());

    // Update telemetry
    frontLeftModule.updateTelemetry();
    frontRightModule.updateTelemetry();
    updateDashboard();
  }

  /**
   * Drive the robot with velocity and rotation commands
   *
   * @param xVelocity Forward/backward velocity in m/s
   * @param yVelocity Left/right velocity in m/s
   * @param rotation Rotational velocity in rad/s
   * @param fieldOriented Whether to use field-oriented control
   */
  public void drive(double xVelocity, double yVelocity, double rotation, boolean fieldOriented) {
    // Create chassis speeds
    ChassisSpeeds chassisSpeeds;
    if (fieldOriented) {
      chassisSpeeds =
          ChassisSpeeds.fromFieldRelativeSpeeds(
              xVelocity, yVelocity, rotation, odometry.getPoseMeters().getRotation());
    } else {
      chassisSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotation);
    }

    // Convert to module states
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(chassisSpeeds);

    // Desaturate wheel speeds
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, Constants.Drivetrain.MAX_SPEED_MPS);

    // Set module states
    setModuleStates(moduleStates);
  }

  /**
   * Set the states of the swerve modules
   *
   * @param moduleStates Array of desired module states [FL, FR]
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    if (moduleStates.length != 2) {
      throw new IllegalArgumentException("Expected 2 module states for DriveDouble");
    }

    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
  }

  /**
   * Get the current module states
   *
   * @return Array of current module states [FL, FR]
   */
  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[] {frontLeftModule.getState(), frontRightModule.getState()};
  }

  /**
   * Get the current module positions
   *
   * @return Array of current module positions [FL, FR]
   */
  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
      frontLeftModule.getPosition(), frontRightModule.getPosition()
    };
  }

  /**
   * Test individual modules
   *
   * @param leftDrivePercent Left drive motor percent output
   * @param leftSteerPercent Left steer motor percent output
   * @param rightDrivePercent Right drive motor percent output
   * @param rightSteerPercent Right steer motor percent output
   */
  public void testMotors(
      double leftDrivePercent,
      double leftSteerPercent,
      double rightDrivePercent,
      double rightSteerPercent) {
    frontLeftModule.setDrivePercentOutput(leftDrivePercent);
    frontLeftModule.setSteerPercentOutput(leftSteerPercent);
    frontRightModule.setDrivePercentOutput(rightDrivePercent);
    frontRightModule.setSteerPercentOutput(rightSteerPercent);
  }

  /** Stop all modules */
  public void stop() {
    frontLeftModule.stop();
    frontRightModule.stop();
  }

  /** Reset all steer encoders to absolute positions */
  public void resetSteerEncoders() {
    frontLeftModule.resetSteerEncoder();
    frontRightModule.resetSteerEncoder();
  }

  /**
   * Reset the robot pose
   *
   * @param pose New pose to set
   */
  public void resetPose(Pose2d pose) {
    odometry.resetPosition(
        new Rotation2d(), // TODO: Replace with gyro when available
        getModulePositions(),
        pose);
  }

  /** Reset the robot pose to origin */
  public void resetPose() {
    resetPose(new Pose2d());
  }

  /**
   * Get the current robot pose
   *
   * @return Current robot pose
   */
  public Pose2d getRobotPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Get raw absolute encoder readings for calibration
   *
   * @return Array of raw encoder readings [FL, FR] in degrees
   */
  public double[] getRawAbsoluteEncoders() {
    return new double[] {
      frontLeftModule.getRawAbsoluteEncoderDegrees(),
      frontRightModule.getRawAbsoluteEncoderDegrees()
    };
  }

  /** Update dashboard with useful information */
  private void updateDashboard() {
    Pose2d pose = getRobotPose();
    SmartDashboard.putNumber("Robot X", pose.getX());
    SmartDashboard.putNumber("Robot Y", pose.getY());
    SmartDashboard.putNumber("Robot Rotation (deg)", pose.getRotation().getDegrees());

    SwerveModuleState[] states = getModuleStates();
    SmartDashboard.putNumber("FL Speed (m/s)", states[0].speedMetersPerSecond);
    SmartDashboard.putNumber("FL Angle (deg)", states[0].angle.getDegrees());
    SmartDashboard.putNumber("FR Speed (m/s)", states[1].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Angle (deg)", states[1].angle.getDegrees());

    double[] rawEncoders = getRawAbsoluteEncoders();
    SmartDashboard.putNumber("FL Raw Encoder (deg)", rawEncoders[0]);
    SmartDashboard.putNumber("FR Raw Encoder (deg)", rawEncoders[1]);
  }
}
