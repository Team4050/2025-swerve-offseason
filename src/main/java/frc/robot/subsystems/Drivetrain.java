package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.function.Consumer;

public class Drivetrain extends SubsystemBase {
  private final SwerveModule[] modules;

  private SwerveModulePosition[] modulePositions;
  private double[] rawEncodersDegrees;

  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;

  public Drivetrain(SwerveModule single) {
    modules = new SwerveModule[] {single};
    modulePositions = new SwerveModulePosition[modules.length];

    kinematics = new SwerveDriveKinematics(Constants.Drivetrain.FL_POSITION);
    odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());

    resetSteerEncoders();
  }

  public Drivetrain(SwerveModule left, SwerveModule right) {
    modules = new SwerveModule[] {left, right};
    modulePositions = new SwerveModulePosition[modules.length];

    kinematics =
        new SwerveDriveKinematics(
            Constants.Drivetrain.FL_POSITION, Constants.Drivetrain.FR_POSITION);
    odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());

    resetSteerEncoders();
  }

  public Drivetrain(
      SwerveModule frontLeft,
      SwerveModule frontRight,
      SwerveModule backLeft,
      SwerveModule backRight) {
    modules = new SwerveModule[] {frontLeft, frontRight, backLeft, backRight};
    modulePositions = new SwerveModulePosition[modules.length];

    kinematics =
        new SwerveDriveKinematics(
            Constants.Drivetrain.FL_POSITION,
            Constants.Drivetrain.FR_POSITION,
            Constants.Drivetrain.BL_POSITION,
            Constants.Drivetrain.BR_POSITION);
    odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());

    resetSteerEncoders();
  }

  public void drive(
      double xVelocity, double yVelocity, double rotationalVelocity, boolean fieldOriented) {

    ChassisSpeeds chassisSpeeds =
        fieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                xVelocity, yVelocity, rotationalVelocity, odometry.getPoseMeters().getRotation())
            : new ChassisSpeeds(xVelocity, yVelocity, rotationalVelocity);

    setModuleStates(kinematics.toSwerveModuleStates(chassisSpeeds));
  }

  public void setModuleStates(SwerveModuleState[] states) {
    if (states.length != modules.length) {
      throw new IllegalArgumentException("Expected number of states to match number of modules");
    }
    for (int i = 0; i < modules.length; i++) {
      modules[i].setDesiredState(states[i]);
    }
  }

  @Override
  public void periodic() {
    odometry.update(getGyroAngle(), getModulePositions());
  }

  public SwerveModulePosition[] getModulePositions() {
    for (int i = 0; i < modules.length; i++) {
      modulePositions[i] = modules[i].getPosition();
    }
    return modulePositions;
  }

  public void resetSteerEncoders() {
    forEachModule(SwerveModule::resetSteerEncoder);
  }

  public void stop() {
    forEachModule(SwerveModule::stop);
  }

  public void resetPose() {
    resetPose(new Pose2d());
  }

  public void resetPose(Pose2d pose) {
    odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
  }

  public Rotation2d getGyroAngle() {
    // TODO: Replace with gyro when available
    return new Rotation2d();
  }

  public void forEachModule(Consumer<SwerveModule> moduleCallback) {
    for (SwerveModule module : modules) {
      moduleCallback.accept(module);
    }
  }

  public double[] getRawAbsoluteEncoders() {
    rawEncodersDegrees = new double[modules.length];
    for (int i = 0; i < modules.length; i++) {
      rawEncodersDegrees[i] = modules[i].getRawAbsoluteEncoderDegrees();
    }
    return rawEncodersDegrees;
  }
}
