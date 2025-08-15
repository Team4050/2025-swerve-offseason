package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import java.util.List;

/**
 * A flexible swerve drivetrain subsystem that supports multiple configurations.
 *
 * <p>This drivetrain can be configured with 1, 2, or 4 swerve modules depending on the robot's
 * needs. It handles kinematics calculations, odometry tracking, and provides methods for
 * controlling individual modules.
 *
 * <p>Supported configurations:
 *
 * <ul>
 *   <li>Single module - for testing and prototyping
 *   <li>Double module - simplified two-wheel swerve drive
 *   <li>Quad module - standard four-wheel swerve drive
 * </ul>
 */
public class Drivetrain extends SubsystemBase {
    private final List<SwerveModule> modules;
    private final SwerveDriveKinematics kinematics;
    private final SwerveDriveOdometry odometry;

    /**
     * Creates a single-module drivetrain configuration for testing purposes.
     *
     * @param module the single swerve module to use
     * @return a new Drivetrain instance with one module
     */
    public static Drivetrain createSingle(SwerveModule module) {
        return new Drivetrain(List.of(module), DriveConfig.SINGLE);
    }

    /**
     * Creates a two-module drivetrain configuration.
     *
     * @param left the left swerve module
     * @param right the right swerve module
     * @return a new Drivetrain instance with two modules
     */
    public static Drivetrain createDouble(SwerveModule left, SwerveModule right) {
        return new Drivetrain(List.of(left, right), DriveConfig.DOUBLE);
    }

    /**
     * Creates a standard four-module swerve drivetrain configuration.
     *
     * @param frontLeft the front left swerve module
     * @param frontRight the front right swerve module
     * @param backLeft the back left swerve module
     * @param backRight the back right swerve module
     * @return a new Drivetrain instance with four modules
     */
    public static Drivetrain createQuad(
            SwerveModule frontLeft,
            SwerveModule frontRight,
            SwerveModule backLeft,
            SwerveModule backRight) {
        return new Drivetrain(
                List.of(frontLeft, frontRight, backLeft, backRight), DriveConfig.QUAD);
    }

    /**
     * Constructs a new Drivetrain with the specified modules and configuration.
     *
     * <p>This constructor validates that the number of provided modules matches the expected count
     * for the given configuration, initializes the kinematics and odometry systems, and resets all
     * module steer encoders.
     *
     * @param modules the list of swerve modules to use
     * @param config the drive configuration that defines module positions
     * @throws IllegalArgumentException if the number of modules doesn't match the configuration
     */
    private Drivetrain(List<SwerveModule> modules, DriveConfig config) {
        if (modules.size() != config.getPositions().length) {
            throw new IllegalArgumentException("Module count doesn't match drive config");
        }

        this.modules = List.copyOf(modules);
        this.kinematics = new SwerveDriveKinematics(config.getPositions());
        this.odometry = new SwerveDriveOdometry(kinematics, getGyroAngle(), getModulePositions());

        resetModuleSteerEncoders();
    }

    /* Drivetrain methods */

    public void drive(double xVelocity, double yVelocity, double rVelocity, boolean fieldOriented) {
        ChassisSpeeds speeds;

        if (fieldOriented) {
            speeds =
                    ChassisSpeeds.fromFieldRelativeSpeeds(
                            xVelocity,
                            yVelocity,
                            rVelocity,
                            odometry.getPoseMeters().getRotation());
        } else {
            speeds = new ChassisSpeeds(xVelocity, yVelocity, rVelocity);
        }

        setModuleStates(kinematics.toSwerveModuleStates(speeds));
    }

    public void stop() {
        modules.forEach(SwerveModule::stop);
    }

    public Rotation2d getGyroAngle() {
        // TODO: Gyro
        return new Rotation2d();
    }

    public void resetPose() {
        resetPose(new Pose2d());
    }

    public void resetPose(Pose2d pose) {
        odometry.resetPosition(getGyroAngle(), getModulePositions(), pose);
    }

    public void setSteeringCoast(boolean coast) {
        modules.forEach(
                module ->
                        module.setSteerNeutralMode(
                                coast ? NeutralModeValue.Coast : NeutralModeValue.Brake));
    }

    public Double[] getRawAbsoluteEncoderDegrees() {
        return modules.stream()
                .map(SwerveModule::getRawAbsoluteEncoderDegrees)
                .toArray(Double[]::new);
    }

    public void setModuleStates(SwerveModuleState[] states) {
        if (states.length != modules.size()) {
            throw new IllegalArgumentException(
                    "Expected " + modules.size() + " states, got " + states.length);
        }

        for (int i = 0; i < modules.size(); i++) {
            modules.get(i).setDesiredState(states[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return modules.stream().map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
    }

    public void resetModuleSteerEncoders() {
        modules.forEach(SwerveModule::resetSteerEncoder);
    }

    /**
     * Defines the different drive configurations supported by the drivetrain. Each configuration
     * specifies the number and positions of swerve modules.
     *
     * <p>Supported configurations:
     *
     * <ul>
     *   <li>{@link #SINGLE} - Single module configuration for testing
     *   <li>{@link #DOUBLE} - Two module configuration (front left and front right)
     *   <li>{@link #QUAD} - Standard four module swerve drive configuration
     * </ul>
     */
    private enum DriveConfig {
        SINGLE(new Translation2d[] {Constants.Drivetrain.FL_POSITION}),
        DOUBLE(
                new Translation2d[] {
                    Constants.Drivetrain.FL_POSITION, Constants.Drivetrain.FR_POSITION
                }),
        QUAD(
                new Translation2d[] {
                    Constants.Drivetrain.FL_POSITION, Constants.Drivetrain.FR_POSITION,
                    Constants.Drivetrain.BL_POSITION, Constants.Drivetrain.BR_POSITION
                });

        private final Translation2d[] positions;

        private DriveConfig(Translation2d[] positions) {
            this.positions = positions;
        }

        private Translation2d[] getPositions() {
            return positions.clone();
        }
    }
}
