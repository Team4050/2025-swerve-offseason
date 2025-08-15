package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class SwerveModule {
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder absoluteEncoder;

    private final PIDController steerPIDController;

    private final double absoluteEncoderOffset;
    private final String moduleName;

    // Control requests for Phoenix 6
    private final VelocityVoltage driveVelocityRequest = new VelocityVoltage(0);
    private final PositionVoltage steerPositionRequest = new PositionVoltage(0);

    /**
     * Creates a new SwerveModule
     *
     * @param moduleName Name for telemetry/debugging
     * @param driveMotorID CAN ID for drive motor
     * @param steerMotorID CAN ID for steer motor
     * @param absoluteEncoderID CAN ID for absolute encoder
     * @param absoluteEncoderOffset Offset in degrees for absolute encoder
     */
    public SwerveModule(
            String moduleName,
            int driveMotorID,
            int steerMotorID,
            int absoluteEncoderID,
            double absoluteEncoderOffset) {
        this.moduleName = moduleName;
        this.absoluteEncoderOffset = absoluteEncoderOffset;

        // Initialize hardware
        driveMotor = new TalonFX(driveMotorID);
        steerMotor = new TalonFX(steerMotorID);
        absoluteEncoder = new CANcoder(absoluteEncoderID);

        // Initialize PID controller for steering
        steerPIDController =
                new PIDController(
                        Constants.SwerveModule.STEER_KP,
                        Constants.SwerveModule.STEER_KI,
                        Constants.SwerveModule.STEER_KD);
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        configureMotors();
    }

    private void configureMotors() {
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();
        driveConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveModule.DRIVE_CURRENT_LIMIT;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotor.getConfigurator().apply(driveConfig);

        TalonFXConfiguration steerConfig = new TalonFXConfiguration();
        steerConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveModule.STEER_CURRENT_LIMIT;
        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
        steerMotor.getConfigurator().apply(steerConfig);

        CANcoderConfiguration encoderConfig = new CANcoderConfiguration();
        encoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        absoluteEncoder.getConfigurator().apply(encoderConfig);
    }

    public void setSteerNeutralMode(NeutralModeValue neutralMode) {
        steerMotor.setNeutralMode(neutralMode);
    }

    /**
     * Get the current state of the module
     *
     * @return Current SwerveModuleState
     */
    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocityMPS(), new Rotation2d(getSteerAngleRadians()));
    }

    /**
     * Get the current position of the module
     *
     * @return Current SwerveModulePosition
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                getDrivePositionMeters(), new Rotation2d(getSteerAngleRadians()));
    }

    /**
     * Set the desired state for the module
     *
     * @param desiredState Desired SwerveModuleState
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize the state to avoid spinning more than 90 degrees
        SwerveModuleState optimizedState =
                SwerveModuleState.optimize(desiredState, getState().angle);

        // Set drive velocity
        double driveVelocityRPS =
                optimizedState.speedMetersPerSecond
                        / (Math.PI * Constants.SwerveModule.WHEEL_DIAMETER)
                        * Constants.SwerveModule.DRIVE_GEAR_RATIO;
        driveMotor.setControl(driveVelocityRequest.withVelocity(driveVelocityRPS));

        // Set steer position
        double steerPositionRotations =
                optimizedState.angle.getRadians()
                        / (2 * Math.PI)
                        * Constants.SwerveModule.STEER_GEAR_RATIO;
        steerMotor.setControl(steerPositionRequest.withPosition(steerPositionRotations));
    }

    /** Stop the module */
    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    /**
     * Set drive motor to a specific percentage output (for testing)
     *
     * @param percentOutput Percent output (-1.0 to 1.0)
     */
    public void setDrivePercentOutput(double percentOutput) {
        driveMotor.set(percentOutput);
    }

    /**
     * Set steer motor to a specific percentage output (for testing)
     *
     * @param percentOutput Percent output (-1.0 to 1.0)
     */
    public void setSteerPercentOutput(double percentOutput) {
        steerMotor.set(percentOutput);
    }

    /** Reset the steer encoder to the absolute encoder position */
    public void resetSteerEncoder() {
        double absolutePosition = getAbsoluteEncoderRadians();
        double steerPositionRotations =
                absolutePosition / (2 * Math.PI) * Constants.SwerveModule.STEER_GEAR_RATIO;
        steerMotor.setPosition(steerPositionRotations);
    }

    /**
     * Get drive velocity in meters per second
     *
     * @return Drive velocity in m/s
     */
    private double getDriveVelocityMPS() {
        double velocityRPS = driveMotor.getVelocity().getValueAsDouble();
        return velocityRPS
                * Math.PI
                * Constants.SwerveModule.WHEEL_DIAMETER
                / Constants.SwerveModule.DRIVE_GEAR_RATIO;
    }

    /**
     * Get drive position in meters
     *
     * @return Drive position in meters
     */
    private double getDrivePositionMeters() {
        double positionRotations = driveMotor.getPosition().getValueAsDouble();
        return positionRotations
                * Math.PI
                * Constants.SwerveModule.WHEEL_DIAMETER
                / Constants.SwerveModule.DRIVE_GEAR_RATIO;
    }

    /**
     * Get steer angle in radians
     *
     * @return Steer angle in radians
     */
    private double getSteerAngleRadians() {
        double steerPositionRotations = steerMotor.getPosition().getValueAsDouble();
        return steerPositionRotations * 2 * Math.PI / Constants.SwerveModule.STEER_GEAR_RATIO;
    }

    /**
     * Get absolute encoder angle in radians (with offset applied)
     *
     * @return Absolute encoder angle in radians
     */
    private double getAbsoluteEncoderRadians() {
        double absolutePositionDegrees =
                absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
        double offsetPositionDegrees = absolutePositionDegrees - absoluteEncoderOffset;
        return Units.degreesToRadians(offsetPositionDegrees);
    }

    /**
     * Get raw absolute encoder reading in degrees (for calibration)
     *
     * @return Raw absolute encoder reading in degrees
     */
    public double getRawAbsoluteEncoderDegrees() {
        return absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 360;
    }

    /** Update telemetry for this module */
    public void updateTelemetry() {
        SmartDashboard.putNumber(moduleName + " Drive Velocity (m/s)", getDriveVelocityMPS());
        SmartDashboard.putNumber(
                moduleName + " Steer Angle (deg)", Math.toDegrees(getSteerAngleRadians()));
        SmartDashboard.putNumber(
                moduleName + " Absolute Encoder (deg)", getRawAbsoluteEncoderDegrees());
        SmartDashboard.putNumber(moduleName + " Drive Position (m)", getDrivePositionMeters());
    }
}
