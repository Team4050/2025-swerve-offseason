// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.AutoAimSubsystem;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed =
            0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1)
            .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    // Auto-aim driving request - uses built-in HeadingController
    private final SwerveRequest.FieldCentricFacingAngle driveWithAutoAim = new SwerveRequest.FieldCentricFacingAngle()
            .withDeadband(MaxSpeed * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    // Changed from CommandXboxController to HazardXbox for deadband support
    private final HazardXbox joystick =
            new HazardXbox(Constants.OI.DRIVER_CONTROLLER_PORT, Constants.OI.DRIVER_DEADBAND);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // New subsystems
    private final VisionSubsystem vision;
    private final AutoAimSubsystem autoAim;

    public RobotContainer() {
        // Instantiate subsystems
        vision = new VisionSubsystem(drivetrain);
        autoAim = new AutoAimSubsystem(drivetrain);

        // Configure the HeadingController PID gains for FieldCentricFacingAngle
        driveWithAutoAim.HeadingController.setPID(
                FieldOverrides.getHeadingKP(), FieldOverrides.getHeadingKI(), FieldOverrides.getHeadingKD());
        // Enable continuous input for heading (wraps around at +/- PI)
        driveWithAutoAim.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(() -> {
                    // Get driver inputs
                    double velocityX = -joystick.getLeftY() * MaxSpeed;
                    double velocityY = -joystick.getLeftX() * MaxSpeed;
                    double driverRotation = joystick.getRightX();

                    // Check for manual rotation override (any rotation input after deadband = override)
                    if (driverRotation != 0) {
                        autoAim.disableFromManualInput();
                    }

                    // Choose request based on auto-aim state
                    if (autoAim.isActivelyAiming()) {
                        // Use FieldCentricFacingAngle with calculated target
                        return driveWithAutoAim
                                .withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withTargetDirection(autoAim.getTargetAngle().plus(Rotation2d.k180deg));
                    } else {
                        // Use normal FieldCentric with driver rotation
                        return drive.withVelocityX(velocityX)
                                .withVelocityY(velocityY)
                                .withRotationalRate(-driverRotation * MaxAngularRate);
                    }
                }));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled()
                .whileTrue(drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b()
                .whileTrue(drivetrain.applyRequest(
                        () -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on Start + Back combo (moved from Left Bumper)
        joystick.start().and(joystick.back()).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // Left Bumper: Toggle auto-aim on/off
        joystick.leftBumper().onTrue(Commands.runOnce(() -> autoAim.toggleEnabled()));

        // Left Trigger: One-press aim (tap = aim once, hold = continuous)
        joystick.leftTrigger()
                .onTrue(Commands.runOnce(() -> autoAim.startOnePressAim()))
                .onFalse(Commands.runOnce(() -> autoAim.endOnePressAim()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
                // Reset our field centric heading to match the robot
                // facing away from our alliance station wall (0 deg).
                drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
                // Then slowly drive forward (away from us) for 5 seconds.
                drivetrain
                        .applyRequest(
                                () -> drive.withVelocityX(0.5).withVelocityY(0).withRotationalRate(0))
                        .withTimeout(5.0),
                // Finally idle for the rest of auton
                drivetrain.applyRequest(() -> idle));
    }
}
