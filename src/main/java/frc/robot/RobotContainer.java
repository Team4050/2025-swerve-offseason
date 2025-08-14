// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.TeleopDrive;
import frc.robot.hazard.HazardXbox;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;

public class RobotContainer {
  private final HazardXbox driverController =
      new HazardXbox(Constants.OI.DRIVER_CONTROLLER_PORT, Constants.OI.CONTROLLER_DEADZONE);

  private final Drivetrain drivetrain =
      new Drivetrain(
          new SwerveModule(
              "FL",
              Constants.Drivetrain.FL_DRIVE_ID,
              Constants.Drivetrain.FL_STEER_ID,
              Constants.Drivetrain.FL_ANGLE_ID,
              Constants.Drivetrain.FL_OFFSET));

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    // Reset pose
    driverController.start().onTrue(new InstantCommand(() -> drivetrain.resetPose()));

    // Reset steer encoder
    driverController.back().onTrue(new InstantCommand(() -> drivetrain.resetSteerEncoders()));
  }

  private void configureDefaultCommands() {
    drivetrain.setDefaultCommand(
        new TeleopDrive(
            drivetrain,
            () -> -driverController.getLeftY(), // Forward/backward (inverted)
            () -> -driverController.getLeftX(), // Left/right (inverted)
            () -> -driverController.getRightX(), // Rotation (inverted)
            () -> driverController.leftBumper().getAsBoolean() // Field-oriented toggle
            ));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
