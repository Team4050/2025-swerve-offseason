// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SingleModuleTestCommand;
import frc.robot.subsystems.DrivetrainSingle;

public class RobotContainer {
  // Subsystems
  private final DrivetrainSingle drivetrain = new DrivetrainSingle();

  // Controllers
  private final CommandXboxController driverController = new CommandXboxController(0);

  public RobotContainer() {
    configureBindings();
    configureDefaultCommands();
  }

  private void configureBindings() {
    // Add button bindings for testing

    // A button: Stop module
    driverController.a().onTrue(Commands.runOnce(() -> drivetrain.stop(), drivetrain));

    // B button: Enable/disable module
    driverController
        .b()
        .onTrue(Commands.runOnce(() -> drivetrain.setEnabled(!drivetrain.isEnabled()), drivetrain));

    // Y button: Calibrate angle offset (prints current position to console)
    driverController.y().onTrue(Commands.runOnce(() -> drivetrain.calibrateAngleOffset()));
  }

  private void configureDefaultCommands() {
    // Set the default command for the drivetrain to use the single module test command
    drivetrain.setDefaultCommand(new SingleModuleTestCommand(drivetrain, driverController));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
