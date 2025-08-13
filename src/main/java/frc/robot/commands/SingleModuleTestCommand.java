package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DrivetrainSingle;

/**
 * Command for testing a single swerve module with Xbox controller input. Provides intuitive control
 * for module testing and tuning.
 */
public class SingleModuleTestCommand extends Command {
  private final DrivetrainSingle drivetrain;
  private final CommandXboxController controller;

  // Speed scaling factors for testing (adjust as needed)
  private static final double MAX_SPEED_MPS = 1.5; // Conservative for testing
  private static final double ROTATION_SPEED_SCALE = 180; // degrees per second when stick is full

  public SingleModuleTestCommand(DrivetrainSingle drivetrain, CommandXboxController controller) {
    this.drivetrain = drivetrain;
    this.controller = controller;
    addRequirements(drivetrain);
  }

  @Override
  public void execute() {
    // Get controller inputs
    double leftY =
        -controller.getLeftY(); // Forward/backward (inverted because stick up is negative)
    double rightX = controller.getRightX(); // Rotation input for angle control

    // Apply deadband
    leftY = applyDeadband(leftY, 0.05);
    rightX = applyDeadband(rightX, 0.05);

    // Calculate drive speed
    double speed = leftY * MAX_SPEED_MPS;

    // Calculate target angle based on right stick
    // Velocity-based steering: continuous rotation based on stick input
    double currentAngle = drivetrain.getCurrentAngleDeg();
    double angularVelocity = rightX * ROTATION_SPEED_SCALE; // degrees per second
    double targetAngle = currentAngle + (angularVelocity * 0.02); // 20ms loop time

    // Alternative control scheme: Uncomment below for absolute positioning
    /*
    if (Math.abs(rightX) > 0.1) {
      // Right stick controls absolute angle (-90 to +90 degrees)
      targetAngle = rightX * 90.0;
    } else {
      // Hold current angle when stick is neutral
      targetAngle = currentAngle;
    }
    */

    // Drive the module
    drivetrain.drive(speed, targetAngle);
  }

  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  /**
   * Apply deadband to controller input.
   *
   * @param value Raw controller value
   * @param deadband Deadband threshold
   * @return Processed value with deadband applied
   */
  private double applyDeadband(double value, double deadband) {
    if (Math.abs(value) < deadband) {
      return 0.0;
    }
    return Math.copySign((Math.abs(value) - deadband) / (1.0 - deadband), value);
  }
}
