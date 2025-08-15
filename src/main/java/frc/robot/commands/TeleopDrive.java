package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

public class TeleopDrive extends Command {
    private final Drivetrain driveSubsystem;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotationSupplier;
    private final BooleanSupplier fieldOrientedSupplier;

    public TeleopDrive(
            Drivetrain driveSubsystem,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotationSupplier,
            BooleanSupplier fieldOrientedSupplier) {
        this.driveSubsystem = driveSubsystem;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotationSupplier = rotationSupplier;
        this.fieldOrientedSupplier = fieldOrientedSupplier;

        addRequirements(driveSubsystem);
    }

    @Override
    public void execute() {
        double xInput = xSupplier.getAsDouble();
        double yInput = ySupplier.getAsDouble();
        double rotationInput = rotationSupplier.getAsDouble();
        boolean fieldOriented = fieldOrientedSupplier.getAsBoolean();

        double xVelocity = xInput * Constants.Drivetrain.MAX_SPEED_MPS;
        double yVelocity = yInput * Constants.Drivetrain.MAX_SPEED_MPS;
        double rotationalVelocity = rotationInput * Constants.Drivetrain.MAX_ANGULAR_SPEED_RPS;

        driveSubsystem.drive(xVelocity, yVelocity, rotationalVelocity, fieldOriented);
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.stop();
    }
}
