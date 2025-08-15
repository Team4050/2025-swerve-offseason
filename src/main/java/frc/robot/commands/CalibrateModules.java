package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

public class CalibrateModules extends Command {
    private final Drivetrain drivetrain;

    public CalibrateModules(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;

        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        SmartDashboard.putString(
                "Calibration Status", "CALIBRATING - Align all wheels straight forward");
        drivetrain.setSteeringCoast(true);
    }

    @Override
    public void execute() {
        Double[] rawEncoders = drivetrain.getRawAbsoluteEncoderDegrees();
        SmartDashboard.putNumber("FL Raw Encoder (for offset)", rawEncoders[0]);
        SmartDashboard.putNumber("FR Raw Encoder (for offset)", rawEncoders[1]);
        SmartDashboard.putNumber("BL Raw Encoder (for offset)", rawEncoders[2]);
        SmartDashboard.putNumber("BR Raw Encoder (for offset)", rawEncoders[3]);
        SmartDashboard.putString(
                "FL Offset Code", String.format("FL_OFFSET = %.2f;", rawEncoders[0]));
        SmartDashboard.putString(
                "FR Offset Code", String.format("FR_OFFSET = %.2f;", rawEncoders[1]));
        SmartDashboard.putString(
                "BL Offset Code", String.format("BL_OFFSET = %.2f;", rawEncoders[2]));
        SmartDashboard.putString(
                "BR Offset Code", String.format("BR_OFFSET = %.2f;", rawEncoders[3]));
    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString(
                "Calibration Status", "COMPLETE - Copy offset values to Constants.java");
        drivetrain.setSteeringCoast(false);
    }

    @Override
    public boolean isFinished() {
        return false; // Run until manually stopped
    }
}
