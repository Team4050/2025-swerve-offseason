package frc.robot.hazard;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class HazardXbox extends CommandXboxController {

    private float deadband = 0.08f;
    private XboxController hid;

    /**
     * Initializes a new Xbox controller with the specified port and deadband
     *
     * @param port USB port of the controller
     * @param deadband Deadband (0 - 1)
     */
    public HazardXbox(int port, float deadband) {
        this(port);

        this.deadband = deadband;
        this.hid = super.getHID();
    }

    /**
     * Initializes a new Xbox controller with the specified port
     *
     * @param port USB port of the controller
     */
    public HazardXbox(int port) {
        super(port);

        this.hid = super.getHID();
    }

    /**
     * Rumble the controller
     *
     * @param amount Value to rumble (0 - 1)
     */
    public void rumble(float amount) {
        this.rumbleLeft(amount);
        this.rumbleRight(amount);
    }

    /**
     * Rumble the left side of the controller
     *
     * @param amount Value to rumble (0 - 1)
     */
    public void rumbleLeft(float amount) {
        this.hid.setRumble(GenericHID.RumbleType.kLeftRumble, amount);
    }

    /**
     * Rumble the right side of the controller
     *
     * @param amount Value to rumble (0 - 1)
     */
    public void rumbleRight(float amount) {
        this.hid.setRumble(GenericHID.RumbleType.kRightRumble, amount);
    }

    /* The below code consists of overrides and new methods to apply deadzone */

    /**
     * Returns the value of the axis after accounting for the configured deadband
     *
     * @param deadband The deadband to apply
     * @param v The value of the axis
     * @return The value of the axis after accounting for deadband
     */
    private double deadband(float deadband, double v) {
        return MathUtil.applyDeadband(deadband, v);
    }

    @Override
    public double getLeftX() {
        return this.deadband(this.deadband, super.getLeftX());
    }

    public double getLeftX(float deadband) {
        return this.deadband(deadband, super.getLeftX());
    }

    @Override
    public double getLeftY() {
        return this.deadband(this.deadband, super.getLeftY());
    }

    public double getLeftY(float deadband) {
        return this.deadband(deadband, super.getLeftY());
    }

    @Override
    public double getRightX() {
        return this.deadband(this.deadband, super.getRightX());
    }

    public double getRightX(float deadband) {
        return this.deadband(deadband, super.getRightX());
    }

    @Override
    public double getRightY() {
        return this.deadband(this.deadband, super.getRightY());
    }

    public double getRightY(float deadband) {
        return this.deadband(deadband, super.getRightY());
    }
}
