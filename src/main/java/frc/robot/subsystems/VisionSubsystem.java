// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Stubbed vision subsystem for future Limelight integration.
 * When enabled, will provide AprilTag-based pose estimation to the drivetrain.
 */
public class VisionSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;
    private final boolean enabled;

    public VisionSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        this.enabled = Constants.Vision.VISION_ENABLED;
    }

    @Override
    public void periodic() {
        if (!enabled) {
            return;
        }

        // TODO: Implement Limelight pose estimation when hardware is available
        // 1. Get pose from Limelight NetworkTables (LimelightHelpers or direct NT access)
        // 2. Validate pose (check for valid targets, reasonable values)
        // 3. Calculate timestamp from latency
        // 4. Call drivetrain.addVisionMeasurement(pose, timestamp, stdDevs)
    }

    /**
     * @return Whether vision is enabled
     */
    public boolean isEnabled() {
        return enabled;
    }

    /**
     * @return Whether a valid AprilTag target is visible
     */
    public boolean hasValidTarget() {
        // Stub - always false until Limelight is integrated
        return false;
    }
}
