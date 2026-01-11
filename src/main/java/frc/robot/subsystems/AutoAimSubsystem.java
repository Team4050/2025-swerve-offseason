// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.FieldOverrides;
import java.util.Optional;

/**
 * Auto-aim subsystem that calculates target angles and manages aiming state.
 * Works with RobotContainer to switch between FieldCentric and FieldCentricFacingAngle requests.
 */
public class AutoAimSubsystem extends SubsystemBase {

    private final CommandSwerveDrivetrain drivetrain;

    // State
    private boolean toggleEnabled = false; // Toggle state (LB button)
    private boolean onePressActive = false; // One-press aim active (LT held)
    private boolean manualOverrideActive = false; // Driver rotation detected

    // Cached values (updated in periodic)
    private boolean inZone = false;
    private boolean isAiming = false;
    private boolean inOptimalRange = false;
    private boolean isAimed = false;
    private double distanceToTarget = 0.0;
    private double angleError = 0.0;
    private Rotation2d targetAngle = Rotation2d.kZero;
    private Rotation2d currentAngle = Rotation2d.kZero;

    // Configuration (loaded from FieldOverrides/Constants)
    private final double blueTargetX;
    private final double blueTargetY;
    private final double redTargetX;
    private final double redTargetY;
    private final double blueZoneMinX, blueZoneMaxX, blueZoneMinY, blueZoneMaxY;
    private final double redZoneMinX, redZoneMaxX, redZoneMinY, redZoneMaxY;
    private final double minShootingDistance;
    private final double maxShootingDistance;

    // NetworkTables publishers
    private final NetworkTable table;
    private final BooleanPublisher enabledPub;
    private final BooleanPublisher inZonePub;
    private final BooleanPublisher isAimingPub;
    private final BooleanPublisher shooterReadyPub;
    private final BooleanPublisher inOptimalRangePub;
    private final DoublePublisher distanceToTargetPub;
    private final DoublePublisher angleErrorPub;
    private final DoublePublisher targetAnglePub;
    private final DoublePublisher currentAnglePub;

    public AutoAimSubsystem(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        // Load configuration
        blueTargetX = FieldOverrides.getBlueTargetX();
        blueTargetY = FieldOverrides.getBlueTargetY();
        redTargetX = FieldOverrides.getRedTargetX();
        redTargetY = FieldOverrides.getRedTargetY();
        blueZoneMinX = FieldOverrides.getBlueZoneMinX();
        blueZoneMaxX = FieldOverrides.getBlueZoneMaxX();
        blueZoneMinY = FieldOverrides.getBlueZoneMinY();
        blueZoneMaxY = FieldOverrides.getBlueZoneMaxY();
        redZoneMinX = FieldOverrides.getRedZoneMinX();
        redZoneMaxX = FieldOverrides.getRedZoneMaxX();
        redZoneMinY = FieldOverrides.getRedZoneMinY();
        redZoneMaxY = FieldOverrides.getRedZoneMaxY();
        minShootingDistance = FieldOverrides.getMinShootingDistance();
        maxShootingDistance = FieldOverrides.getMaxShootingDistance();

        // Setup NetworkTables
        table = NetworkTableInstance.getDefault().getTable("SmartDashboard/AutoAim");
        enabledPub = table.getBooleanTopic("Enabled").publish();
        inZonePub = table.getBooleanTopic("InZone").publish();
        isAimingPub = table.getBooleanTopic("IsAiming").publish();
        shooterReadyPub = table.getBooleanTopic("ShooterReady").publish();
        inOptimalRangePub = table.getBooleanTopic("InOptimalRange").publish();
        distanceToTargetPub = table.getDoubleTopic("DistanceToTarget").publish();
        angleErrorPub = table.getDoubleTopic("AngleError").publish();
        targetAnglePub = table.getDoubleTopic("TargetAngle").publish();
        currentAnglePub = table.getDoubleTopic("CurrentAngle").publish();
    }

    @Override
    public void periodic() {
        Pose2d robotPose = drivetrain.getState().Pose;
        Optional<Alliance> alliance = DriverStation.getAlliance();

        if (alliance.isEmpty()) {
            // No alliance set - cannot determine target/zone
            inZone = false;
            isAiming = false;
            publishTelemetry();
            return;
        }

        Alliance currentAlliance = alliance.get();
        currentAngle = robotPose.getRotation();

        // Update zone status
        inZone = isInAutoAimZone(robotPose, currentAlliance);

        // Update target calculations
        targetAngle = calculateTargetAngle(robotPose, currentAlliance);
        distanceToTarget = getDistanceToTarget(robotPose, currentAlliance);
        angleError = targetAngle.minus(currentAngle).getRadians();

        // Update range status
        inOptimalRange = distanceToTarget >= minShootingDistance && distanceToTarget <= maxShootingDistance;

        // Update aimed status
        isAimed = Math.abs(angleError) <= Constants.AutoAim.AIM_TOLERANCE_RADIANS;

        // Determine if actively aiming
        // Active if: (toggle enabled OR one-press active) AND in zone AND no manual override
        isAiming = (toggleEnabled || onePressActive) && inZone && !manualOverrideActive;

        publishTelemetry();
    }

    /**
     * Check if robot is within the auto-aim zone for current alliance.
     */
    public boolean isInAutoAimZone(Pose2d robotPose, Alliance alliance) {
        double x = robotPose.getX();
        double y = robotPose.getY();

        if (alliance == Alliance.Blue) {
            return x >= blueZoneMinX && x <= blueZoneMaxX && y >= blueZoneMinY && y <= blueZoneMaxY;
        } else {
            return x >= redZoneMinX && x <= redZoneMaxX && y >= redZoneMinY && y <= redZoneMaxY;
        }
    }

    /**
     * Calculate the angle to point at the target from current position.
     */
    public Rotation2d calculateTargetAngle(Pose2d robotPose, Alliance alliance) {
        Translation2d target = (alliance == Alliance.Blue)
                ? new Translation2d(blueTargetX, blueTargetY)
                : new Translation2d(redTargetX, redTargetY);

        Translation2d robotToTarget = target.minus(robotPose.getTranslation());
        return new Rotation2d(robotToTarget.getX(), robotToTarget.getY());
    }

    /**
     * Get distance from robot to target.
     */
    public double getDistanceToTarget(Pose2d robotPose, Alliance alliance) {
        Translation2d target = (alliance == Alliance.Blue)
                ? new Translation2d(blueTargetX, blueTargetY)
                : new Translation2d(redTargetX, redTargetY);

        return robotPose.getTranslation().getDistance(target);
    }

    // --- State Control Methods (called from RobotContainer) ---

    /**
     * Toggle auto-aim enabled state (LB button).
     */
    public void toggleEnabled() {
        toggleEnabled = !toggleEnabled;
        manualOverrideActive = false; // Reset manual override when toggling
    }

    /**
     * Start one-press aim (LT pressed).
     */
    public void startOnePressAim() {
        onePressActive = true;
        manualOverrideActive = false;
    }

    /**
     * End one-press aim (LT released).
     */
    public void endOnePressAim() {
        onePressActive = false;
    }

    /**
     * Called when driver applies manual rotation.
     * Disables auto-aim until re-enabled.
     */
    public void disableFromManualInput() {
        manualOverrideActive = true;
    }

    /**
     * @return Whether auto-aim should actively control rotation right now.
     */
    public boolean isActivelyAiming() {
        return isAiming;
    }

    /**
     * @return The target rotation for FieldCentricFacingAngle request.
     */
    public Rotation2d getTargetAngle() {
        return targetAngle;
    }

    /**
     * @return Whether toggle mode is enabled.
     */
    public boolean isToggleEnabled() {
        return toggleEnabled;
    }

    /**
     * @return Whether robot is in the auto-aim zone.
     */
    public boolean isInZone() {
        return inZone;
    }

    /**
     * @return Whether robot is in optimal shooting range.
     */
    public boolean isInOptimalRange() {
        return inOptimalRange;
    }

    /**
     * @return Whether robot is aimed at target (within tolerance).
     */
    public boolean isAimed() {
        return isAimed;
    }

    /**
     * @return Whether shooter is ready (in range AND aimed).
     */
    public boolean isShooterReady() {
        return inOptimalRange && isAimed;
    }

    private void publishTelemetry() {
        enabledPub.set(toggleEnabled || onePressActive);
        inZonePub.set(inZone);
        isAimingPub.set(isAiming);
        shooterReadyPub.set(inOptimalRange && isAimed);
        inOptimalRangePub.set(inOptimalRange);
        distanceToTargetPub.set(distanceToTarget);
        angleErrorPub.set(angleError);
        targetAnglePub.set(targetAngle.getRadians());
        currentAnglePub.set(currentAngle.getRadians());
    }
}
