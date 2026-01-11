// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Field-specific overrides for auto-aim constants.
 * Set OVERRIDES_ENABLED to true and modify values as needed for specific venues.
 * Values set to null use defaults from Constants.
 */
public final class FieldOverrides {

    // Set to true to enable field overrides
    public static final boolean OVERRIDES_ENABLED = false;

    // Target position overrides (null = use default)
    public static final Double BLUE_TARGET_X = null;
    public static final Double BLUE_TARGET_Y = null;
    public static final Double RED_TARGET_X = null;
    public static final Double RED_TARGET_Y = null;

    // Zone boundary overrides (null = use default)
    public static final Double BLUE_ZONE_MIN_X = null;
    public static final Double BLUE_ZONE_MAX_X = null;
    public static final Double BLUE_ZONE_MIN_Y = null;
    public static final Double BLUE_ZONE_MAX_Y = null;
    public static final Double RED_ZONE_MIN_X = null;
    public static final Double RED_ZONE_MAX_X = null;
    public static final Double RED_ZONE_MIN_Y = null;
    public static final Double RED_ZONE_MAX_Y = null;

    // Distance threshold overrides (null = use default)
    public static final Double MIN_SHOOTING_DISTANCE = null;
    public static final Double MAX_SHOOTING_DISTANCE = null;

    // Heading PID overrides (null = use default)
    public static final Double HEADING_KP = null;
    public static final Double HEADING_KI = null;
    public static final Double HEADING_KD = null;

    // Helper methods to get effective values
    public static double getBlueTargetX() {
        return (OVERRIDES_ENABLED && BLUE_TARGET_X != null) ? BLUE_TARGET_X : Constants.AutoAim.BLUE_TARGET_X;
    }

    public static double getBlueTargetY() {
        return (OVERRIDES_ENABLED && BLUE_TARGET_Y != null) ? BLUE_TARGET_Y : Constants.AutoAim.BLUE_TARGET_Y;
    }

    public static double getRedTargetX() {
        return (OVERRIDES_ENABLED && RED_TARGET_X != null) ? RED_TARGET_X : Constants.AutoAim.RED_TARGET_X;
    }

    public static double getRedTargetY() {
        return (OVERRIDES_ENABLED && RED_TARGET_Y != null) ? RED_TARGET_Y : Constants.AutoAim.RED_TARGET_Y;
    }

    public static double getBlueZoneMinX() {
        return (OVERRIDES_ENABLED && BLUE_ZONE_MIN_X != null) ? BLUE_ZONE_MIN_X : Constants.AutoAim.BLUE_ZONE_MIN_X;
    }

    public static double getBlueZoneMaxX() {
        return (OVERRIDES_ENABLED && BLUE_ZONE_MAX_X != null) ? BLUE_ZONE_MAX_X : Constants.AutoAim.BLUE_ZONE_MAX_X;
    }

    public static double getBlueZoneMinY() {
        return (OVERRIDES_ENABLED && BLUE_ZONE_MIN_Y != null) ? BLUE_ZONE_MIN_Y : Constants.AutoAim.BLUE_ZONE_MIN_Y;
    }

    public static double getBlueZoneMaxY() {
        return (OVERRIDES_ENABLED && BLUE_ZONE_MAX_Y != null) ? BLUE_ZONE_MAX_Y : Constants.AutoAim.BLUE_ZONE_MAX_Y;
    }

    public static double getRedZoneMinX() {
        return (OVERRIDES_ENABLED && RED_ZONE_MIN_X != null) ? RED_ZONE_MIN_X : Constants.AutoAim.RED_ZONE_MIN_X;
    }

    public static double getRedZoneMaxX() {
        return (OVERRIDES_ENABLED && RED_ZONE_MAX_X != null) ? RED_ZONE_MAX_X : Constants.AutoAim.RED_ZONE_MAX_X;
    }

    public static double getRedZoneMinY() {
        return (OVERRIDES_ENABLED && RED_ZONE_MIN_Y != null) ? RED_ZONE_MIN_Y : Constants.AutoAim.RED_ZONE_MIN_Y;
    }

    public static double getRedZoneMaxY() {
        return (OVERRIDES_ENABLED && RED_ZONE_MAX_Y != null) ? RED_ZONE_MAX_Y : Constants.AutoAim.RED_ZONE_MAX_Y;
    }

    public static double getMinShootingDistance() {
        return (OVERRIDES_ENABLED && MIN_SHOOTING_DISTANCE != null)
                ? MIN_SHOOTING_DISTANCE
                : Constants.AutoAim.MIN_SHOOTING_DISTANCE;
    }

    public static double getMaxShootingDistance() {
        return (OVERRIDES_ENABLED && MAX_SHOOTING_DISTANCE != null)
                ? MAX_SHOOTING_DISTANCE
                : Constants.AutoAim.MAX_SHOOTING_DISTANCE;
    }

    public static double getHeadingKP() {
        return (OVERRIDES_ENABLED && HEADING_KP != null) ? HEADING_KP : Constants.AutoAim.HEADING_KP;
    }

    public static double getHeadingKI() {
        return (OVERRIDES_ENABLED && HEADING_KI != null) ? HEADING_KI : Constants.AutoAim.HEADING_KI;
    }

    public static double getHeadingKD() {
        return (OVERRIDES_ENABLED && HEADING_KD != null) ? HEADING_KD : Constants.AutoAim.HEADING_KD;
    }
}
