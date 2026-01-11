// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Central location for all robot constants.
 */
public final class Constants {

    public static final class AutoAim {
        // Target coordinates (meters) - WPILib field coordinate system
        // Origin at blue alliance corner, X toward red, Y toward left
        public static final double BLUE_TARGET_X = 0.0;
        public static final double BLUE_TARGET_Y = 5.55; // Center of field width
        public static final double RED_TARGET_X = 13; // Field length
        public static final double RED_TARGET_Y = 4;

        // Auto-aim zones (rectangular, defined by two corners in meters)
        // Blue alliance zone (near blue target)
        public static final double BLUE_ZONE_MIN_X = 0.0;
        public static final double BLUE_ZONE_MAX_X = 5.0;
        public static final double BLUE_ZONE_MIN_Y = 1.0;
        public static final double BLUE_ZONE_MAX_Y = 7.0;

        // Red alliance zone (near red target)
        public static final double RED_ZONE_MIN_X = 9;
        public static final double RED_ZONE_MAX_X = 17;
        public static final double RED_ZONE_MIN_Y = 0;
        public static final double RED_ZONE_MAX_Y = 7.8;

        // Distance thresholds (meters) - optimal shooting range
        public static final double MIN_SHOOTING_DISTANCE = 2.44; // ~8 feet
        public static final double MAX_SHOOTING_DISTANCE = 3.66; // ~12 feet

        // Angular tolerance for "aimed" state (radians)
        public static final double AIM_TOLERANCE_RADIANS = Math.toRadians(2.0);

        // HeadingController PID gains for FieldCentricFacingAngle
        public static final double HEADING_KP = 7.0;
        public static final double HEADING_KI = 0.0;
        public static final double HEADING_KD = 0.3;
    }

    public static final class Vision {
        public static final String LIMELIGHT_NAME = "limelight";
        public static final boolean VISION_ENABLED = false; // Disabled for now

        // Standard deviations for vision measurements [x, y, theta]
        public static final double VISION_STD_DEV_X = 0.5;
        public static final double VISION_STD_DEV_Y = 0.5;
        public static final double VISION_STD_DEV_THETA = Math.toRadians(10);
    }

    public static final class OI {
        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final float DRIVER_DEADBAND = 0.08f;
    }
}
