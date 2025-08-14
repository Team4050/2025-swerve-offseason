package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class SwerveModule {
    public static final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    public static final double DRIVE_GEAR_RATIO = 6.12; // MK4i L2 drive ratio
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0; // MK4i steer ratio (~21.43:1)

    // Motor configuration
    public static final double DRIVE_CURRENT_LIMIT = 40.0; // Amps
    public static final double STEER_CURRENT_LIMIT = 20.0; // Amps

    // PID Constants for steer motor
    public static final double STEER_KP = 0.2;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 0.0;
  }

  public static final class Drivetrain {
    // Module CAN IDs
    public static final int FL_DRIVE_ID = 1;
    public static final int FL_STEER_ID = 2;
    public static final int FL_ANGLE_ID = 3;

    public static final int FR_DRIVE_ID = 4;
    public static final int FR_STEER_ID = 5;
    public static final int FR_ANGLE_ID = 6;

    public static final int BL_DRIVE_ID = 7;
    public static final int BL_STEER_ID = 8;
    public static final int BL_ANGLE_ID = 9;

    public static final int BR_DRIVE_ID = 10;
    public static final int BR_STEER_ID = 11;
    public static final int BR_ANGLE_ID = 12;

    // Calibration offsets (in degrees) - set these after calibration
    public static final double FL_OFFSET = 0.0;
    public static final double FR_OFFSET = 0.0;
    public static final double BL_OFFSET = 0.0;
    public static final double BR_OFFSET = 0.0;

    // Module positions for kinematics (side by side, 20" apart)
    public static final double TRACK_WIDTH = Units.inchesToMeters(20.0);
    public static final double WHEEL_BASE = Units.inchesToMeters(20.0); // Future 4-module use

    public static final Translation2d FL_POSITION =
        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d FR_POSITION =
        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2);
    public static final Translation2d BL_POSITION =
        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2);
    public static final Translation2d BR_POSITION =
        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2);

    // Speed limits (50% as requested)
    public static final double MAX_SPEED_MPS = 2.5; // meters per second
    public static final double MAX_ANGULAR_SPEED_RPS = Math.PI; // radians per second
  }

  public static final class OI {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final float CONTROLLER_DEADZONE = 0.08f;
  }
}
