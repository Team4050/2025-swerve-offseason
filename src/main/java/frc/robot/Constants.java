package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * Constants for the single module test setup.
 *
 * <p>IMPORTANT SETUP INSTRUCTIONS: 1. Set the CAN IDs below to match your hardware 2. Run the
 * calibrateAngleOffset() method to find your module's zero position 3. Update ANGLE_OFFSET_RAD with
 * the calibration value 4. Tune PID gains if needed for smooth steering
 */
public final class Constants {

  public static final class SingleModuleConstants {
    // ========== HARDWARE CONFIGURATION ==========
    // TODO: UPDATE THESE CAN IDs TO MATCH YOUR SETUP
    public static final int DRIVE_MOTOR_ID = 2; // Kraken X60 for driving
    public static final int STEER_MOTOR_ID = 4; // Kraken X60 for steering
    public static final int CANCODER_ID = 11; // CANcoder for absolute position

    // ========== MECHANICAL SPECIFICATIONS ==========
    // MK4i Level 2 specifications (shouldn't need to change these)
    public static final double WHEEL_DIAMETER_M = Units.inchesToMeters(4.0); // 4" wheels
    public static final double DRIVE_GEAR_RATIO = 6.75; // MK4i L2 drive ratio
    public static final double STEER_GEAR_RATIO = 150.0 / 7.0; // MK4i steer ratio (~21.43:1)
    public static final double MOTOR_FREE_RPM = 6000.0; // Kraken X60 free speed

    // ========== CALIBRATION ==========
    // TODO: RUN CALIBRATION AND UPDATE THIS VALUE
    public static final double ANGLE_OFFSET_RAD =
        60.9; // Set this after running calibrateAngleOffset()

    // ========== CONTROL PARAMETERS ==========
    // Steer PID gains - tune these for your module
    public static final double STEER_KP = 5.0; // Proportional gain
    public static final double STEER_KI = 0.0; // Integral gain (start at 0)
    public static final double STEER_KD = 0.1; // Derivative gain

    // Speed limits for safe testing
    public static final double MAX_DRIVE_SPEED_MPS = 2.0; // Limit for safety during testing

    // ========== INVERSION SETTINGS ==========
    // Adjust these if your module spins the wrong direction
    public static final boolean DRIVE_MOTOR_INVERTED = false; // Set true if drive goes backward
    public static final boolean STEER_MOTOR_INVERTED =
        true; // Set true if steer goes wrong direction
    public static final boolean CANCODER_INVERTED = false; // Set true if CANcoder reads opposite
  }

  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;

    // Input scaling factors for testing
    public static final double DRIVE_SPEED_SCALE = 1.5; // Max m/s when stick is full
    public static final double STEER_ANGLE_RANGE_DEG = 90.0; // +/- degrees when stick is full
  }

  // ========== SETUP CHECKLIST ==========
  /*
   * BEFORE RUNNING THE CODE:
   *
   * 1. HARDWARE SETUP:
   *    - Connect your Kraken X60 motors to the CAN bus
   *    - Connect your CANcoder to the CAN bus
   *    - Update the CAN IDs above to match your setup
   *
   * 2. MOTOR DIRECTIONS:
   *    - Enable the robot and try driving forward (positive left stick Y)
   *    - If the wheel spins backward, set DRIVE_MOTOR_INVERTED = true
   *    - Try steering (right stick X) - wheel should point in logical direction
   *    - If steering goes wrong way, set STEER_MOTOR_INVERTED = true
   *
   * 3. CANCODER CALIBRATION:
   *    - Point your module straight forward (0 degrees)
   *    - Press Y button on controller to run calibration
   *    - Look at console output and update ANGLE_OFFSET_RAD
   *    - Restart robot code with new offset
   *
   * 4. PID TUNING (if needed):
   *    - If steering is too slow/fast, adjust STEER_KP
   *    - If steering oscillates, reduce STEER_KP or increase STEER_KD
   *    - If there's steady-state error, add small STEER_KI
   *
   * 5. CONTROLS:
   *    - Left stick Y: Drive forward/backward
   *    - Right stick X: Steer left/right (absolute position)
   *    - A button: Stop module
   *    - B button: Enable/disable module
   *    - Y button: Calibrate angle offset
   *
   * 6. SMARTDASHBOARD:
   *    - Monitor "Single/Current Angle" and "Single/Current Speed"
   *    - Watch motor temperatures during testing
   *    - Use "Single/Module Enabled" to enable/disable
   */
}
