// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * Constants for FRC 2026 REBUILT season Contains game-specific values, timing,
 * and robot
 * configuration
 */
public final class Constants {

  /** Game timing constants for REBUILT */
  public static final class GameConstants {
    // Match period durations (seconds)
    public static final double AUTO_DURATION = 20.0; // 0:00 - 0:20
    public static final double TELEOP_DURATION = 150.0; // 2:30 total
    public static final double ENDGAME_THRESHOLD = 30.0; // Last 30 seconds

    // Hub Shift timing (transition period where both hubs are active)
    public static final double TRANSITION_START_TIME = 150.0; // Match time when transition starts
    public static final double TRANSITION_END_TIME = 140.0; // Match time when transition ends
    public static final double TRANSITION_DURATION = 10.0; // 10 second transition period

    // Fuel constants
    public static final int PRELOAD_FUEL_LIMIT = 8;
    // NO max capacity during gameplay - robots can hoard unlimited fuel!

    // Climb scoring
    public static final int CLIMB_L1_POINTS = 15; // Level 1 climb points
    public static final int CLIMB_L2_POINTS = 30; // Level 2 climb points (estimated)
    public static final int CLIMB_L3_POINTS = 50; // Level 3 climb points (estimated)

    // Height restriction during climb (inches)
    public static final double MAX_HEIGHT_DURING_CLIMB = 30.0;
  }

  /** Controller port assignments */
  public static final class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;

    // Deadband for joystick inputs
    public static final double JOYSTICK_DEADBAND = 0.1;
  }

  /** Drivetrain constants */
  public static final class DrivetrainConstants {
    // ==================== SPEEDS ====================
    // These are the ABSOLUTE maximums - derived from TunerConstants.kSpeedAt12Volts
    // The robot physically cannot exceed these values
    public static final double MAX_SPEED_MPS = 10.0; // The theoretical possible maximum translation speed is 10.81
    // m/s - from TunerConstants
    public static final double MAX_ANGULAR_RATE_RAD_PER_SEC = Math.PI * 2.0; // 360 deg/s
    // ==================== SPEED COEFFICIENTS ====================
    // Runtime multipliers applied to MAX_SPEED - these can be changed dynamically
    // Use: swerve.driveFieldCentricSmooth(..., MAX_SPEED_MPS *
    // NORMAL_SPEED_COEFFICIENT, ...)
    public static final double NORMAL_SPEED_COEFFICIENT = 1.0; // Full speed
    public static final double SLOW_MODE_COEFFICIENT = 0.7; // Precision mode (70%)
    public static final double SCORING_SPEED_COEFFICIENT = 0.5; // Slow for scoring

    // ==================== DEADBAND ====================
    public static final double DEADBAND_PERCENT = 0.1;

    // Vision alignment tolerances
    public static final double POSITION_TOLERANCE_METERS = 0.02; // 2cm position tolerance
    public static final double YAW_TOLERANCE_RADIANS = Math.PI / 32; // ~5.6 degrees

    // Alignment PID values
    public static final double ALIGN_PID_KP = 8.0;
    public static final double ALIGN_PID_KI = 0.0;
    public static final double ALIGN_PID_KD = 0.01;
    public static final double ALIGN_ROTATION_KP = 3.0;
    public static final double ALIGN_ROTATION_KD = 0.02;

    // Movement speeds during alignment (m/s, rad/s)
    public static final double ALIGN_SPEED_MPS = 3.5;
    public static final double ALIGN_ROTATION_SPEED = 0.9;

    // Alignment offset distances from AprilTag (meters)
    // These define where the robot stops relative to the tag
    public static final double ALIGN_OFFSET_X_LEFT = -0.41; // Back from tag for LEFT
    public static final double ALIGN_OFFSET_Y_LEFT = 0.13; // Left of tag center
    public static final double ALIGN_OFFSET_X_RIGHT = -0.41; // Back from tag for RIGHT
    public static final double ALIGN_OFFSET_Y_RIGHT = -0.23; // Right of tag center
    public static final double ALIGN_OFFSET_X_CENTER = -0.50; // Back from tag for CENTER
    public static final double ALIGN_OFFSET_Y_CENTER = 0.0; // Centered on tag
  }

  /** Intake constants (placeholder) */
  public static final class IntakeConstants {
    public static final double INTAKE_SPEED = -0.5;
    public static final int INTAKE_MOTOR_ID = 19; // TODO: Make real
    public static final int PIVOT_ENCODER_ID = 1; // TODO: Make real
    public static final int PIVOT_MOTOR_ID = 2; // TODO: Make real
    public static final double PIVOT_INTAKE_POSITION = 0.0; // TODO: Tune
    public static final double PIVOT_STOWED_POSITION = 0.5; // TODO: Tune
  }

  /**
   * Shooter constants
   *
   * <p>
   * Hardware: 2x Kraken X60 (TalonFX) on CANivore bus Configuration:
   * Counter-rotating flywheels
   * (slave inverted)
   */
  public static final class ShooterConstants {
    // CAN IDs for shooter motors
    public static final int MASTER_MOTOR_ID = 7; // TODO: Set actual CAN ID
    public static final int SLAVE_MOTOR_ID = 20; // TODO: Set actual CAN ID

    // Target RPM values
    public static final double SHOOTER_IDLE_RPM = 0;
    public static final double SHOOTER_SPINUP_RPM = 1000; // Default spin-up target
    public static final double SHOOTER_MAX_RPM = 5500; // Kraken X60 free speed ~6000 RPM

    // RPM tolerance for "at setpoint" check
    // 1678 used 500 RPM - starting with 150 cuz Krakens are more consistent at that
    // speed
    public static final double SHOOTER_RPM_TOLERANCE = 150;

    // Velocity PID gains
    // Units: kS in Volts, kV in Volts per RPS, kP in Volts per RPS error
    // These values need to be tuned!
    public static final double SHOOTER_KS = 0.15; // Static friction compensation
    public static final double SHOOTER_KV = 0.12; // Velocity feedforward (main term)
    public static final double SHOOTER_KP = 0.3; // Proportional gain for error correction

    // Feeder speed when firing
    public static final double FEEDER_SPEED = 1.0;
  }

  /** Climber constants (placeholder) */
  public static final class ClimberConstants {
    // Climber arm positions (encoder units - fill in with actual values later)
    public static final double CLIMBER_STOWED_POSITION = 0;
    public static final double CLIMBER_L1_REACH_POSITION = 50;
    public static final double CLIMBER_L2_REACH_POSITION = 80;
    public static final double CLIMBER_L3_REACH_POSITION = 100;

    // Climber motor speeds
    public static final double CLIMBER_EXTEND_SPEED = 0.7;
    public static final double CLIMBER_RETRACT_SPEED = -0.8;
  }

  /**
   * Heading Controller constants
   *
   * <p>
   * These control the swerve heading lock behavior. SNAP mode: Higher gains for
   * quickly rotating
   * to a target MAINTAIN mode: Lower gains for holding position with minimal
   * oscillation
   */
  public static final class HeadingControllerConstants {
    // SNAP mode gains - aggressive to quickly reach target heading
    // Output is normalized (-1 to 1), so these are effectively output per degree of
    // error
    public static final double SNAP_KP = 0.02; // 254 used 0.05 (degrees output per degree error)
    public static final double SNAP_KI = 0.0;
    public static final double SNAP_KD = 0.001;

    // MAINTAIN mode gains - gentle to hold position without oscillation
    public static final double MAINTAIN_KP = 0.01; // 254 used 0.01
    public static final double MAINTAIN_KI = 0.0;
    public static final double MAINTAIN_KD = 0.0005;

    // Heading tolerance for "at goal" detection (degrees)
    public static final double HEADING_TOLERANCE_DEGREES = 2.0;
  }

  /** State machine timing constants */
  public static final class StateMachineConstants {
    // Rumble feedback durations (seconds)
    public static final double RUMBLE_SHORT = 0.15;
    public static final double RUMBLE_MEDIUM = 0.3;
    public static final double RUMBLE_LONG = 0.5;
    public static final double RUMBLE_EXTRA_LONG = 1.0;

    // Rumble intensities
    public static final double RUMBLE_LIGHT = 0.3;
    public static final double RUMBLE_MEDIUM_INTENSITY = 0.5;
    public static final double RUMBLE_STRONG = 0.8;
    public static final double RUMBLE_MAX = 1.0;

    // State history buffer size
    public static final int STATE_HISTORY_SIZE = 20;

    /** Minimum cycle time to be considered valid (seconds) */
    public static final double MIN_VALID_CYCLE_TIME = 1.0;
  }

  /** Vision/Limelight constants (placeholder) */
  public static final class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";

    // Pipeline IDs
    public static final int PIPELINE_HUB_TRACKING = 0;
    public static final int PIPELINE_FUEL_DETECTION = 0;
    public static final int PIPELINE_APRILTAG = 0;

    // Target height for hub (inches from floor)
    public static final double HUB_TARGET_HEIGHT_INCHES = 104.0; // Placeholder

    // Camera mounting (inches)
    public static final double CAMERA_HEIGHT_INCHES = 24.0; // Placeholder
    public static final double CAMERA_MOUNT_ANGLE_DEGREES = 30.0; // Placeholder

    // ==================== MEGATAG2 VISION ESTIMATION CONSTANTS
    // ====================
    // Standard deviation coefficients for vision measurements
    // These scale with distance and inversely with tag count
    // Formula: stdDev = coefficient * (distance^1.2) / (tagCount^2.0)
    public static final double XY_STD_DEV_COEFFICIENT = 0.01; // Base XY standard deviation
    public static final double THETA_STD_DEV_COEFFICIENT = 0.03; // Base theta standard deviation (not used with
    // MegaTag2)

    // Maximum angular velocity for valid vision measurements (degrees/sec)
    // MegaTag2 results degrade significantly when spinning fast
    // Limelight docs recommend 360 deg/s; previously 720 was too lenient
    public static final double MAX_ANGULAR_VELOCITY_DEG_PER_SEC = 360.0; // 1 rotation/sec

    // Field boundary margins for pose rejection (meters)
    public static final double FIELD_BORDER_MARGIN = 0.5;
    public static final double FIELD_LENGTH_METERS = 16.54;
    public static final double FIELD_WIDTH_METERS = 8.07;

    // Maximum Z error for valid poses (meters) - robot should be near the ground
    public static final double MAX_Z_ERROR = 0.75;

    // Ambiguity threshold for single-tag MegaTag1 (not used with MegaTag2)
    public static final double MAX_AMBIGUITY = 0.3;

    // ==================== MEGATAG2 FLIP FIX CONSTANTS ====================
    // Maximum allowable distance (meters) between vision pose and odometry pose.
    // If the vision pose diverges further than this from odometry, it is rejected.
    // This catches the "MegaTag2 flip" bug where the pose jumps to the opposite
    // side of the field due to IMU heading errors or multi-tag ambiguity.
    public static final double MAX_VISION_ODO_DIVERGENCE_SINGLE_TAG = 2.0; // 1 tag seen
    public static final double MAX_VISION_ODO_DIVERGENCE_MULTI_TAG = 4.0; // 2+ tags seen

    // When multi-tag measurement is suspicious (diverges > this threshold but
    // within the hard-reject limit), scale up std devs by this factor instead
    // of hard-rejecting. This lets the Kalman filter apply a very weak correction.
    public static final double SUSPICIOUS_DIVERGENCE_THRESHOLD = 1.0; // meters
    public static final double SUSPICIOUS_STD_DEV_SCALE = 10.0; // multiply std devs by 10x

    // ==================== LIMELIGHT 4 IMU MODES ====================
    // Mode 0: EXTERNAL_ONLY - Uses external gyro via SetRobotOrientation
    // Mode 1: EXTERNAL_SEED - Seeds internal IMU with external, uses external for
    // botpose
    // Mode 2: INTERNAL_ONLY - Uses LL4's internal IMU only
    // Mode 3: INTERNAL_MT1_ASSIST - Internal IMU + MT1 vision yaw correction
    // Mode 4: INTERNAL_EXTERNAL_ASSIST - Internal 1kHz IMU + external gyro drift
    // correction
    public static final int IMU_MODE_EXTERNAL_ONLY = 0;
    public static final int IMU_MODE_SEED_EXTERNAL = 1;
    public static final int IMU_MODE_INTERNAL_ONLY = 2;
    public static final int IMU_MODE_INTERNAL_MT1_ASSIST = 3;
    public static final int IMU_MODE_INTERNAL_EXTERNAL_ASSIST = 4;

    // IMU assist alpha for complementary filter (modes 3 and 4)
    public static final double IMU_ASSIST_ALPHA = 0.001; // Higher = faster convergence to assist source
  }

  public static final class IndexerConstants {
    public static final int kFeederMotorID = 20;
    public static final int kSpindexerMotorID = 15;

    // Safety
    public static final int kCurrentLimit = 40; // Amps

    // ==================== FEEDER PID GAINS ====================
    public static final double kFeederKP = 0.2;
    public static final double kFeederKI = 0.0;
    public static final double kFeederKD = 0.0;
    public static final double kFeederKS = 0.15;
    public static final double kFeederKV = 0.12;
    public static final double kFeederKA = 0.01;

    public static final double kFeederKG = 0.0;

    // ==================== SPINDEXER PID GAINS ====================
    public static final double kSpindexerKP = 0.3;
    public static final double kSpindexerKI = 0.0;
    public static final double kSpindexerKD = 0.0;
    public static final double kSpindexerKS = 0.2;
    public static final double kSpindexerKV = 0.12;
    public static final double kSpindexerKA = 0.01;
    public static final double kSpindexerKG = 0.0;

    // ==================== TARGET SPEEDS (RPM) ====================
    // Feeder (Fast)
    public static final double kFeederTargetRPM = 3000;
    public static final double kFeederReverseRPM = -1000;

    // Spindexer (Slow/Torque)
    public static final double kSpindexerTargetRPM = 1000;
    public static final double kSpindexerReverseRPM = -1000;
  }
  // ==================== UTILITY METHODS ====================

  /** Inches to Meters conversion factor */
  public static final double INCHES_TO_METERS = 0.0254;

  /**
   * Check if a value exists in an array
   *
   * @param array The array to search
   * @param value The value to find
   * @return True if value is in array
   */
  public static boolean contains(double[] array, double value) {
    for (double element : array) {
      if (element == value) {
        return true;
      }
    }
    return false;
  }

  /**
   * Check if an int value exists in an int array
   *
   * @param array The array to search
   * @param value The value to find
   * @return True if value is in array
   */
  public static boolean contains(int[] array, int value) {
    for (int element : array) {
      if (element == value) {
        return true;
      }
    }
    return false;
  }

  // ==================== APRIL TAG FIELD LAYOUT ====================

  /**
   * AprilTag positions on the field
   *
   * <p>
   * Format: HashMap<TagID, double[]{X_inches, Y_inches, Z_inches, Yaw_degrees,
   * Pitch_degrees}>
   */
  public static final class AprilTagMaps {
    public static final java.util.HashMap<Integer, double[]> aprilTagMap = new java.util.HashMap<>();

    static {
      // 2026 REBUILT Field - Welded Perimeter
      // Format: {X, Y, Z, Yaw, Pitch}
      aprilTagMap.put(1, new double[] { 467.64, 292.31, 35.00, 180.0, 0.0 }); // Trench, Red
      aprilTagMap.put(2, new double[] { 469.11, 182.60, 44.25, 90.0, 0.0 }); // Hub, Red
      aprilTagMap.put(3, new double[] { 445.35, 172.84, 44.25, 180.0, 0.0 }); // Hub, Red
      aprilTagMap.put(4, new double[] { 445.35, 158.84, 44.25, 180.0, 0.0 }); // Hub, Red
      aprilTagMap.put(5, new double[] { 469.11, 135.09, 44.25, 270.0, 0.0 }); // Hub, Red
      aprilTagMap.put(6, new double[] { 467.64, 25.37, 35.00, 180.0, 0.0 }); // Trench, Red
      aprilTagMap.put(7, new double[] { 470.59, 25.37, 35.00, 0.0, 0.0 }); // Trench, Red
      aprilTagMap.put(8, new double[] { 483.11, 135.09, 44.25, 270.0, 0.0 }); // Hub, Red
      aprilTagMap.put(9, new double[] { 492.88, 144.84, 44.25, 0.0, 0.0 }); // Hub, Red
      aprilTagMap.put(10, new double[] { 492.88, 158.84, 44.25, 0.0, 0.0 }); // Hub, Red
      aprilTagMap.put(11, new double[] { 483.11, 182.60, 44.25, 90.0, 0.0 }); // Hub, Red
      aprilTagMap.put(12, new double[] { 470.59, 292.31, 35.00, 0.0, 0.0 }); // Trench, Red
      aprilTagMap.put(13, new double[] { 650.92, 291.47, 21.75, 180.0, 0.0 }); // Outpost, Red
      aprilTagMap.put(14, new double[] { 650.92, 274.47, 21.75, 180.0, 0.0 }); // Outpost, Red
      aprilTagMap.put(15, new double[] { 650.90, 170.22, 21.75, 180.0, 0.0 }); // Tower, Red
      aprilTagMap.put(16, new double[] { 650.90, 153.22, 21.75, 180.0, 0.0 }); // Tower, Red
      aprilTagMap.put(17, new double[] { 183.59, 25.37, 35.00, 0.0, 0.0 }); // Trench, Blue
      aprilTagMap.put(18, new double[] { 182.11, 135.09, 44.25, 270.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(19, new double[] { 205.87, 144.84, 44.25, 0.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(20, new double[] { 205.87, 158.84, 44.25, 0.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(21, new double[] { 182.11, 182.60, 44.25, 90.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(22, new double[] { 183.59, 292.31, 35.00, 0.0, 0.0 }); // Trench, Blue
      aprilTagMap.put(23, new double[] { 180.64, 292.31, 35.00, 180.0, 0.0 }); // Trench, Blue
      aprilTagMap.put(24, new double[] { 168.11, 182.60, 44.25, 90.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(25, new double[] { 158.34, 172.84, 44.25, 180.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(26, new double[] { 158.34, 158.84, 44.25, 180.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(27, new double[] { 168.11, 135.09, 44.25, 270.0, 0.0 }); // Hub, Blue
      aprilTagMap.put(28, new double[] { 180.64, 25.37, 35.00, 180.0, 0.0 }); // Trench, Blue
      aprilTagMap.put(29, new double[] { 0.30, 26.22, 21.75, 0.0, 0.0 }); // Outpost, Blue
      aprilTagMap.put(30, new double[] { 0.30, 43.22, 21.75, 0.0, 0.0 }); // Outpost, Blue
      aprilTagMap.put(31, new double[] { 0.32, 147.47, 21.75, 0.0, 0.0 }); // Tower, Blue
      aprilTagMap.put(32, new double[] { 0.32, 164.47, 21.75, 0.0, 0.0 }); // Tower, Blue
    }

    // Red side tag IDs (for direction flipping logic)
    public static final int[] RED_SIDE_TAGS = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16
    };

    // Blue side tag IDs
    public static final int[] BLUE_SIDE_TAGS = {
        17, 18, 19, 20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32
    };
  }

  /** Alignment position enum - left or right side offset from AprilTag */
  public enum AlignPosition {
    LEFT,
    RIGHT,
    CENTER
  }

  /** Starting position enum for autonomous */
  public enum StartingPosition {
    LEFT,
    CENTER,
    RIGHT
  }
}
