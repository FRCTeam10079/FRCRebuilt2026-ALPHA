// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers;

/**
 * Subsystem for Limelight vision camera Handles AprilTag detection and robot pose estimation
 *
 * <p>For REBUILT 2026 season - used for hub alignment
 */
public class LimelightSubsystem extends SubsystemBase {
  // Basic targeting data entries
  private final NetworkTableEntry m_tid; // Target AprilTag ID
  private final NetworkTableEntry m_tx; // Horizontal offset from crosshair (degrees)
  private final NetworkTableEntry m_ty; // Vertical offset from crosshair (degrees)
  private final NetworkTableEntry m_ta; // Target area (0-100% of image)
  private final NetworkTableEntry m_tv; // Valid target exists (0 or 1)

  // Pose data entries
  private final NetworkTableEntry m_botPose; // Robot pose relative to target
  private final NetworkTableEntry m_botPoseFieldBlue; // Robot pose on field (Blue origin)
  private final NetworkTableEntry m_activePipeline; // Current pipeline index

  // Configuration
  private boolean m_useLimelightForOdometry = false;

  // Reference to drivetrain for odometry updates (set via setDrivetrain method)
  private CommandSwerveDrivetrain m_drivetrain;

  /** Creates a new LimelightSubsystem */
  public LimelightSubsystem() {
    // Get the limelight NetworkTable
    var limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);

    // Initialize NetworkTable entries
    m_tid = limelightTable.getEntry("tid");
    m_tx = limelightTable.getEntry("tx");
    m_ty = limelightTable.getEntry("ty");
    m_ta = limelightTable.getEntry("ta");
    m_tv = limelightTable.getEntry("tv");
    m_botPose = limelightTable.getEntry("botpose_targetspace");
    m_botPoseFieldBlue = limelightTable.getEntry("botpose_wpiblue");
    m_activePipeline = limelightTable.getEntry("getpipe");

    // Configure Limelight
    LimelightHelpers.setPipelineIndex(
        VisionConstants.LIMELIGHT_NAME, VisionConstants.PIPELINE_APRILTAG);
    LimelightHelpers.setLEDMode_PipelineControl(VisionConstants.LIMELIGHT_NAME);
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
  }

  /**
   * Set the drivetrain reference for vision-based odometry updates
   *
   * @param drivetrain The CommandSwerveDrivetrain instance
   */
  public void setDrivetrain(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  /**
   * Enable or disable using Limelight for odometry updates
   *
   * <p>WATCH OUT! Vision-based pose estimation is now handled directly in CommandSwerveDrivetrain
   * using MegaTag2 with proper standard deviations. This method is kept for compatibility but the
   * subsystem's vision updates are disabled.
   *
   * @param enable True to use Limelight for odometry (currently disabled - use drivetrain instead)
   * @deprecated Vision updates are now handled in CommandSwerveDrivetrain.updateVision()
   */
  @Deprecated
  public void setUseLimelightForOdometry(boolean enable) {
    // Vision updates are now consolidated in CommandSwerveDrivetrain.updateVision()
    // This prevents duplicate vision measurements and ensures consistent std dev
    // calculations
    m_useLimelightForOdometry = false; // Always disabled - drivetrain handles vision
  }

  @Override
  public void periodic() {
    // Update SmartDashboard with vision data
    SmartDashboard.putNumber("Limelight/TX", getTx());
    SmartDashboard.putNumber("Limelight/TY", getTy());
    SmartDashboard.putNumber("Limelight/TA", getTa());
    SmartDashboard.putNumber("Limelight/TID", getTid());
    SmartDashboard.putBoolean("Limelight/HasTarget", hasTarget());
    SmartDashboard.putNumber("Limelight/Yaw", getYaw());

    // WATCH OUT! Vision-based odometry updates are now handled in
    // CommandSwerveDrivetrain.updateVision()
    // This prevents duplicate measurements and ensures proper MegaTag2 integration
    // with:
    // - SetRobotOrientation called before reading pose
    // - Dynamic standard deviations based on tag distance/count
    // - Angular velocity rejection for MegaTag2 accuracy
    // - Field boundary validation
  }

  /** Update drivetrain odometry with Limelight vision measurements */
  private void updateOdometryWithVision() {
    // Get drivetrain state for orientation data
    var driveState = m_drivetrain.getState();
    double headingDeg = driveState.Pose.getRotation().getDegrees();
    double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

    // Set robot orientation in Limelight
    LimelightHelpers.SetRobotOrientation(VisionConstants.LIMELIGHT_NAME, headingDeg, 0, 0, 0, 0, 0);

    // Get pose estimate using MegaTag2
    var llMeasurement =
        LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(VisionConstants.LIMELIGHT_NAME);

    // Only add measurement if valid (has tags and robot isn't spinning too fast)
    if (llMeasurement != null && llMeasurement.tagCount > 0 && omegaRps < 2.0) {
      m_drivetrain.addVisionMeasurement(llMeasurement.pose, llMeasurement.timestampSeconds);
    }
  }

  // ==================== GETTER METHODS ====================

  /** @return The robot pose relative to the target [tx, ty, tz, pitch, yaw, roll] */
  public double[] getBotPose() {
    return m_botPose.getDoubleArray(new double[6]);
  }

  /**
   * @return The robot pose on field from Blue alliance origin [X, Y, Z, Roll, Pitch, Yaw, Latency,
   *     Tag Count, Tag Span, Avg Tag Distance, Avg Tag Area]
   */
  public double[] getBotPoseFieldBlue() {
    return m_botPoseFieldBlue.getDoubleArray(new double[11]);
  }

  /** @return The yaw of the robot relative to the target (degrees) */
  public double getYaw() {
    return getBotPose()[4];
  }

  /** @return Horizontal offset from crosshair to target (degrees) */
  public double getTx() {
    return m_tx.getDouble(0);
  }

  /** @return Vertical offset from crosshair to target (degrees) */
  public double getTy() {
    return m_ty.getDouble(0);
  }

  /** @return Target area as percentage of image (0-100) */
  public double getTa() {
    return m_ta.getDouble(0);
  }

  /** @return The AprilTag ID being tracked (0 if none) */
  public int getTid() {
    return (int) m_tid.getDouble(0);
  }

  /** @return True if a valid target is detected */
  public boolean hasTarget() {
    return m_tv.getDouble(0) == 1.0 && getTid() != 0;
  }

  /** @return True if an AprilTag is detected (alias for hasTarget) */
  public boolean isTagDetected() {
    return hasTarget();
  }

  /** @return Number of tags visible in current frame */
  public double getTagCount() {
    return getBotPoseFieldBlue()[7];
  }

  /** @return Robot pose as Pose2d (X, Y, Yaw on field) */
  public Pose2d getPose() {
    double[] poseData = getBotPoseFieldBlue();
    Rotation2d rotation = new Rotation2d(Math.toRadians(poseData[5]));
    return new Pose2d(poseData[0], poseData[1], rotation);
  }

  /** @return Current active pipeline index */
  public int getActivePipeline() {
    return (int) m_activePipeline.getDouble(0);
  }

  /**
   * Set the Limelight pipeline
   *
   * @param pipelineIndex Pipeline index (0-9)
   */
  public void setPipeline(int pipelineIndex) {
    LimelightHelpers.setPipelineIndex(VisionConstants.LIMELIGHT_NAME, pipelineIndex);
  }

  /** Turn LEDs on */
  public void setLEDsOn() {
    LimelightHelpers.setLEDMode_ForceOn(VisionConstants.LIMELIGHT_NAME);
  }

  /** Turn LEDs off */
  public void setLEDsOff() {
    LimelightHelpers.setLEDMode_ForceOff(VisionConstants.LIMELIGHT_NAME);
  }

  /** Set LEDs to blink */
  public void setLEDsBlink() {
    LimelightHelpers.setLEDMode_ForceBlink(VisionConstants.LIMELIGHT_NAME);
  }
}
