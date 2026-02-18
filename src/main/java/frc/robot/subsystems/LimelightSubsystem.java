// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
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

  // NetworkTable for Limelight data
  private final NetworkTable limelightTable;

  // Basic targeting data entries
  private final NetworkTableEntry tid; // Target AprilTag ID
  private final NetworkTableEntry tx; // Horizontal offset from crosshair (degrees)
  private final NetworkTableEntry ty; // Vertical offset from crosshair (degrees)
  private final NetworkTableEntry ta; // Target area (0-100% of image)
  private final NetworkTableEntry tv; // Valid target exists (0 or 1)

  // Pose data entries
  private final NetworkTableEntry botPose; // Robot pose relative to target
  private final NetworkTableEntry botPoseFieldBlue; // Robot pose on field (Blue origin)
  private final NetworkTableEntry activePipeline; // Current pipeline index

  // Reference to drivetrain (used by other subsystems for alignment)
  private CommandSwerveDrivetrain drivetrain;

  /** Creates a new LimelightSubsystem */
  public LimelightSubsystem() {
    // Get the limelight NetworkTable
    limelightTable = NetworkTableInstance.getDefault().getTable(VisionConstants.LIMELIGHT_NAME);

    // Initialize NetworkTable entries
    tid = limelightTable.getEntry("tid");
    tx = limelightTable.getEntry("tx");
    ty = limelightTable.getEntry("ty");
    ta = limelightTable.getEntry("ta");
    tv = limelightTable.getEntry("tv");
    botPose = limelightTable.getEntry("botpose_targetspace");
    botPoseFieldBlue = limelightTable.getEntry("botpose_wpiblue");
    activePipeline = limelightTable.getEntry("getpipe");

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
    this.drivetrain = drivetrain;
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

  // ==================== GETTER METHODS ====================

  /** @return The robot pose relative to the target [tx, ty, tz, pitch, yaw, roll] */
  public double[] getBotPose() {
    return botPose.getDoubleArray(new double[6]);
  }

  /**
   * @return The robot pose on field from Blue alliance origin [X, Y, Z, Roll, Pitch, Yaw, Latency,
   *     Tag Count, Tag Span, Avg Tag Distance, Avg Tag Area]
   */
  public double[] getBotPoseFieldBlue() {
    return botPoseFieldBlue.getDoubleArray(new double[11]);
  }

  /** @return The yaw of the robot relative to the target (degrees) */
  public double getYaw() {
    return getBotPose()[4];
  }

  /** @return Horizontal offset from crosshair to target (degrees) */
  public double getTx() {
    return tx.getDouble(0);
  }

  /** @return Vertical offset from crosshair to target (degrees) */
  public double getTy() {
    return ty.getDouble(0);
  }

  /** @return Target area as percentage of image (0-100) */
  public double getTa() {
    return ta.getDouble(0);
  }

  /** @return The AprilTag ID being tracked (0 if none) */
  public int getTid() {
    return (int) tid.getDouble(0);
  }

  /** @return True if a valid target is detected */
  public boolean hasTarget() {
    return tv.getDouble(0) == 1.0 && getTid() != 0;
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
    return (int) activePipeline.getDouble(0);
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
