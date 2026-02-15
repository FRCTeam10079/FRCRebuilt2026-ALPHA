// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.*;
import frc.robot.RobotStateMachine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LimelightSubsystem;

/**
 * Command to align the robot to an AprilTag using vision
 *
 * <p>This command: 1. Finds the closest AprilTag (or uses Limelight-detected tag) 2. Calculates
 * target position with optional offset (LEFT/RIGHT/CENTER) 3. Uses PID control to drive the robot
 * to the target pose 4. Rotates to face opposite the tag (facing the tag)
 *
 * <p>For REBUILT 2026 - Generic AprilTag alignment for any field element
 */
public class AlignToAprilTag extends Command {
  // Subsystems
  private final CommandSwerveDrivetrain m_drivetrain;
  private final LimelightSubsystem m_limelight;
  private final RobotStateMachine m_stateMachine;

  // Timer for logging/timeout
  private final Timer m_timer = new Timer();

  // PID Controllers for position control
  private final PIDController m_pidX = new PIDController(
      DrivetrainConstants.ALIGN_PID_KP,
      DrivetrainConstants.ALIGN_PID_KI,
      DrivetrainConstants.ALIGN_PID_KD);
  private final PIDController m_pidY = new PIDController(
      DrivetrainConstants.ALIGN_PID_KP,
      DrivetrainConstants.ALIGN_PID_KI,
      DrivetrainConstants.ALIGN_PID_KD);
  private final PIDController m_pidRotate = new PIDController(
      DrivetrainConstants.ALIGN_ROTATION_KP, 0, DrivetrainConstants.ALIGN_ROTATION_KD);

  // Target pose to align to
  private Pose2d m_targetPose;

  // Offset configuration
  private final AlignPosition m_alignPosition;
  private double m_offsetX = 0;
  private double m_offsetY = 0;

  // Target AprilTag info
  private int m_targetTagID;
  private boolean m_tagDetected = false;

  /**
   * Creates a new AlignToAprilTag command
   *
   * @param drivetrain The swerve drivetrain subsystem
   * @param limelight The limelight vision subsystem
   * @param alignPosition LEFT, RIGHT, or CENTER offset from the AprilTag
   */
  public AlignToAprilTag(
      CommandSwerveDrivetrain drivetrain,
      LimelightSubsystem limelight,
      AlignPosition alignPosition) {
    m_drivetrain = drivetrain;
    m_limelight = limelight;
    m_stateMachine = RobotStateMachine.getInstance();
    m_alignPosition = alignPosition;

    // Enable continuous input for rotation (-PI to PI are same point)
    m_pidRotate.enableContinuousInput(-Math.PI, Math.PI);

    // This command requires the drivetrain
    addRequirements(m_drivetrain);

    System.out.println("[AlignToAprilTag] Created for " + m_alignPosition + " position");
  }

  /** Convenience constructor for CENTER alignment */
  public AlignToAprilTag(CommandSwerveDrivetrain drivetrain, LimelightSubsystem limelight) {
    this(drivetrain, limelight, AlignPosition.CENTER);
  }

  @Override
  public void initialize() {
    // Start timer
    m_timer.restart();

    // Set state machine to vision tracking mode
    m_stateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.VISION_TRACKING);
    m_stateMachine.setAlignedToTarget(false);

    // Get current robot pose
    Pose2d robotPose = m_drivetrain.getState().Pose;
    if (robotPose == null) {
      System.out.println("[AlignToAprilTag] ERROR: Robot pose is null!");
      m_tagDetected = false;
      return;
    }

    // Find the closest AprilTag using odometry
    double minDistance = Double.MAX_VALUE;
    m_targetTagID = -1;

    for (int id : AprilTagMaps.aprilTagMap.keySet()) {
      double[] tagData = AprilTagMaps.aprilTagMap.get(id);
      var tagPose = new Pose2d(
          tagData[0] * Constants.INCHES_TO_METERS,
          tagData[1] * Constants.INCHES_TO_METERS,
          new Rotation2d(Math.toRadians(tagData[3])));

      double distance = calculateDistance(robotPose, tagPose);
      if (distance < minDistance) {
        minDistance = distance;
        m_targetTagID = id;
      }
    }

    // Check if a tag was found
    if (m_targetTagID == -1) {
      System.out.println("[AlignToAprilTag] ERROR: No AprilTag found in map!");
      m_tagDetected = false;
      return;
    }

    System.out.println("[AlignToAprilTag] Closest tag from odometry: " + m_targetTagID);

    // Check if Limelight sees a valid tag - prefer it over odometry
    int limelightTagID = m_limelight.getTid();
    if (limelightTagID != 0 && AprilTagMaps.aprilTagMap.containsKey(limelightTagID)) {
      m_targetTagID = limelightTagID;
      System.out.println("[AlignToAprilTag] Using Limelight tag: " + m_targetTagID);
    } else if (limelightTagID == 0) {
      System.out.println("[AlignToAprilTag] Limelight has no target, using odometry closest tag: "
          + m_targetTagID);
    } else {
      System.out.println("[AlignToAprilTag] Limelight tag "
          + limelightTagID
          + " not in map, using odometry: "
          + m_targetTagID);
    }

    // Get tag data
    double[] tagData = AprilTagMaps.aprilTagMap.get(m_targetTagID);
    if (tagData == null) {
      System.out.println("[AlignToAprilTag] ERROR: Tag data is null for ID: " + m_targetTagID);
      m_tagDetected = false;
      return;
    }

    // Convert tag position to Pose2d
    var aprilTagPose = new Pose2d(
        tagData[0] * Constants.INCHES_TO_METERS,
        tagData[1] * Constants.INCHES_TO_METERS,
        new Rotation2d(Math.toRadians(tagData[3])));

    m_tagDetected = true;

    // Calculate offset based on alignment position
    // Offsets are relative to the tag's coordinate frame
    switch (m_alignPosition) {
      case LEFT:
        m_offsetX = DrivetrainConstants.ALIGN_OFFSET_X_LEFT;
        m_offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_LEFT;
        break;
      case RIGHT:
        m_offsetX = DrivetrainConstants.ALIGN_OFFSET_X_RIGHT;
        m_offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_RIGHT;
        break;
      case CENTER:
      default:
        m_offsetX = DrivetrainConstants.ALIGN_OFFSET_X_CENTER;
        m_offsetY = DrivetrainConstants.ALIGN_OFFSET_Y_CENTER;
        break;
    }

    // Target rotation is opposite the tag (facing the tag)
    double targetRotation = aprilTagPose.getRotation().getRadians() - Math.PI;
    targetRotation = MathUtil.angleModulus(targetRotation);

    // Rotate the offset from tag-relative to field-relative
    double rotatedOffsetX =
        (m_offsetX * Math.cos(targetRotation)) - (m_offsetY * Math.sin(targetRotation));
    double rotatedOffsetY =
        (m_offsetX * Math.sin(targetRotation)) + (m_offsetY * Math.cos(targetRotation));

    // Calculate final target pose
    m_targetPose = new Pose2d(
        aprilTagPose.getX() + rotatedOffsetX,
        aprilTagPose.getY() + rotatedOffsetY,
        new Rotation2d(targetRotation));

    // Set PID setpoints
    m_pidX.setSetpoint(m_targetPose.getX());
    m_pidY.setSetpoint(m_targetPose.getY());
    m_pidRotate.setSetpoint(m_targetPose.getRotation().getRadians());

    System.out.println("[AlignToAprilTag] Target Pose: X="
        + m_targetPose.getX()
        + ", Y="
        + m_targetPose.getY()
        + ", Yaw="
        + Math.toDegrees(m_targetPose.getRotation().getRadians())
        + "°");

    // Log to SmartDashboard
    SmartDashboard.putNumber("AlignToAprilTag/TargetTagID", m_targetTagID);
    SmartDashboard.putNumber("AlignToAprilTag/TargetX", m_targetPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/TargetY", m_targetPose.getY());
    SmartDashboard.putNumber(
        "AlignToAprilTag/TargetYaw", m_targetPose.getRotation().getDegrees());
  }

  @Override
  public void execute() {
    // If no tag detected, don't execute
    if (!m_tagDetected) {
      return;
    }

    // Get current robot pose
    Pose2d currentPose = m_drivetrain.getState().Pose;
    if (currentPose == null) {
      return;
    }

    // Calculate velocities using PID
    double[] velocities = calculatePIDVelocities(currentPose);

    // Log data
    SmartDashboard.putNumber("AlignToAprilTag/CurrentX", currentPose.getX());
    SmartDashboard.putNumber("AlignToAprilTag/CurrentY", currentPose.getY());
    SmartDashboard.putNumber(
        "AlignToAprilTag/CurrentYaw", currentPose.getRotation().getDegrees());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorX", m_pidX.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorY", m_pidY.getError());
    SmartDashboard.putNumber("AlignToAprilTag/ErrorYaw", Math.toDegrees(m_pidRotate.getError()));

    // Check if we need to flip direction based on alliance side
    // Red side tags (6-11) may need direction adjustment
    boolean isRedSide = Constants.contains(new int[] {6, 7, 8, 9, 10, 11}, m_targetTagID);

    // Apply velocities to drivetrain
    m_drivetrain.setControl(new SwerveRequest.FieldCentric()
        .withDriveRequestType(DriveRequestType.Velocity)
        .withVelocityX(velocities[0])
        .withVelocityY(velocities[1])
        .withRotationalRate(velocities[2]));
  }

  /** Calculate velocities using PID control */
  private double[] calculatePIDVelocities(Pose2d currentPose) {
    // Calculate X velocity
    double velocityX = m_pidX.calculate(currentPose.getX());
    velocityX = MathUtil.clamp(
        velocityX, -DrivetrainConstants.ALIGN_SPEED_MPS, DrivetrainConstants.ALIGN_SPEED_MPS);

    // Calculate Y velocity
    double velocityY = m_pidY.calculate(currentPose.getY());
    velocityY = MathUtil.clamp(
        velocityY, -DrivetrainConstants.ALIGN_SPEED_MPS, DrivetrainConstants.ALIGN_SPEED_MPS);

    // Calculate rotation velocity
    double velocityYaw = m_pidRotate.calculate(currentPose.getRotation().getRadians());
    velocityYaw = MathUtil.clamp(velocityYaw, -2.0, 2.0);

    return new double[] {velocityX, velocityY, velocityYaw};
  }

  /** Calculate distance between two poses */
  public static double calculateDistance(Pose2d pose1, Pose2d pose2) {
    return pose1.getTranslation().getDistance(pose2.getTranslation());
  }

  @Override
  public boolean isFinished() {
    // End if no tag detected
    if (!m_tagDetected) {
      return true;
    }

    // Get current pose
    Pose2d currentPose = m_drivetrain.getState().Pose;
    if (currentPose == null || m_targetPose == null) {
      return true;
    }

    // Calculate position and yaw error
    double distance = calculateDistance(m_targetPose, currentPose);
    double yawError = Math.abs(MathUtil.angleModulus(
        m_targetPose.getRotation().getRadians() - currentPose.getRotation().getRadians()));

    // Check if within tolerance
    boolean positionReached = distance <= DrivetrainConstants.POSITION_TOLERANCE_METERS;
    boolean yawReached = yawError <= DrivetrainConstants.YAW_TOLERANCE_RADIANS;

    SmartDashboard.putNumber("AlignToAprilTag/Distance", distance);
    SmartDashboard.putBoolean("AlignToAprilTag/PositionReached", positionReached);
    SmartDashboard.putBoolean("AlignToAprilTag/YawReached", yawReached);

    return positionReached && yawReached;
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the drivetrain
    m_drivetrain.setControl(
        new SwerveRequest.FieldCentric().withDriveRequestType(DriveRequestType.Velocity));

    // Return to field-centric drive mode
    m_stateMachine.setDrivetrainMode(RobotStateMachine.DrivetrainMode.FIELD_CENTRIC);

    // Set alignment status based on completion
    if (!interrupted && m_tagDetected) {
      m_stateMachine.setAlignedToTarget(true);
      System.out.println(
          "[AlignToAprilTag] Completed successfully - aligned to tag " + m_targetTagID);
    } else {
      m_stateMachine.setAlignedToTarget(false);
      System.out.println("[AlignToAprilTag] "
          + (interrupted ? "Interrupted" : "Failed")
          + " - alignment not confirmed");
    }

    // Log completion
    SmartDashboard.putBoolean("AlignToAprilTag/Completed", !interrupted);
    SmartDashboard.putNumber("AlignToAprilTag/Duration", m_timer.get());
  }
}
