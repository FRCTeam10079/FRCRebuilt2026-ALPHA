package frc.robot.pathfinding;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.List;
import java.util.function.Supplier;

/**
 * Command that pathfinds to a target pose using the AD* algorithm and follows the path using PID
 * control.
 *
 * <p>This command combines pathfinding logic (AD* search on a navgrid) with motion control pattern
 * (PID-based trajectory following).
 *
 * <p>The command: Sets the pathfinding goal (e.g., AprilTag scoring pose) Waits for the AD*
 * background thread to calculate a path Follows the path using pure pursuit / PID control Ends when
 * the robot reaches the goal within tolerance
 */
public class PathfindToTagCommand extends Command {
  private final CommandSwerveDrivetrain m_drivetrain;
  private final Supplier<Pose2d> m_targetPoseSupplier;
  private final PathConstraints m_constraints;

  // PID controllers for path following
  private final PIDController m_xController = new PIDController(3.0, 0.0, 0.1);
  private final PIDController m_yController = new PIDController(3.0, 0.0, 0.1);
  private final PIDController m_rotationController = new PIDController(3.0, 0.0, 0.1);

  // State
  private Pose2d m_targetPose;
  private List<Translation2d> m_currentPath;
  private int m_currentWaypointIndex;
  private final Timer m_pathWaitTimer = new Timer();
  private boolean m_hasPath = false;

  // Debug counter for periodic logging
  private int m_debugCounter = 0;

  // NetworkTables publishers for AdvantageScope visualization
  private final StructPublisher<Pose2d> m_currentPosePublisher;
  private final StructPublisher<Pose2d> m_goalPosePublisher;
  private final StructPublisher<Pose2d> m_targetWaypointPublisher;
  private final StructArrayPublisher<Pose2d> m_pathPublisher;

  // Field2d for visualization in AdvantageScope / Shuffleboard
  private static final Field2d m_pathfindingField = new Field2d();

  static {
    // Publish Field2d once so AdvantageScope can see it
    SmartDashboard.putData("Pathfinding/Field", m_pathfindingField);
  }

  // Pure pursuit parameters
  private static final double LOOKAHEAD_DISTANCE = 0.5; // meters
  private static final double WAYPOINT_TOLERANCE = 0.3; // meters to advance waypoint

  /**
   * Create a command to pathfind to a specific AprilTag.
   *
   * @param drivetrain The swerve drivetrain
   * @param tagId The AprilTag ID to target
   */
  public static PathfindToTagCommand toAprilTag(CommandSwerveDrivetrain drivetrain, int tagId) {
    return new PathfindToTagCommand(
        drivetrain,
        () -> PathfindingConstants.getScoringPoseForTag(tagId),
        PathConstraints.DEFAULT);
  }

  /**
   * Create a command to pathfind to AprilTag 10 (Red Alliance Hub Face).
   *
   * @param drivetrain The swerve drivetrain
   */
  public static PathfindToTagCommand toAprilTag10(CommandSwerveDrivetrain drivetrain) {
    return toAprilTag(drivetrain, 10);
  }

  /**
   * Create a pathfinding command.
   *
   * @param drivetrain The swerve drivetrain
   * @param targetPoseSupplier Supplier for the target pose
   * @param constraints Path following constraints
   */
  public PathfindToTagCommand(
      CommandSwerveDrivetrain drivetrain,
      Supplier<Pose2d> targetPoseSupplier,
      PathConstraints constraints) {

    m_drivetrain = drivetrain;
    m_targetPoseSupplier = targetPoseSupplier;
    m_constraints = constraints;

    m_rotationController.enableContinuousInput(-Math.PI, Math.PI);

    // Initialize NetworkTables publishers for AdvantageScope
    var pathfindingTable = NetworkTableInstance.getDefault().getTable("Pathfinding");
    m_currentPosePublisher =
        pathfindingTable.getStructTopic("CurrentPose", Pose2d.struct).publish();
    m_goalPosePublisher =
        pathfindingTable.getStructTopic("GoalPose", Pose2d.struct).publish();
    m_targetWaypointPublisher =
        pathfindingTable.getStructTopic("TargetWaypoint", Pose2d.struct).publish();
    m_pathPublisher =
        pathfindingTable.getStructArrayTopic("Path", Pose2d.struct).publish();

    addRequirements(m_drivetrain);
  }

  @Override
  public void initialize() {
    // Get target pose
    m_targetPose = m_targetPoseSupplier.get();

    // Get current robot state
    Pose2d currentPose = m_drivetrain.getState().Pose;
    ChassisSpeeds currentSpeeds = m_drivetrain.getState().Speeds;

    // Convert to field-relative velocity
    var fieldSpeeds =
        ChassisSpeeds.fromRobotRelativeSpeeds(currentSpeeds, currentPose.getRotation());
    var velocity = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);

    // Ensure pathfinder is initialized
    Pathfinding.ensureInitialized();

    // Set the pathfinding problem
    Pathfinding.setProblem(currentPose, m_targetPose, velocity);

    // Reset state
    m_currentPath = null;
    m_currentWaypointIndex = 0;
    m_hasPath = false;
    m_pathWaitTimer.restart();

    // Reset PID controllers
    m_xController.reset();
    m_yController.reset();
    m_rotationController.reset();

    System.out.println("[PathfindToTag] Starting pathfind from "
        + formatPose(currentPose)
        + " to "
        + formatPose(m_targetPose));
  }

  @Override
  public void execute() {
    Pose2d currentPose = m_drivetrain.getState().Pose;

    // === PERIODIC DEBUG (every 50 loops ~ 1 second) ===
    m_debugCounter++;
    boolean shouldLogDebug = m_debugCounter % 50 == 1;

    // Check for new path from background thread
    if (Pathfinding.isNewPathAvailable()) {
      m_currentPath = Pathfinding.getCurrentPathWaypoints();
      m_currentWaypointIndex = 0;
      m_hasPath = m_currentPath != null && m_currentPath.size() >= 2;

      if (m_hasPath) {
        System.out.println(
            "[PathfindToTag] Received path with " + m_currentPath.size() + " waypoints");
        // Log all waypoints when path is received
        System.out.println("[PathfindToTag] === PATH WAYPOINTS ===");
        for (int i = 0; i < m_currentPath.size(); i++) {
          Translation2d wp = m_currentPath.get(i);
          System.out.println("  [" + i + "]: (" + String.format("%.3f", wp.getX()) + ", "
              + String.format("%.3f", wp.getY()) + ")");
        }
        System.out.println("==========================");
      } else {
        System.out.println(
            "[PathfindToTag] WARNING: Received invalid path (null or < 2 waypoints)");
      }
    }

    // If no path yet, wait (but NEVER drive directly - that would ignore
    // obstacles!)
    if (!m_hasPath) {
      SmartDashboard.putString("Pathfinding/Status", "Waiting for path...");
      SmartDashboard.putNumber("Pathfinding/WaitTime", m_pathWaitTimer.get());

      // Keep publishing current pose even while waiting - VERY IMPORTANT for
      // visibility!
      m_currentPosePublisher.set(currentPose);
      m_goalPosePublisher.set(m_targetPose);

      // Also publish to Field2d while waiting
      m_pathfindingField.setRobotPose(currentPose);
      m_pathfindingField.getObject("Goal").setPose(m_targetPose);
      m_pathfindingField.getObject("TargetWaypoint").setPose(m_targetPose); // No waypoint yet
      m_pathfindingField.getObject("Path").setPoses(); // Empty path

      // If waiting too long, log a warning but DON'T drive directly (that ignores
      // obstacles)
      if (m_pathWaitTimer.hasElapsed(2.0) && shouldLogDebug) {
        System.out.println("[PathfindToTag] WARNING: Still waiting for path after "
            + String.format("%.1f", m_pathWaitTimer.get()) + "s");
      }
      return;
    }

    // Find current target waypoint using lookahead
    Translation2d targetWaypoint = findLookaheadPoint(currentPose);

    // Log state to SmartDashboard (works with AdvantageScope)
    SmartDashboard.putString("Pathfinding/Status", "Following path");
    SmartDashboard.putNumber("Pathfinding/CurrentWaypoint", m_currentWaypointIndex);
    SmartDashboard.putNumber("Pathfinding/TotalWaypoints", m_currentPath.size());

    // Calculate velocities using PID
    double xVelocity = clampVelocity(
        m_xController.calculate(currentPose.getX(), targetWaypoint.getX()),
        m_constraints.maxVelocityMps());

    double yVelocity = clampVelocity(
        m_yController.calculate(currentPose.getY(), targetWaypoint.getY()),
        m_constraints.maxVelocityMps());

    // Rotation towards goal pose
    double rotationVelocity = clampVelocity(
        m_rotationController.calculate(
            currentPose.getRotation().getRadians(), m_targetPose.getRotation().getRadians()),
        m_constraints.maxAngularVelocityRadPerSec());

    // === DEBUG: Log control outputs ===
    if (shouldLogDebug) {
      System.out.println("\n===== PATH FOLLOWING DEBUG =====");
      System.out.println("[Follow] Current Pose: (" + String.format("%.3f", currentPose.getX())
          + ", " + String.format("%.3f", currentPose.getY()) + ", "
          + String.format("%.1f", currentPose.getRotation().getDegrees()) + "°)");
      System.out.println(
          "[Follow] Target Waypoint: (" + String.format("%.3f", targetWaypoint.getX()) + ", "
              + String.format("%.3f", targetWaypoint.getY()) + ")");
      System.out.println("[Follow] Goal Pose: (" + String.format("%.3f", m_targetPose.getX()) + ", "
          + String.format("%.3f", m_targetPose.getY()) + ", "
          + String.format("%.1f", m_targetPose.getRotation().getDegrees()) + "°)");
      System.out.println("[Follow] Position Error: X="
          + String.format("%.3f", targetWaypoint.getX() - currentPose.getX()) + ", Y="
          + String.format("%.3f", targetWaypoint.getY() - currentPose.getY()));
      System.out.println("[Follow] Velocity Commands (field-relative): vX="
          + String.format("%.3f", xVelocity) + ", vY=" + String.format("%.3f", yVelocity)
          + ", omega=" + String.format("%.3f", rotationVelocity));
      System.out.println("[Follow] Waypoint Index: " + m_currentWaypointIndex + "/"
          + (m_currentPath != null ? m_currentPath.size() : 0));
      System.out.println("================================\n");
    }

    // Apply to drivetrain as field-relative speeds
    ChassisSpeeds fieldSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    ChassisSpeeds robotSpeeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

    // === DEBUG: Log robot-relative speeds being sent ===
    if (shouldLogDebug) {
      System.out.println("[Follow] Robot-Relative Speeds: vX="
          + String.format("%.3f", robotSpeeds.vxMetersPerSecond) + ", vY="
          + String.format("%.3f", robotSpeeds.vyMetersPerSecond) + ", omega="
          + String.format("%.3f", robotSpeeds.omegaRadiansPerSecond));
    }

    // === NETWORKTABLES LOGGING FOR ADVANTAGESCOPE ===
    // Publish poses for 2D field visualization
    m_currentPosePublisher.set(currentPose);
    m_goalPosePublisher.set(m_targetPose);
    m_targetWaypointPublisher.set(new Pose2d(targetWaypoint, m_targetPose.getRotation()));

    // === FIELD2D VISUALIZATION (works with AdvantageScope/Shuffleboard) ===
    // Update robot pose on field
    m_pathfindingField.setRobotPose(currentPose);
    // Goal pose shown as a separate object
    m_pathfindingField.getObject("Goal").setPose(m_targetPose);
    // Target waypoint shown as another object
    m_pathfindingField
        .getObject("TargetWaypoint")
        .setPose(new Pose2d(targetWaypoint, m_targetPose.getRotation()));
    // Full path trajectory
    if (m_currentPath != null && !m_currentPath.isEmpty()) {
      Pose2d[] pathPoses = new Pose2d[m_currentPath.size()];
      for (int i = 0; i < m_currentPath.size(); i++) {
        pathPoses[i] = new Pose2d(m_currentPath.get(i), m_targetPose.getRotation());
      }
      m_pathfindingField.getObject("Path").setPoses(pathPoses);
    } else {
      m_pathfindingField.getObject("Path").setPoses();
    }

    // Publish velocity data to SmartDashboard
    SmartDashboard.putNumber("Pathfinding/VelX", xVelocity);
    SmartDashboard.putNumber("Pathfinding/VelY", yVelocity);
    SmartDashboard.putNumber("Pathfinding/VelOmega", rotationVelocity);
    SmartDashboard.putNumber("Pathfinding/ErrorX", targetWaypoint.getX() - currentPose.getX());
    SmartDashboard.putNumber("Pathfinding/ErrorY", targetWaypoint.getY() - currentPose.getY());
    SmartDashboard.putNumber("Pathfinding/CurrentWaypointIdx", m_currentWaypointIndex);
    SmartDashboard.putNumber(
        "Pathfinding/PathLength", m_currentPath != null ? m_currentPath.size() : 0);

    // Publish path for raw struct visualization too
    if (m_currentPath != null && !m_currentPath.isEmpty()) {
      Pose2d[] pathPoses = new Pose2d[m_currentPath.size()];
      for (int i = 0; i < m_currentPath.size(); i++) {
        pathPoses[i] = new Pose2d(m_currentPath.get(i), m_targetPose.getRotation());
      }
      m_pathPublisher.set(pathPoses);
    } else {
      // Publish empty array when no path to clear stale data
      m_pathPublisher.set(new Pose2d[0]);
    }

    m_drivetrain.setControl(
        new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));

    // Advance waypoint if close enough
    advanceWaypointIfNeeded(currentPose);

    // Update pathfinder with current pose for dynamic replanning
    Pathfinding.setStartPose(currentPose);
  }

  @Override
  public void end(boolean interrupted) {
    // Stop the robot
    m_drivetrain.setControl(new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds()
        .withSpeeds(new ChassisSpeeds()));

    if (interrupted) {
      System.out.println("[PathfindToTag] Command interrupted");
    } else {
      System.out.println("[PathfindToTag] Reached goal: " + formatPose(m_targetPose));
    }
  }

  @Override
  public boolean isFinished() {
    Pose2d currentPose = m_drivetrain.getState().Pose;

    // Check position tolerance
    double positionError = AlignToAprilTag.calculateDistance(currentPose, m_targetPose);
    boolean positionOk = positionError < PathfindingConstants.POSITION_TOLERANCE_METERS;

    // Check rotation tolerance
    double rotationError =
        Math.abs(currentPose.getRotation().minus(m_targetPose.getRotation()).getRadians());
    boolean rotationOk = rotationError < PathfindingConstants.ROTATION_TOLERANCE_RADIANS;

    return positionOk && rotationOk;
  }

  // ==================== Path Following Helpers ====================

  private Translation2d findLookaheadPoint(Pose2d currentPose) {
    if (m_currentPath == null || m_currentPath.isEmpty()) {
      return m_targetPose.getTranslation();
    }

    Translation2d robotPos = currentPose.getTranslation();

    // Start from current waypoint index
    for (int i = m_currentWaypointIndex; i < m_currentPath.size(); i++) {
      Translation2d waypoint = m_currentPath.get(i);
      double distance = robotPos.getDistance(waypoint);

      if (distance >= LOOKAHEAD_DISTANCE) {
        // Interpolate to exact lookahead distance
        if (i > 0) {
          Translation2d prev = m_currentPath.get(i - 1);
          Translation2d direction = waypoint.minus(prev);
          double segmentLength = direction.getNorm();
          if (segmentLength > 0.01) {
            // Find point on segment at lookahead distance from robot
            Translation2d toRobot = robotPos.minus(prev);
            double projection =
                toRobot.getX() * direction.getX() + toRobot.getY() * direction.getY();
            projection /= segmentLength * segmentLength;
            projection = Math.max(0, Math.min(1, projection));
            Translation2d closestPoint = prev.plus(direction.times(projection));

            // Move lookahead distance along segment from closest point
            Translation2d segmentDir = direction.div(segmentLength);
            return closestPoint.plus(segmentDir.times(LOOKAHEAD_DISTANCE));
          }
        }
        return waypoint;
      }
    }

    // Return final goal
    return m_targetPose.getTranslation();
  }

  private void advanceWaypointIfNeeded(Pose2d currentPose) {
    if (m_currentPath == null || m_currentWaypointIndex >= m_currentPath.size() - 1) {
      return;
    }

    Translation2d waypoint = m_currentPath.get(m_currentWaypointIndex);
    double distance = currentPose.getTranslation().getDistance(waypoint);

    if (distance < WAYPOINT_TOLERANCE) {
      m_currentWaypointIndex++;
    }
  }

  private void driveTowardsPose(Pose2d currentPose, Pose2d goal) {
    double xVelocity = clampVelocity(
        m_xController.calculate(currentPose.getX(), goal.getX()),
        m_constraints.maxVelocityMps() * 0.3);

    double yVelocity = clampVelocity(
        m_yController.calculate(currentPose.getY(), goal.getY()),
        m_constraints.maxVelocityMps() * 0.3);

    double rotationVelocity = clampVelocity(
        m_rotationController.calculate(
            currentPose.getRotation().getRadians(), goal.getRotation().getRadians()),
        m_constraints.maxAngularVelocityRadPerSec() * 0.3);

    var fieldSpeeds = new ChassisSpeeds(xVelocity, yVelocity, rotationVelocity);
    var robotSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldSpeeds, currentPose.getRotation());

    m_drivetrain.setControl(
        new com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds().withSpeeds(robotSpeeds));
  }

  private double clampVelocity(double velocity, double max) {
    return Math.max(-max, Math.min(max, velocity));
  }

  private String formatPose(Pose2d pose) {
    return String.format(
        "(%.2f, %.2f, %.1f°)", pose.getX(), pose.getY(), pose.getRotation().getDegrees());
  }
}
