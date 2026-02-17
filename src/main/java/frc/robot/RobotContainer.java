// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AlignPosition;
import frc.robot.commands.AlignToAprilTag;
import frc.robot.commands.RunIndexer;
import frc.robot.generated.TunerConstants;
import frc.robot.pathfinding.Pathfinding;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.PivotIntake.IntakeWheelsSubsystem;
import frc.robot.subsystems.PivotIntake.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * RobotContainer for FRC 2026 REBUILT season This class is where the robot's subsystems, commands,
 * and button bindings are defined.
 */
public class RobotContainer {

  // Controllers
  private final CommandXboxController m_driverController = new CommandXboxController(0);
  private final CommandXboxController m_operatorController = new CommandXboxController(1);

  // State Machine
  private final RobotStateMachine m_stateMachine = RobotStateMachine.getInstance();

  // ==================== SUBSYSTEMS ====================
  // Drivetrain - created from TunerConstants
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  // Vision
  public final LimelightSubsystem limelight = new LimelightSubsystem();
  // Indexer
  private final IndexerSubsystem indexer = new IndexerSubsystem();

  // Mechanisms

  public final ShooterSubsystem shooter = new ShooterSubsystem();
  // Intake
  private final IntakeWheelsSubsystem intake = new IntakeWheelsSubsystem();
  private final PivotSubsystem pivot = new PivotSubsystem();

  private final Telemetry m_telemetry =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  public RobotContainer() {
    // Link limelight to drivetrain for vision-based odometry
    limelight.setDrivetrain(drivetrain);

    drivetrain.registerTelemetry(m_telemetry::telemeterize);

    // Register controllers with state machine for haptic feedback
    m_stateMachine.registerControllers(m_driverController, m_operatorController);

    // Initialize the pathfinding system
    initializePathfinding();

    // Configure button bindings
    configureBindings();
  }

  /**
   * Initialize the pathfinding system. This loads the navgrid and starts the background AD*
   * planning thread.
   */
  private void initializePathfinding() {
    System.out.println("[RobotContainer] Initializing pathfinding system...");
    Pathfinding.ensureInitialized();
    System.out.println("[RobotContainer] Pathfinding system ready");
  }

  /**
   * Configure button bindings for driver and operator controllers This is where you bind controller
   * buttons to commands
   */
  private void configureBindings() {
    // ==================== DRIVER CONTROLS ====================
    drivetrain.setDefaultCommand(drivetrain.smoothTeleopDriveCommand(
        () -> m_driverController.getLeftY(), // Forward/backward
        () -> m_driverController.getLeftX(), // Left/right strafe
        () -> m_driverController.getRightX(), // Rotation
        Constants.DrivetrainConstants.MAX_SPEED_MPS,
        Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // ==================== VISION ALIGNMENT ====================
    // A button - Align to AprilTag (CENTER position)
    m_driverController
        .a()
        .whileTrue(new AlignToAprilTag(drivetrain, limelight, AlignPosition.CENTER));

    // Left Bumper - Align to AprilTag (LEFT position)
    m_driverController
        .leftBumper()
        .whileTrue(new AlignToAprilTag(drivetrain, limelight, AlignPosition.LEFT));

    // Right Bumper - Align to AprilTag (RIGHT position)
    m_driverController
        .rightBumper()
        .whileTrue(new AlignToAprilTag(drivetrain, limelight, AlignPosition.RIGHT));

    // Y button - Reset Heading
    m_driverController.y().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    // Right Trigger - Run Indexer Forward (Intake/Feed)

    m_driverController
        .rightTrigger(0.5)
        .whileTrue(new RunIndexer(indexer, Constants.IndexerConstants.kForwardSpeed)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    // B Button - Run Indexer Reverse (Unjam)

    m_driverController
        .b()
        .whileTrue(new RunIndexer(indexer, Constants.IndexerConstants.kReverseSpeed)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));
    // ==================== SLOW MODE ====================
    // Left trigger - Hold for slow mode (useful for precise positioning/scoring)
    m_driverController
        .leftTrigger(0.5)
        .whileTrue(Commands.startEnd(
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.SLOW_MODE_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.SLOW_MODE_COEFFICIENT);
            },
            () -> {
              drivetrain.setTeleopVelocityCoefficient(
                  Constants.DrivetrainConstants.NORMAL_SPEED_COEFFICIENT);
              drivetrain.setRotationVelocityCoefficient(
                  Constants.DrivetrainConstants.NORMAL_SPEED_COEFFICIENT);
            }));

    // ==================== OPERATOR CONTROLS ====================
    // TODO: Add intake controls
    // Intake In
    m_operatorController
        .x()
        .toggleOnTrue(new StartEndCommand(() -> intake.intakeIn(), () -> intake.stop(), intake));
    // Intake Out
    m_operatorController
        .a()
        .toggleOnTrue(new StartEndCommand(() -> intake.intakeOut(), () -> intake.stop(), intake));
    // Deploy Pivot Controls
    m_operatorController
        .rightBumper()
        .toggleOnTrue(new StartEndCommand(
            () -> pivot.deployPivot(), // action when button pressed
            () -> {}, // nothing special on release
            pivot));
    // Stow Pivot Controls
    m_operatorController
        .leftBumper()
        .toggleOnTrue(new StartEndCommand(
            () -> pivot.stowPivot(), // action when button pressed
            () -> {}, // nothing special on release
            pivot));
    // TODO: Add climb controls

    // ==================== PATHFINDING CONTROLS ====================
    // X button - Pathfind to AprilTag 10 (Red Alliance Hub Face)
    // Uses AD* algorithm to find safe path around obstacles
    m_driverController.x().whileTrue(drivetrain.pathfindToAprilTag10());

    // B button - Pathfind to AprilTag 18 (Blue Alliance Hub Face)
    // Alternative hub for testing/blue alliance
    m_driverController.b().whileTrue(drivetrain.pathfindToAprilTag(18));

    // ==================== OPERATOR (TEST) CONTROLS ====================
    // Heading Lock to 0 degrees
    // Hold X to lock heading to 0 degrees (facing opponent alliance wall)
    m_operatorController
        .x()
        .whileTrue(drivetrain.headingLockedDriveCommand(
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            0.0, // Lock to 0 degrees
            Constants.DrivetrainConstants.MAX_SPEED_MPS,
            Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // Heading Lock to face AprilTag
    // Hold right trigger to lock heading toward visible AprilTag
    // Uses limelight TX to compute target heading dynamically
    m_operatorController
        .rightTrigger(0.5)
        .whileTrue(drivetrain.headingLockedDriveCommand(
            () -> m_driverController.getLeftY(),
            () -> m_driverController.getLeftX(),
            () -> computeAprilTagHeading(), // Dynamic heading from Limelight
            Constants.DrivetrainConstants.MAX_SPEED_MPS,
            Constants.DrivetrainConstants.MAX_ANGULAR_RATE_RAD_PER_SEC));

    // Shooter Spin-Up Test
    // Hold Left Trigger to spin up shooter - controller will rumble when stable
    // This is so that we can test the debounced isReady() logic
    m_operatorController
        .leftTrigger()
        .whileTrue(shooter.holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM));

    // Trigger-based rumble: rumble controller when shooter is ready
    new Trigger(shooter::isReady)
        .and(m_operatorController.leftTrigger()) // Only rumble while Left Trigger is held
        .onTrue(Commands.runOnce(
            () -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.8)))
        .onFalse(Commands.runOnce(
            () -> m_operatorController.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    // ==================== STATE MACHINE EXAMPLES ====================
    // Example: Manual state transitions (add your actual bindings)
    // m_driverController.y().onTrue(Commands.runOnce(() ->
    // m_stateMachine.setGameState(RobotStateMachine.GameState.AIMING_AT_HUB)));

    // Example: Hub shift state can be set based on FMS data or operator input
    // m_operatorController.start().onTrue(Commands.runOnce(() ->
    // m_stateMachine.setHubShiftState(RobotStateMachine.HubShiftState.MY_HUB_ACTIVE)));
  }

  /** Get the driver controller for use in commands/subsystems */
  public CommandXboxController getDriverController() {
    return m_driverController;
  }

  /** Get the operator controller for use in commands/subsystems */
  public CommandXboxController getOperatorController() {
    return m_operatorController;
  }

  /** Get the state machine instance */
  public RobotStateMachine getStateMachine() {
    return m_stateMachine;
  }

  /**
   * Returns the autonomous command to run during autonomous period. Uses pathfinding to navigate to
   * AprilTag 10 (Red Alliance Hub).
   */
  public Command getAutonomousCommand() {
    // Set pathfinding to use autonomous obstacles (tighter margins)
    Pathfinding.setAutoObstacles();

    // Return pathfinding command to AprilTag 10
    return drivetrain.pathfindToAprilTag10().withName("Auto: Pathfind to Tag 10");
  }

  /**
   * Compute the target heading to face the currently visible AprilTag.
   *
   * <p>If a tag is visible, returns current heading - TX (to center the tag). If no tag visible,
   * returns the current heading (maintain position).
   *
   * <p>This is used by the heading lock test to dynamically track AprilTags.
   */
  private double computeAprilTagHeading() {
    if (limelight.hasTarget()) {
      // Target heading = current heading - TX (TX positive means target to the right)
      double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
      double tx = limelight.getTx();
      double targetHeading = currentHeading - tx;
      return MathUtil.inputModulus(targetHeading, -180.0, 180.0);
    } else {
      // No tag visible - maintain current heading
      double currentHeading = drivetrain.getState().Pose.getRotation().getDegrees();
      return MathUtil.inputModulus(currentHeading, -180.0, 180.0);
    }
  }
}
