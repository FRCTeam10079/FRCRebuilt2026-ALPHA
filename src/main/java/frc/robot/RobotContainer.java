// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
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
  public final IntakeSubsystem intake = new IntakeSubsystem();
  public final ShooterSubsystem shooter = new ShooterSubsystem();

  private final Telemetry m_telemetry =
      new Telemetry(TunerConstants.kSpeedAt12Volts.in(MetersPerSecond));

  // ==================== AUTO CHOOSER ====================
  private final SendableChooser<Command> autoChooser;
  private final SendableChooser<String> choreoChooser = new SendableChooser<>();
  private final AutoFactory choreoAutoFactory;

  public RobotContainer() {
    // Link limelight to drivetrain for vision-based odometry
    limelight.setDrivetrain(drivetrain);

    drivetrain.registerTelemetry(m_telemetry::telemeterize);

    // Register controllers with state machine for haptic feedback
    m_stateMachine.registerControllers(m_driverController, m_operatorController);

    // Initialize the pathfinding system
    initializePathfinding();

    choreoAutoFactory = new AutoFactory(
        () -> drivetrain.getState().Pose,
        drivetrain::resetPose,
        drivetrain::followTrajectory,
        true,
        drivetrain);

    // ==================== REGISTER NAMED COMMANDS ====================
    // These MUST be registered BEFORE any PathPlanner autos/paths are created.
    // Named commands are referenced by name in PathPlanner GUI auto files.
    registerNamedCommands();
    registerChoreoBindings();

    // ==================== BUILD AUTO CHOOSER ====================
    // Build a SendableChooser populated with all PathPlanner autos in the
    // deploy/pathplanner/autos directory. Falls back to AD* pathfind if no
    // PathPlanner autos exist yet.
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureChoreoChooser();
    SmartDashboard.putData("Choreo Auto", choreoChooser);

    // Configure button bindings
    configureBindings();
  }

  private void configureChoreoChooser() {
    choreoChooser.setDefaultOption("RS", "RS");
    choreoChooser.addOption("LS_Depot", "LS_Depot");
    choreoChooser.addOption("LS_Neutral", "LS_Neutral");
    choreoChooser.addOption("MS_Depot_Climb", "MS_Depot_Climb");
    choreoChooser.addOption("None", "");
  }

  /**
   * Register all named commands for PathPlanner autonomous routines. These commands are triggered
   * by event markers and command groups in PathPlanner auto files.
   *
   * <p>IMPORTANT: Must be called BEFORE any PathPlannerAuto or AutoBuilder.buildAutoChooser()
   * calls.
   */
  private void registerNamedCommands() {
    // Intake: run intake motors to collect game pieces
    NamedCommands.registerCommand(
        "intake", Commands.startEnd(() -> intake.intakeIn(), () -> intake.stop(), intake));

    // Indexer: feed game pieces forward

    NamedCommands.registerCommand(
        "runIndexer",
        new RunIndexer(
            indexer,
            Constants.IndexerConstants.kFeederTargetRPM,
            Constants.IndexerConstants.kSpindexerTargetRPM));

    // Indexer: reverse to unjam

    NamedCommands.registerCommand(
        "reverseIndexer",
        new RunIndexer(
            indexer,
            Constants.IndexerConstants.kFeederReverseRPM,
            Constants.IndexerConstants.kSpindexerReverseRPM));

    // Shooter: spin up flywheel to target RPM and hold
    NamedCommands.registerCommand(
        "spinUpShooter", shooter.holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM));

    // Shoot: spin up shooter, wait until ready, then feed with indexer
    // FIXED: Updated inner RunIndexer to use RPM constants
    NamedCommands.registerCommand(
        "shoot",
        shooter
            .holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM)
            .alongWith(Commands.waitUntil(shooter::isReady)
                .andThen(new RunIndexer(
                        indexer,
                        Constants.IndexerConstants.kFeederTargetRPM,
                        Constants.IndexerConstants.kSpindexerTargetRPM)
                    .withTimeout(1.0))));

    // Stop all mechanisms
    NamedCommands.registerCommand("stopAll", Commands.runOnce(() -> {
      intake.stop();
      indexer.stop();
      // Shooter stop is handled by ending the holdRPM command
    }));
    System.out.println("[RobotContainer] Named commands registered for PathPlanner");
  }

  /**
   * Register Choreo global marker bindings for subsystem actions.
   *
   * <p>These bindings are evaluated from event markers inside Choreo trajectories when using
   * AutoRoutine APIs.
   */
  private void registerChoreoBindings() {
    // FIXED: All RunIndexer calls updated below
    choreoAutoFactory
        .bind("intake", Commands.startEnd(() -> intake.intakeIn(), () -> intake.stop(), intake))
        .bind(
            "runIndexer",
            new RunIndexer(
                indexer,
                Constants.IndexerConstants.kFeederTargetRPM,
                Constants.IndexerConstants.kSpindexerTargetRPM))
        .bind(
            "reverseIndexer",
            new RunIndexer(
                indexer,
                Constants.IndexerConstants.kFeederReverseRPM,
                Constants.IndexerConstants.kSpindexerReverseRPM))
        .bind(
            "spinUpShooter", shooter.holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM))
        .bind(
            "shoot",
            shooter
                .holdRPMCommand(Constants.ShooterConstants.SHOOTER_SPINUP_RPM)
                .alongWith(Commands.waitUntil(shooter::isReady)
                    .andThen(new RunIndexer(
                            indexer,
                            Constants.IndexerConstants.kFeederTargetRPM,
                            Constants.IndexerConstants.kSpindexerTargetRPM)
                        .withTimeout(1.0))))
        .bind(
            "stopAll",
            Commands.runOnce(
                () -> {
                  intake.stop();
                  indexer.stop();
                  shooter.stop();
                },
                intake,
                indexer,
                shooter));

    System.out.println("[RobotContainer] Marker bindings registered for Choreo");
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
        () -> -m_driverController.getRightX(), // Rotation
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
    // ==================== INDEXER CONTROLS ====================

    // Right Trigger, Run Indexer Forward (Intake/Feed)
    // Uses the new RPM constants for Feeder and Spindexer
    m_driverController
        .rightTrigger(0.5)
        .whileTrue(new RunIndexer(
                indexer,
                Constants.IndexerConstants.kFeederTargetRPM,
                Constants.IndexerConstants.kSpindexerTargetRPM)
            .withInterruptBehavior(Command.InterruptionBehavior.kCancelIncoming));

    // B Button - Run Indexer Reverse (Unjam)
    m_driverController
        .b()
        .whileTrue(new RunIndexer(
                indexer,
                Constants.IndexerConstants.kFeederReverseRPM,
                Constants.IndexerConstants.kSpindexerReverseRPM)
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
    m_driverController
        .x()
        .toggleOnTrue(new StartEndCommand(() -> intake.intakeIn(), () -> intake.stop(), intake));
    // TODO: Add climb controls

    // ==================== PATHFINDING CONTROLS ====================
    // X button - Pathfind to AprilTag 10 (Red Alliance Hub Face)
    // Uses AD* algorithm to find safe path around obstacles
    // m_driverController.x().whileTrue(drivetrain.pathfindToAprilTag10());

    // NOTE: Pathfind to AprilTag 18 moved to operator POV-Up to avoid
    // conflicting with driver B button (reverse indexer).
    m_operatorController.povUp().whileTrue(drivetrain.pathfindToAprilTag(18));

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
   * Returns the autonomous command to run during autonomous period. Uses the auto chooser from
   * SmartDashboard if a PathPlanner auto is selected, otherwise falls back to AD* pathfinding.
   */
  public Command getAutonomousCommand() {
    String selectedChoreoTrajectory = choreoChooser.getSelected();
    if (selectedChoreoTrajectory != null && !selectedChoreoTrajectory.isBlank()) {
      AutoRoutine routine = choreoAutoFactory.newRoutine("SelectedChoreo");
      AutoTrajectory trajectory = routine.trajectory(selectedChoreoTrajectory);
      routine.active().onTrue(Commands.sequence(trajectory.resetOdometry(), trajectory.cmd()));
      return routine.cmd().withName("Choreo: " + selectedChoreoTrajectory);
    }

    Command selectedAuto = autoChooser.getSelected();

    // If a PathPlanner auto was selected (not the default "none"), use it
    if (selectedAuto != null) {
      return selectedAuto;
    }

    // Fallback: use AD* pathfinding to AprilTag 10 (Red Alliance Hub)
    Pathfinding.setAutoObstacles();
    return drivetrain.pathfindToAprilTag10().withName("Fallback: Pathfind to Tag 10");
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
