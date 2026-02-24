package frc.robot;

import edu.wpi.first.hal.PowerJNI;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.ArrayList;
import java.util.List;

/**
 * MASTER ROBOT STATE MACHINE - FRC 2026 REBUILT This is the HIGHEST level state machine that
 * controls the ENTIRE robot. Every aspect of robot operation flows through this state machine.
 *
 * <p>REBUILT Game Overview: - Score Fuel (balls) into Hubs - Hub Shift mechanic: Hubs alternate
 * between ACTIVE and INACTIVE - Climb the Tower with multiple levels (L1=15pts, L2, L3) - Height
 * restriction: Must be < 30 inches during climbing phases
 */
public class RobotStateMachine extends SubsystemBase {

  // Singleton instance
  private static RobotStateMachine instance;

  // Current states
  private MatchState matchState = MatchState.DISABLED;
  private DrivetrainMode driveMode = DrivetrainMode.FIELD_CENTRIC;
  private GameState gameState = GameState.IDLE;
  private AllianceColor alliance = AllianceColor.UNKNOWN;
  private FuelState fuelState = FuelState.EMPTY; // Fuel (ball) tracking
  private HubShiftState hubShiftState = HubShiftState.UNKNOWN; // Hub shift tracking
  private ClimbState climbState = ClimbState.NOT_CLIMBING; // Climb state tracking

  // Alignment tracking (for auto-aiming to hub)
  private boolean isAlignedToTarget = false;
  private boolean isShooterAtRPM = false; // Shooter ready to fire

  // State tracking
  private double stateStartTime = 0;
  private double matchStartTime = 0;
  private MatchState previousMatchState = MatchState.DISABLED;
  private GameState previousGameState = GameState.IDLE;

  // State history (circular buffer for last 20 states)
  private static final int STATE_HISTORY_SIZE = 20;
  private final List<StateHistoryEntry> stateHistory = new ArrayList<>();

  // Cycle counting for performance tracking
  private int fuelScoredAuto = 0;
  private int fuelScoredTeleop = 0;
  private int intakeCyclesCompleted = 0;
  private int scoringCyclesCompleted = 0;
  private double lastCycleTime = 0;
  private double fastestCycleTime = Double.MAX_VALUE;

  // Fuel capacity tracking
  private int fuelCount = 0;

  // Driver feedback
  private CommandXboxController driverController;
  private CommandXboxController operatorController;
  private double rumbleEndTime = 0;

  /** FUEL STATE - Tracks fuel (ball) inventory */
  public enum FuelState {
    EMPTY("Empty", "Robot has no fuel"),
    ACQUIRING("Acquiring", "Intake actively collecting fuel"),
    LOADED("Loaded", "Robot has fuel (can hold unlimited!)"),
    FIRING("Firing", "Actively shooting fuel");

    public final String name;
    public final String description;

    FuelState(String name, String description) {
      this.name = name;
      this.description = description;
    }
  }

  /** HUB SHIFT STATE - Tracks which hub is currently active */
  public enum HubShiftState {
    UNKNOWN("Unknown", "Hub state not determined yet"),
    MY_HUB_ACTIVE("My Hub Active", "Our alliance hub is ACTIVE - SCORE NOW!"),
    MY_HUB_INACTIVE("My Hub Inactive", "Our hub is inactive - DEFEND/HOARD"),
    TRANSITION("Transition Shift", "Both hubs active during transition period");

    public final String name;
    public final String description;

    HubShiftState(String name, String description) {
      this.name = name;
      this.description = description;
    }
  }

  /**
   * CLIMB STATE - Tower climbing state machine Climbing has height restrictions (must be < 30
   * inches at certain phases)
   */
  public enum ClimbState {
    NOT_CLIMBING("Not Climbing", "Robot on ground"),
    APPROACHING_TOWER("Approaching Tower", "Driving to climb zone"),
    DEPLOYING_L1("Deploy & Grab L1", "Deploying climber, grabbing Level 1 rung"),
    PULLING_UP_L1("Pull Up Off Floor", "Pulling robot off floor - 15pts"),
    REACHING_L2("Reach for Mid Rung", "Extending to grab Level 2 rung"),
    HEIGHT_CHECK_L2("Height Check L2", "Checking height < 30 inches"),
    GRABBING_L2("Grab L2 / Release L1", "Transferring to Level 2"),
    REACHING_L3("Reach for High Rung", "Extending to grab Level 3 rung"),
    HEIGHT_CHECK_L3("Height Check L3", "Checking height < 30 inches"),
    GRABBING_L3("Grab L3 / Release L2", "Transferring to Level 3"),
    ENGAGED_BRAKE("Brake Engaged", "Climb complete - brake locked"),
    CLIMB_FAILED("Climb Failed", "Climb sequence failed - recovery needed");

    public final String name;
    public final String description;

    ClimbState(String name, String description) {
      this.name = name;
      this.description = description;
    }

    public boolean isClimbing() {
      return this != NOT_CLIMBING && this != APPROACHING_TOWER && this != CLIMB_FAILED;
    }

    public boolean isCompleted() {
      return this == ENGAGED_BRAKE;
    }
  }

  /** State History Entry for debugging */
  private static class StateHistoryEntry {
    public final double timestamp;
    public final String stateType;
    public final String fromState;
    public final String toState;

    public StateHistoryEntry(double timestamp, String stateType, String fromState, String toState) {
      this.timestamp = timestamp;
      this.stateType = stateType;
      this.fromState = fromState;
      this.toState = toState;
    }

    @Override
    public String toString() {
      return String.format("[%.2f] %s: %s -> %s", timestamp, stateType, fromState, toState);
    }
  }

  /** MATCH STATE - Robot's lifecycle phase */
  public enum MatchState {
    DISABLED("Robot Disabled", false, false),
    AUTO_INIT("Autonomous Initializing", true, true),
    AUTO_RUNNING("Autonomous Running", true, true),
    TELEOP_INIT("Teleop Initializing", true, false),
    TELEOP_RUNNING("Teleop Running", true, false),
    TRANSITION_SHIFT("Transition Shift Period", true, false), // Both
    // hubs
    // active
    // (2:30-2:20)
    TEST_INIT("Test Mode Initializing", true, false),
    TEST_RUNNING("Test Mode Running", true, false),
    ENDGAME("Endgame Period", true, false), // < 30 seconds remaining
    ESTOP("Emergency Stop", false, false);

    public final String description;
    public final boolean enabled;
    public final boolean autonomous;

    MatchState(String description, boolean enabled, boolean autonomous) {
      this.description = description;
      this.enabled = enabled;
      this.autonomous = autonomous;
    }
  }

  /** DRIVETRAIN MODE - How the drivetrain is being controlled */
  public enum DrivetrainMode {
    FIELD_CENTRIC("Field-Centric Drive", "Driver controls relative to field"),
    ROBOT_CENTRIC("Robot-Centric Drive", "Driver controls relative to robot"),
    PATH_FOLLOWING("Path Following", "Following PathPlanner trajectory"),
    VISION_TRACKING("Vision Tracking", "Limelight auto-aiming at hub"),
    LOCKED("Wheels Locked", "X-formation brake"),
    SLOW_MODE("Slow Precision Mode", "Reduced speed for alignment"),
    AUTO_ALIGN_HUB("Auto-Aligning to Hub", "Automatic aiming for scoring"),
    DEFENSE_MODE("Defense Mode", "Blocking opponent access"),
    DISABLED("Drivetrain Disabled", "No movement");

    public final String name;
    public final String description;

    DrivetrainMode(String name, String description) {
      this.name = name;
      this.description = description;
    }
  }

  /** GAME STATE - Robot strategy/behavior for REBUILT Implements the Hub Shift Meta strategy */
  public enum GameState {
    // Pre-game
    IDLE("Idle/Stowed", "Robot waiting, all mechanisms stowed"),
    PRE_MATCH_CHECK("Pre-Match Check", "Running systems check"),

    // ===== AUTONOMOUS (0:00-0:20) =====
    AUTO_ROUTINE("Auto: Running Routine", "Executing autonomous routine"),
    AUTO_SCORING_PRELOAD("Auto: Scoring Preload", "Scoring preloaded fuel"),
    AUTO_COLLECTING("Auto: Collecting Fuel", "Collecting fuel in auto"),
    AUTO_NAVIGATING("Auto: Navigating", "Path following to position"),
    AUTO_LEAVING_ZONE("Auto: Leave Zone", "Leaving starting zone"),
    AUTO_CLIMBING_L1("Auto: Climbing L1", "Auto climb to Level 1 - 15pts"),

    // ===== TRANSITION PERIOD (2:30-2:20) =====
    TRANSITION("Transition Period", "Both hubs ACTIVE - score now!"),

    // ===== HUB ACTIVE STATE (Offense) =====
    SHIFT_ACTIVE("Shift: Hub Active", "Our hub is ACTIVE - offensive mode"),
    AIMING_AT_HUB("Aiming at Hub", "Aligning drivetrain to hub"),
    SHOOTER_SPINUP("Shooter Spinup", "Spinning up shooter to target RPM"),
    FIRING_FUEL("Feeder: FIRE", "Feeding fuel into shooter"),

    // ===== HUB INACTIVE STATE (Defense/Hoard) =====
    SHIFT_INACTIVE("Shift: Hub Inactive", "Our hub is INACTIVE - defense/hoard mode"),
    INTAKE_ACQUIRING("Intake: Acquiring", "Actively collecting fuel during inactive"),
    DEFENSE_BLOCKING("Defense: Blocking", "Blocking opponent from scoring"),
    HOARDING("Hoarding Fuel", "Collecting and storing fuel for next shift"),
    HIDING("Hiding", "Avoiding opponents while holding fuel"),

    // ===== COLLECTING STATES =====
    SEEKING_FUEL("Seeking Fuel", "Driving to fuel location"),
    COLLECTING_GROUND("Collecting from Ground", "Ground intake active"),
    FUEL_SECURED("Fuel Secured", "Fuel in hopper, ready to score"),

    // ===== SCORING STATES =====
    APPROACHING_HUB("Approaching Hub", "Driving to scoring position"),
    ALIGNING_TO_HUB("Aligning to Hub", "Fine-tuning aim for shot"),
    SCORING_HUB("Scoring Hub", "Shooting fuel into hub"),

    // ===== ENDGAME - CLIMB SEQUENCE =====
    APPROACHING_CLIMB("Approaching Climb", "Driving to tower/climb zone"),
    PREPARING_CLIMB("Preparing to Climb", "Setting up mechanisms for climb"),
    CLIMBING_L1("Climbing Level 1", "Climbing to Level 1 - 15pts"),
    CLIMBING_L2("Climbing Level 2", "Climbing to Level 2"),
    CLIMBING_L3("Climbing Level 3", "Climbing to Level 3 (High)"),
    CLIMBED_L1("Climbed Level 1", "Successfully on Level 1 rung"),
    CLIMBED_L2("Climbed Level 2", "Successfully on Level 2 rung"),
    CLIMBED_L3("Climbed Level 3", "Successfully on Level 3 (High) rung"),
    SCORING_FINAL("Scoring Final Fuel", "Scoring remaining fuel before climb"),

    // ===== SPECIAL STATES =====
    REPOSITIONING("Repositioning", "Moving to strategic position"),

    // Error/Recovery
    RECOVERING("Recovering from Error", "Attempting error recovery"),
    MANUAL_OVERRIDE("Manual Override", "Driver direct control"),
    EMERGENCY_STOP("Emergency Stop", "All systems halted");

    public final String name;
    public final String description;

    GameState(String name, String description) {
      this.name = name;
      this.description = description;
    }

    // Helper methods for state categories
    public boolean isCollecting() {
      return this == COLLECTING_GROUND
          || this == AUTO_COLLECTING
          || this == INTAKE_ACQUIRING
          || this == SEEKING_FUEL
          || this == HOARDING;
    }

    public boolean isScoring() {
      return this == SCORING_HUB
          || this == FIRING_FUEL
          || this == AUTO_SCORING_PRELOAD
          || this == SCORING_FINAL;
    }

    public boolean isNavigating() {
      return this == APPROACHING_HUB
          || this == APPROACHING_CLIMB
          || this == AUTO_NAVIGATING
          || this == SEEKING_FUEL
          || this == REPOSITIONING;
    }

    public boolean isEndgame() {
      return this == APPROACHING_CLIMB
          || this == PREPARING_CLIMB
          || this == CLIMBING_L1
          || this == CLIMBING_L2
          || this == CLIMBING_L3
          || this == CLIMBED_L1
          || this == CLIMBED_L2
          || this == CLIMBED_L3
          || this == SCORING_FINAL;
    }

    public boolean isClimbed() {
      return this == CLIMBED_L1 || this == CLIMBED_L2 || this == CLIMBED_L3;
    }

    public boolean isClimbing() {
      return this == CLIMBING_L1 || this == CLIMBING_L2 || this == CLIMBING_L3;
    }

    public boolean isHubActive() {
      return this == SHIFT_ACTIVE
          || this == AIMING_AT_HUB
          || this == SHOOTER_SPINUP
          || this == FIRING_FUEL;
    }

    public boolean isHubInactive() {
      return this == SHIFT_INACTIVE
          || this == INTAKE_ACQUIRING
          || this == DEFENSE_BLOCKING
          || this == HOARDING
          || this == HIDING;
    }

    public boolean isDefending() {
      return this == DEFENSE_BLOCKING || this == HIDING;
    }

    public boolean isAuto() {
      return this.name().startsWith("AUTO_");
    }
  }

  /** ALLIANCE COLOR */
  public enum AllianceColor {
    RED("Red Alliance"),
    BLUE("Blue Alliance"),
    UNKNOWN("Unknown Alliance");

    public final String description;

    AllianceColor(String description) {
      this.description = description;
    }
  }

  /** Singleton pattern */
  public static RobotStateMachine getInstance() {
    if (instance == null) {
      instance = new RobotStateMachine();
    }
    return instance;
  }

  private RobotStateMachine() {
    updateAlliance();
  }

  /** Update alliance color from DriverStation */
  public void updateAlliance() {
    if (DriverStation.getAlliance().isPresent()) {
      alliance = DriverStation.getAlliance().get() == Alliance.Red
          ? AllianceColor.RED
          : AllianceColor.BLUE;
    } else {
      alliance = AllianceColor.UNKNOWN;
    }
  }

  /** Transition to new match state */
  public void setMatchState(MatchState newState) {
    if (matchState != newState) {
      previousMatchState = matchState;
      matchState = newState;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      onMatchStateChange();
      logStateChange("Match", previousMatchState.name(), newState.name());
    }
  }

  /** Transition to new game state */
  public void setGameState(GameState newState) {
    if (gameState != newState) {
      previousGameState = gameState;
      gameState = newState;
      stateStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
      onGameStateChange();
      logStateChange("Game", previousGameState.name(), newState.name());
    }
  }

  /** Transition to new drivetrain mode */
  public void setDrivetrainMode(DrivetrainMode newMode) {
    if (driveMode != newMode) {
      DrivetrainMode previousMode = driveMode;
      driveMode = newMode;
      logStateChange("Drivetrain", previousMode.name(), newMode.name());
    }
  }

  /** Set hub shift state based on match timing or field data */
  public void setHubShiftState(HubShiftState newState) {
    if (hubShiftState != newState) {
      HubShiftState previousState = hubShiftState;
      hubShiftState = newState;
      logStateChange("HubShift", previousState.name(), newState.name());
      onHubShiftChange();
    }
  }

  /** Handle match state transitions */
  private void onMatchStateChange() {
    switch (matchState) {
      case DISABLED:
        setGameState(GameState.IDLE);
        setDrivetrainMode(DrivetrainMode.DISABLED);
        isAlignedToTarget = false;
        isShooterAtRPM = false;
        climbState = ClimbState.NOT_CLIMBING;
        break;

      case AUTO_INIT:
        setGameState(GameState.AUTO_ROUTINE);
        setDrivetrainMode(DrivetrainMode.PATH_FOLLOWING);
        isAlignedToTarget = false;
        updateAlliance();
        resetCycleCounters();
        break;

      case TELEOP_INIT:
        setGameState(GameState.IDLE);
        setDrivetrainMode(DrivetrainMode.FIELD_CENTRIC);
        isAlignedToTarget = false;
        hubShiftState = HubShiftState.UNKNOWN; // Will be updated based on timing
        break;

      case TRANSITION_SHIFT:
        // Transition period (2:30-2:20): Both hubs ACTIVE!
        setHubShiftState(HubShiftState.TRANSITION);
        setGameState(GameState.TRANSITION);
        System.out.println("=== TRANSITION SHIFT - BOTH HUBS ACTIVE! ===");
        rumbleDriver(1.0, 0.5); // Alert driver
        break;

      case ENDGAME:
        System.out.println("=== ENDGAME PERIOD STARTED! ===");
        rumbleDriver(0.8, 0.75); // Alert driver: Consider climbing!
        break;

      default:
        break;
    }
  }

  /** Handle game state transitions Drivetrain mode is decoupled from game state for flexibility */
  private void onGameStateChange() {
    switch (gameState) {
      case EMERGENCY_STOP:
        setDrivetrainMode(DrivetrainMode.DISABLED);
        break;

      // Climbing states lock wheels for safety
      case CLIMBING_L1:
      case CLIMBING_L2:
      case CLIMBING_L3:
      case CLIMBED_L1:
      case CLIMBED_L2:
      case CLIMBED_L3:
        setDrivetrainMode(DrivetrainMode.LOCKED);
        break;

      // Defense mode
      case DEFENSE_BLOCKING:
      case HIDING:
        setDrivetrainMode(DrivetrainMode.DEFENSE_MODE);
        break;

      // Aiming states use vision tracking
      case AIMING_AT_HUB:
      case ALIGNING_TO_HUB:
        setDrivetrainMode(DrivetrainMode.VISION_TRACKING);
        break;

      default:
        // Other states do not force drivetrain mode
        break;
    }
  }

  /** Handle hub shift state changes - THE META! */
  private void onHubShiftChange() {
    switch (hubShiftState) {
      case MY_HUB_ACTIVE:
        // Time to score! Switch to offensive mode
        if (!gameState.isEndgame() && !gameState.isClimbing()) {
          setGameState(GameState.SHIFT_ACTIVE);
          rumbleDriver(0.5, 0.3); // Quick buzz: Hub is active!
        }
        break;

      case MY_HUB_INACTIVE:
        // Time to defend/hoard!
        if (!gameState.isEndgame() && !gameState.isClimbing()) {
          setGameState(GameState.SHIFT_INACTIVE);
          rumbleDriver(0.3, 0.2); // Gentle buzz: Switch to defense
        }
        break;

      case TRANSITION:
        // Both hubs active - SCORE NOW!
        rumbleDriver(1.0, 0.5);
        break;

      default:
        break;
    }
  }

  /** Get time in current state (seconds) */
  public double getTimeInState() {
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - stateStartTime;
  }

  /** Check if match time indicates endgame (< 30 seconds) */
  public boolean isEndgamePeriod() {
    return DriverStation.isTeleopEnabled()
        && DriverStation.getMatchTime() > 0
        && DriverStation.getMatchTime() <= 30;
  }

  /**
   * Check if in transition shift period (2:30-2:20 in teleop, i.e. 10 seconds after teleop starts)
   */
  public boolean isTransitionPeriod() {
    if (!DriverStation.isTeleopEnabled()) return false;
    double matchTime = DriverStation.getMatchTime();
    // Transition is typically at the start of teleop (2:30-2:20, so match time
    // 150-140)
    return matchTime >= 140 && matchTime <= 150;
  }

  /** Automatic period detection */
  private void checkPeriodTransitions() {
    if (matchState == MatchState.TELEOP_RUNNING) {
      // Check for endgame
      if (isEndgamePeriod() && matchState != MatchState.ENDGAME) {
        setMatchState(MatchState.ENDGAME);
      }
      // Check for transition shift period
      else if (isTransitionPeriod() && matchState != MatchState.TRANSITION_SHIFT) {
        setMatchState(MatchState.TRANSITION_SHIFT);
      }
    }
  }

  /** State validation - prevent invalid transitions */
  public boolean canTransitionTo(GameState newState) {
    // Disabled can only be IDLE
    if (matchState == MatchState.DISABLED && newState != GameState.IDLE) {
      return false;
    }

    // Auto states only in auto
    if (newState.isAuto() && !matchState.autonomous) {
      return false;
    }

    // Can always emergency stop
    if (newState == GameState.EMERGENCY_STOP) {
      return true;
    }

    return true;
  }

  /** Safe state transition with validation */
  public void requestGameState(GameState newState) {
    if (canTransitionTo(newState)) {
      setGameState(newState);
    } else {
      System.out.println("INVALID STATE TRANSITION: " + gameState + " -> " + newState);
    }
  }

  /** Log state changes */
  private void logStateChange(String type, String from, String to) {
    String message = String.format("%s State: %s -> %s", type, from, to);
    System.out.println(message);

    // SmartDashboard
    SmartDashboard.putString("Last State Change", message);

    // Add to history
    addToStateHistory(type, from, to);
  }

  @Override
  public void periodic() {
    // Automatic period detection
    checkPeriodTransitions();

    // Update driver feedback
    updateRumble();

    // Update telemetry
    updateTelemetry();
  }

  /** Telemetry logging */
  private void updateTelemetry() {
    // Match State
    SmartDashboard.putString("Match State", matchState.name());
    SmartDashboard.putString("Match Description", matchState.description);
    SmartDashboard.putNumber("Match Time", getMatchTime());
    SmartDashboard.putBoolean("Robot Enabled", matchState.enabled);
    SmartDashboard.putBoolean("Is Autonomous", matchState.autonomous);
    SmartDashboard.putNumber("Voltage", getBatteryVoltage());

    // Game State
    SmartDashboard.putString("Game State", gameState.name());
    SmartDashboard.putString("Game Description", gameState.description);
    SmartDashboard.putBoolean("Is Collecting", gameState.isCollecting());
    SmartDashboard.putBoolean("Is Scoring", gameState.isScoring());
    SmartDashboard.putBoolean("Is Navigating", gameState.isNavigating());
    SmartDashboard.putBoolean("Is Endgame State", gameState.isEndgame());
    SmartDashboard.putBoolean("Is Climbing", gameState.isClimbing());
    SmartDashboard.putBoolean("Is Hub Active Mode", gameState.isHubActive());
    SmartDashboard.putBoolean("Is Hub Inactive Mode", gameState.isHubInactive());

    // Drivetrain Mode
    SmartDashboard.putString("Drivetrain Mode", driveMode.name);
    SmartDashboard.putString("Drive Description", driveMode.description);

    // Fuel State
    SmartDashboard.putString("Fuel State", fuelState.name());
    SmartDashboard.putNumber("Fuel Count", fuelCount);
    SmartDashboard.putBoolean("Has Fuel", hasFuel());
    SmartDashboard.putBoolean("Ready to Fire", isReadyToFire());

    // Hub Shift State (THE META!)
    SmartDashboard.putString("Hub Shift State", hubShiftState.name());
    SmartDashboard.putBoolean("My Hub Active", hubShiftState == HubShiftState.MY_HUB_ACTIVE);
    SmartDashboard.putBoolean("Is Transition Period", hubShiftState == HubShiftState.TRANSITION);

    // Climb State
    SmartDashboard.putString("Climb State", climbState.name());
    SmartDashboard.putBoolean("Is Climbing (Climb)", climbState.isClimbing());
    SmartDashboard.putBoolean("Climb Complete", climbState.isCompleted());

    // Shooter Status
    SmartDashboard.putBoolean("Aligned to Target", isAlignedToTarget);
    SmartDashboard.putBoolean("Shooter at RPM", isShooterAtRPM);

    // Alliance & Timing
    SmartDashboard.putString("Alliance", alliance.description);
    SmartDashboard.putNumber("Time in State", getTimeInState());
    SmartDashboard.putBoolean("Is Endgame Period", isEndgamePeriod());
    SmartDashboard.putBoolean("Is Transition Period", isTransitionPeriod());

    // Cycle Statistics
    SmartDashboard.putNumber("Fuel Scored (Auto)", fuelScoredAuto);
    SmartDashboard.putNumber("Fuel Scored (Teleop)", fuelScoredTeleop);
    SmartDashboard.putNumber("Total Fuel Scored", getTotalFuelScored());
    SmartDashboard.putNumber("Intake Cycles", intakeCyclesCompleted);
    SmartDashboard.putNumber("Scoring Cycles", scoringCyclesCompleted);
    SmartDashboard.putNumber("Fastest Cycle Time", getFastestCycleTime());
  }

  // ===== GETTERS =====
  public MatchState getMatchState() {
    return matchState;
  }

  public double getMatchTime(){
    return DriverStation.getMatchTime();
  }

  public GameState getGameState() {
    return gameState;
  }

  public DrivetrainMode getDrivetrainMode() {
    return driveMode;
  }

  public AllianceColor getAlliance() {
    return alliance;
  }

  public FuelState getFuelState() {
    return fuelState;
  }

  public HubShiftState getHubShiftState() {
    return hubShiftState;
  }

  public ClimbState getClimbState() {
    return climbState;
  }

  public int getFuelCount() {
    return fuelCount;
  }

  public double getBatteryVoltage(){
    return PowerJNI.getVinVoltage();
  }

  public boolean isEnabled() {
    return matchState.enabled;
  }

  public boolean isAutonomous() {
    return matchState.autonomous;
  }

  public boolean isTeleop() {
    return matchState == MatchState.TELEOP_RUNNING
        || matchState == MatchState.TRANSITION_SHIFT
        || matchState == MatchState.ENDGAME;
  }

  // State queries
  public boolean isInState(MatchState state) {
    return matchState == state;
  }

  public boolean isInState(GameState state) {
    return gameState == state;
  }

  public boolean isInState(DrivetrainMode mode) {
    return driveMode == mode;
  }

  public boolean isInState(FuelState state) {
    return fuelState == state;
  }

  public boolean isInState(HubShiftState state) {
    return hubShiftState == state;
  }

  public boolean isInState(ClimbState state) {
    return climbState == state;
  }

  // ===== FUEL MANAGEMENT =====
  // NOTE: No max capacity!
  public boolean hasFuel() {
    return fuelCount > 0;
  }

  public boolean isEmpty() {
    return fuelCount == 0;
  }

  /** Check if ready to fire (aligned + RPM + has fuel) */
  public boolean isReadyToFire() {
    return hasFuel()
        && isAlignedToTarget
        && isShooterAtRPM
        && (hubShiftState == HubShiftState.MY_HUB_ACTIVE
            || hubShiftState == HubShiftState.TRANSITION);
  }

  /** Alignment tracking methods */
  public void setAlignedToTarget(boolean aligned) {
    if (isAlignedToTarget != aligned) {
      isAlignedToTarget = aligned;
      SmartDashboard.putBoolean("Aligned to Target", aligned);
      if (aligned) {
        rumbleDriver(0.4, 0.15); // Quick rumble - alignment successful
      }
    }
  }

  public boolean isAlignedToTarget() {
    return isAlignedToTarget;
  }

  /** Shooter RPM tracking */
  public void setShooterAtRPM(boolean atRPM) {
    if (isShooterAtRPM != atRPM) {
      isShooterAtRPM = atRPM;
      SmartDashboard.putBoolean("Shooter at RPM", atRPM);
    }
  }

  public boolean isShooterAtRPM() {
    return isShooterAtRPM;
  }

  /** Set fuel state with automatic driver feedback */
  public void setFuelState(FuelState newState) {
    if (fuelState != newState) {
      FuelState previousState = fuelState;
      fuelState = newState;

      addToStateHistory("Fuel", previousState.name(), newState.name());
      SmartDashboard.putString("Fuel State", newState.name());

      // Driver feedback on transitions
      switch (newState) {
        case LOADED:
          rumbleDriver(0.3, 0.2); // Short rumble - fuel collected
          intakeCyclesCompleted++;
          break;
        case EMPTY:
          if (previousState == FuelState.FIRING) {
            rumbleDriver(1.0, 0.5); // Strong rumble - scored!
            if (isAutonomous()) {
              fuelScoredAuto += fuelCount; // All fuel was scored
            } else {
              fuelScoredTeleop += fuelCount;
            }
            scoringCyclesCompleted++;
            updateCycleTime();
          }
          break;
        default:
          break;
      }
    }
  }

  /** Update fuel count */
  public void setFuelCount(int count) {
    fuelCount = Math.max(0, count); // Only enforce minimum of 0, NO maximum!

    // Update fuel state based on count
    if (fuelCount == 0) {
      setFuelState(FuelState.EMPTY);
    } else {
      setFuelState(FuelState.LOADED); // Has fuel (could be 1 or 100!)
    }

    SmartDashboard.putNumber("Fuel Count", fuelCount);
  }

  /** Add fuel (when collected) */
  public void addFuel(int amount) {
    setFuelCount(fuelCount + amount);
  }

  /** Remove fuel (when fired) */
  public void removeFuel(int amount) {
    setFuelCount(fuelCount - amount);
  }

  /** Set climb state */
  public void setClimbState(ClimbState newState) {
    if (climbState != newState) {
      ClimbState previousState = climbState;
      climbState = newState;
      addToStateHistory("Climb", previousState.name(), newState.name());
      SmartDashboard.putString("Climb State", newState.name());

      // Feedback on climb completion
      if (newState == ClimbState.ENGAGED_BRAKE) {
        rumbleDriver(1.0, 1.0); // Strong, long rumble - climb complete!
        System.out.println("=== CLIMB COMPLETE - BRAKE ENGAGED ===");
      } else if (newState == ClimbState.CLIMB_FAILED) {
        rumbleDriver(0.5, 0.5); // Alert: climb failed
        System.out.println("!!! CLIMB FAILED - RECOVERY NEEDED !!!");
      }
    }
  }

  /** Register controllers for driver feedback */
  public void registerControllers(CommandXboxController driver, CommandXboxController operator) {
    this.driverController = driver;
    this.operatorController = operator;
  }

  /** Rumble driver controller for feedback */
  public void rumbleDriver(double intensity, double durationSeconds) {
    if (driverController != null) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, intensity);
      rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
    }
  }

  /** Rumble operator controller for feedback */
  public void rumbleOperator(double intensity, double durationSeconds) {
    if (operatorController != null) {
      operatorController.getHID().setRumble(RumbleType.kBothRumble, intensity);
      rumbleEndTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() + durationSeconds;
    }
  }

  /** Check and stop rumble when duration elapsed */
  private void updateRumble() {
    if (rumbleEndTime > 0 && edu.wpi.first.wpilibj.Timer.getFPGATimestamp() >= rumbleEndTime) {
      if (driverController != null) {
        driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      if (operatorController != null) {
        operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
      }
      rumbleEndTime = 0;
    }
  }

  /** Add entry to state history */
  private void addToStateHistory(String stateType, String from, String to) {
    StateHistoryEntry entry =
        new StateHistoryEntry(edu.wpi.first.wpilibj.Timer.getFPGATimestamp(), stateType, from, to);
    stateHistory.add(entry);
    if (stateHistory.size() > STATE_HISTORY_SIZE) {
      stateHistory.remove(0);
    }
  }

  /** Get state history for debugging */
  public List<StateHistoryEntry> getStateHistory() {
    return new ArrayList<>(stateHistory);
  }

  /** Print state history to console */
  public void printStateHistory() {
    System.out.println("=== STATE HISTORY ===");
    for (StateHistoryEntry entry : stateHistory) {
      System.out.println(entry);
    }
    System.out.println("====================");
  }

  // ===== CYCLE TIME TRACKING =====
  private void updateCycleTime() {
    double now = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
    if (lastCycleTime > 0) {
      double cycleTime = now - lastCycleTime;
      if (cycleTime < fastestCycleTime && cycleTime > 1.0) {
        fastestCycleTime = cycleTime;
      }
    }
    lastCycleTime = now;
  }

  // Cycle statistics getters
  public int getFuelScoredAuto() {
    return fuelScoredAuto;
  }

  public int getFuelScoredTeleop() {
    return fuelScoredTeleop;
  }

  public int getTotalFuelScored() {
    return fuelScoredAuto + fuelScoredTeleop;
  }

  public int getIntakeCyclesCompleted() {
    return intakeCyclesCompleted;
  }

  public int getScoringCyclesCompleted() {
    return scoringCyclesCompleted;
  }

  public double getFastestCycleTime() {
    if (fastestCycleTime == Double.MAX_VALUE) {
      return 0;
    } else {
      return fastestCycleTime;
    }
  }

  /** Reset cycle counters (call at match start) */
  public void resetCycleCounters() {
    fuelScoredAuto = 0;
    fuelScoredTeleop = 0;
    intakeCyclesCompleted = 0;
    scoringCyclesCompleted = 0;
    lastCycleTime = 0;
    fastestCycleTime = Double.MAX_VALUE;
    fuelCount = 0;
    stateHistory.clear();
    matchStartTime = edu.wpi.first.wpilibj.Timer.getFPGATimestamp();
  }

  /** Get match elapsed time */
  public double getMatchElapsedTime() {
    if (matchStartTime == 0) return 0;
    return edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - matchStartTime;
  }
}
