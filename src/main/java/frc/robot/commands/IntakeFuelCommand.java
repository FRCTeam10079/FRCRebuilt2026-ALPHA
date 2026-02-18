package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntake.IntakeWheelsSubsystem;
import frc.robot.subsystems.PivotIntake.PivotSubsystem;

/**
 * A command that spins up intakeWheels and begins pivoting down the intake The command ENDS once
 * the pivot has finished moving
 */
public class IntakeFuelCommand extends Command {

  private final PivotSubsystem pivot;
  private final IntakeWheelsSubsystem wheels;

  public IntakeFuelCommand(PivotSubsystem pivot, IntakeWheelsSubsystem wheels) {
    this.pivot = pivot;
    this.wheels = wheels;

    addRequirements(pivot, wheels);
  }

  @Override
  public void initialize() {
    // initializes by deploying pivot and beginning the intake sequence
    pivot.deployPivot();
    wheels.intakeIn();
  }

  /** Only finishes once deployed completely */
  @Override
  public boolean isFinished() {
    return pivot.reachedSetpoint();
  }
}
