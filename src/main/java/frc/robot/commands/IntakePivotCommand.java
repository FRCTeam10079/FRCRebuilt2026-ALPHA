package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntake.PivotSubsystem;

public class IntakePivotCommand extends Command {

  private final PivotSubsystem pivot;

  public IntakePivotCommand(PivotSubsystem pivot) {
    this.pivot = pivot;

    addRequirements(pivot);
  }

  /** Deploys the pivot upon initalization */
  @Override
  public void initialize() {
    pivot.deployPivot();
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  /** Only finishes once deployed completely */
  @Override
  public boolean isFinished() {
    return pivot.reachedSetpoint();
  }
}
