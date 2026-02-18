package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotIntake.IntakeWheelsSubsystem;
import frc.robot.subsystems.PivotIntake.PivotSubsystem;

/**
 * A command that stops intake wheels and stows the pivot to protect it The command only ENDS once
 * the pivot has stopped moving
 */
public class EndIntakingCommand extends Command {

  private final PivotSubsystem pivot;
  private final IntakeWheelsSubsystem wheels;

  public EndIntakingCommand(PivotSubsystem pivot, IntakeWheelsSubsystem wheels) {
    this.pivot = pivot;
    this.wheels = wheels;

    addRequirements(pivot, wheels);
  }

  @Override
  public void initialize() {
    // initializes by stowing pivot and stopping intake wheels
    pivot.stowPivot();
    wheels.stopIntake();
  }

  /** Only finishes once stowed completely */
  @Override
  public boolean isFinished() {
    return pivot.reachedSetpoint();
  }
}
