package frc.robot.subsystems.PivotIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.generated.TunerConstants;

public class IntakeWheelsSubsystem extends SubsystemBase {

  private final TalonFX intakeMotor =
      new TalonFX(IntakeConstants.Wheels.MOTOR_ID, TunerConstants.kCANBus);
  private final VelocityVoltage m_velocityRequest =
      new VelocityVoltage(0).withSlot(0).withEnableFOC(true);

  public IntakeWheelsSubsystem() {
    configureIntakeMotor();
  }

  private void configureIntakeMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Wheels.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimit = IntakeConstants.Wheels.STATOR_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = true;

    config.Slot0 = config.Slot0.withKA(IntakeConstants.Wheels.KA)
        .withKV(IntakeConstants.Wheels.KV)
        .withKD(IntakeConstants.Wheels.KD)
        .withKS(IntakeConstants.Wheels.KS)
        .withKI(IntakeConstants.Wheels.KI)
        .withKP(IntakeConstants.Wheels.KP);

    intakeMotor.getConfigurator().apply(config);
  }

  public void intakeIn() {
    // conversion of RPM to RPS
    intakeMotor.setControl(
        m_velocityRequest.withVelocity(IntakeConstants.Wheels.INTAKE_RPM / 60.0));
  }

  public void intakeOut() {
    // conversion of RPM to RPS
    intakeMotor.setControl(
        m_velocityRequest.withVelocity(IntakeConstants.Wheels.INTAKEOUT_RPM / 60.0));
  }

  public void stop() {
    intakeMotor.setControl(m_velocityRequest.withVelocity(0));
  }

  // Intake Wheel Spin Command
  public Command runIntakeOnce() {
    return Commands.runOnce(() -> intakeIn(), this);
  }

  // Intake Wheel Reverse Command
  public Command runIntakeOutOnce() {
    return Commands.runOnce(() -> intakeOut(), this);
  }
}
