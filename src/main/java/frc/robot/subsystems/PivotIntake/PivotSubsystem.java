package frc.robot.subsystems.PivotIntake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.lib.NetworkedLib.NetworkedTalonFX;

public class PivotSubsystem extends SubsystemBase {

  private final NetworkedTalonFX pivotMotor = new NetworkedTalonFX(IntakeConstants.Pivot.MOTOR_ID, "rio");
  private double pivotSetpoint = IntakeConstants.Pivot.STOWED_POSITION;
  private PositionVoltage m_positionVoltage = new PositionVoltage(pivotSetpoint);
  private NeutralOut m_neutralVoltage = new NeutralOut();

  public PivotSubsystem() {
    configurePivotMotor();
  }

  // Configure Pivot Motor
  private void configurePivotMotor() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    // PID constants
    config.Slot0 = config.Slot0.withGravityType(GravityTypeValue.Arm_Cosine)
        .withKA(IntakeConstants.Pivot.KA)
        .withKV(IntakeConstants.Pivot.KV)
        .withKD(IntakeConstants.Pivot.KD)
        .withKG(IntakeConstants.Pivot.KG)
        .withKS(IntakeConstants.Pivot.KS)
        .withKI(IntakeConstants.Pivot.KI)
        .withKP(IntakeConstants.Pivot.KP);

    // config.Feedback.SensorToMechanismRatio =
    // IntakeConstants.Pivot.SENSOR_MECHANISM_RATIO;

    config.CurrentLimits.SupplyCurrentLimit = IntakeConstants.Pivot.SUPPLY_CURRENT_LIMIT;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = IntakeConstants.Pivot.INTAKE_POSITION;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;

    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = IntakeConstants.Pivot.STOWED_POSITION;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    pivotMotor.applyConfiguration(config);
  }

  public void setPivotPosition(double position) {
    pivotSetpoint = position;
    m_positionVoltage = m_positionVoltage.withPosition(pivotSetpoint);
  }

  public void deployPivot() {
    setPivotPosition(IntakeConstants.Pivot.INTAKE_POSITION);
  }

  public void stowPivot() {
    setPivotPosition(IntakeConstants.Pivot.STOWED_POSITION);
  }

  public boolean reachedSetpoint() {
    return Math.abs(Math
        .abs(getPivotPosition()) - Math.abs(pivotSetpoint)) < IntakeConstants.Pivot.DEPLOY_TOLERANCE;
  }

  public double getPivotPosition() {
    return pivotMotor.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    pivotMotor.periodic();
    if (reachedSetpoint()) {
      pivotMotor.setControl(m_neutralVoltage);
    } else {
      pivotMotor.setControl(m_positionVoltage);
    }

    SmartDashboard.putNumber("Pivot/setpoint", pivotSetpoint);
    SmartDashboard.putNumber("Pivot/position", getPivotPosition());
    SmartDashboard.putBoolean("Pivot/reachedSetpoint?", reachedSetpoint());
  }
}
