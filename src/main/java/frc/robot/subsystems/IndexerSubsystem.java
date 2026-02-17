package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IndexerConstants;

public class IndexerSubsystem extends SubsystemBase {

  // Two independent motors
  private final TalonFX m_feederMotor;
  private final TalonFX m_spindexerMotor;

  // Two independent control requests (allows us to send different speeds)
  private final VelocityVoltage m_feederRequest = new VelocityVoltage(0).withSlot(0);
  private final VelocityVoltage m_spindexerRequest = new VelocityVoltage(0).withSlot(0);

  public IndexerSubsystem() {
    // Initialize Hardware
    // Now uses rio
    m_feederMotor = new TalonFX(IndexerConstants.kFeederMotorID, "rio");
    m_spindexerMotor = new TalonFX(IndexerConstants.kSpindexerMotorID, "rio");

    // ==================== FEEDER CONFIGURATION ====================
    TalonFXConfiguration feederConfig = new TalonFXConfiguration();

    // Safety
    feederConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    feederConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Feeder PID (Uses Feeder Constants)
    feederConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kFeederKP)
        .withKI(IndexerConstants.kFeederKI)
        .withKD(IndexerConstants.kFeederKD)
        .withKS(IndexerConstants.kFeederKS)
        .withKV(IndexerConstants.kFeederKV)
        .withKA(IndexerConstants.kFeederKA)
        .withKG(IndexerConstants.kFeederKG);

    m_feederMotor.getConfigurator().apply(feederConfig);

    // ==================== SPINDEXER CONFIGURATION ====================
    TalonFXConfiguration spindexerConfig = new TalonFXConfiguration();

    // Safety (Same limits, but distinct config object)
    spindexerConfig.CurrentLimits.StatorCurrentLimit = IndexerConstants.kCurrentLimit;
    spindexerConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    spindexerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // Spindexer PID (Uses Spindexer Constants)
    spindexerConfig.Slot0 = new Slot0Configs()
        .withKP(IndexerConstants.kSpindexerKP)
        .withKI(IndexerConstants.kSpindexerKI)
        .withKD(IndexerConstants.kSpindexerKD)
        .withKS(IndexerConstants.kSpindexerKS)
        .withKV(IndexerConstants.kSpindexerKV)
        .withKA(IndexerConstants.kSpindexerKA)
        .withKG(IndexerConstants.kSpindexerKG);

    m_spindexerMotor.getConfigurator().apply(spindexerConfig);
  }

  /**
   * Sets the speeds of both indexer motors independently.
   *
   * @param feederRPM    Target RPM for the fast feeder wheel
   * @param spindexerRPM Target RPM for the floor/spindexer wheel
   */
  public void setSpeeds(double feederRPM, double spindexerRPM) {
    // 1. Convert RPM to RPS
    double feederRPS = feederRPM / 60.0;
    double spindexerRPS = spindexerRPM / 60.0;

    // 2. Send commands to motors
    m_feederMotor.setControl(m_feederRequest.withVelocity(feederRPS));
    m_spindexerMotor.setControl(m_spindexerRequest.withVelocity(spindexerRPS));
  }

  public void stop() {
    m_feederMotor.stopMotor();
    m_spindexerMotor.stopMotor();
  }
}