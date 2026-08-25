package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.DumpConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dump extends SubsystemBase {
  private TalonFX m_kicker;
  private TalonFX m_floor;

  private TalonFXConfiguration m_kickerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_floorConfig = new TalonFXConfiguration();

  private CurrentLimitsConfigs m_kickerCurrentConfig;
  private CurrentLimitsConfigs m_floorCurrentConfig;

  private double m_kickerTargetRPM;
  private double m_floorTargetVoltage;

  private VelocityVoltage m_kickerVelControl;
  private VoltageOut m_floorVoltageOut;

  public Dump() {
    m_kickerCurrentConfig = new CurrentLimitsConfigs();

    m_kicker = new TalonFX(KICKER_ID, RIO_BUS);
    m_floor = new TalonFX(FLOOR_ID, RIO_BUS);

    m_kickerCurrentConfig.withSupplyCurrentLimit(KICKER_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(KICKER_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    m_floorCurrentConfig.withSupplyCurrentLimit(FLOOR_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(FLOOR_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);

    m_kickerConfig.Slot0.kP = KICKER_KP;
    m_kickerConfig.Slot0.kI = KICKER_KI;
    m_kickerConfig.Slot0.kD = KICKER_KD;
    m_kickerConfig.Slot0.kV = KICKER_KV;
    m_kickerConfig.CurrentLimits = m_kickerCurrentConfig;

    m_floorConfig.CurrentLimits = m_floorCurrentConfig;

    // TODO: change brake/coast values depending on what we need
    m_kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_kicker.getConfigurator().apply(m_kickerConfig);

    m_floor.getConfigurator().apply(m_floorConfig);

    m_kickerVelControl = new VelocityVoltage(0);
    m_floorVoltageOut = new VoltageOut(0);
  }

  public void setKickerPID(double new_P, double new_I, double new_D) {
    Slot0Configs slot0Config2 = new Slot0Configs();
    slot0Config2.kP = new_P;
    slot0Config2.kI = new_I;
    slot0Config2.kD = new_D;
    slot0Config2.kV = KICKER_KV;

    m_kicker.getConfigurator().apply(slot0Config2);
  }

  public double getKickerRPM() {
    return m_kicker.getVelocity().getValueAsDouble() * 60;
  }

  public double getKickerTargetRPM() {
    return m_kickerTargetRPM;
  }

  public double getFloorTargetVoltage() {
    return m_floorTargetVoltage;
  }

  public Command stopCommand() {
    return run(() -> {
      m_kickerTargetRPM = 0;
    });
  }

  public Command setKickerRPM(double rpm) {
    return startEnd(() -> {
      m_kickerTargetRPM = rpm;
    }, () -> {
      m_kickerTargetRPM = 0;
    });
  }

  public Command runFloor(double voltage) {
    return startEnd(() -> {
      m_floorTargetVoltage = voltage;
    }, () -> {
      m_floorTargetVoltage = 0;
    });
  }

  @Override
  public void periodic() {
    m_kicker.setControl(m_kickerVelControl.withVelocity(m_kickerTargetRPM / 60));

    m_floor.setControl(m_floorVoltageOut.withOutput(m_floorTargetVoltage));
  }
}