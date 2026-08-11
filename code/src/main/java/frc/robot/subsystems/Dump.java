package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.MotorOutputStatusValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.DumpConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dump extends SubsystemBase {
  private TalonFX m_leftRoller;
  private TalonFX m_leftRollerFollow;
  private TalonFX m_rightRoller;
  private TalonFX m_rightRollerFollow;
  private TalonFX m_kicker;
  private TalonFX m_kickerFollow;
  private TalonFX m_floor;

  private TalonFXConfiguration m_rollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_leftRollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_rightRollerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_kickerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_rollerFollowerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_kickerFollowerConfig = new TalonFXConfiguration();
  private TalonFXConfiguration m_floorConfig = new TalonFXConfiguration();

  private CurrentLimitsConfigs m_rollerCurrentConfig;
  private CurrentLimitsConfigs m_kickerCurrentConfig;
  private CurrentLimitsConfigs m_floorCurrentConfig;

  private double m_rollerTargetRPM;
  private double m_kickerTargetRPM;
  private double m_floorTargetVoltage;

  private VelocityVoltage m_rollerVelControl;
  private VelocityVoltage m_kickerVelControl;
  private VoltageOut m_floorVoltageOut;

  public Dump() {
    m_rollerCurrentConfig = new CurrentLimitsConfigs();
    m_kickerCurrentConfig = new CurrentLimitsConfigs();

    m_leftRoller = new TalonFX(LEFT_ROLLER_ID, RIO_BUS);
    m_leftRollerFollow = new TalonFX(LEFT_ROLLER_FOLLOWER_ID, RIO_BUS);
    m_rightRoller = new TalonFX(RIGHT_ROLLER_ID, RIO_BUS);
    m_rightRollerFollow = new TalonFX(RIGHT_ROLLER_FOLLOWER_ID, RIO_BUS);
    m_kicker = new TalonFX(KICKER_ID, RIO_BUS);
    m_kickerFollow = new TalonFX(KICKER_FOLLOWER_ID, RIO_BUS);
    m_floor = new TalonFX(FLOOR_ID, RIO_BUS);

    m_rollerCurrentConfig.withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    m_kickerCurrentConfig.withSupplyCurrentLimit(KICKER_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(KICKER_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    m_floorCurrentConfig.withSupplyCurrentLimit(FLOOR_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(FLOOR_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);

    m_rollerConfig.Slot0.kP = ROLLER_KP;
    m_rollerConfig.Slot0.kI = ROLLER_KI;
    m_rollerConfig.Slot0.kD = ROLLER_KD;
    m_rollerConfig.Slot0.kV = ROLLER_KV;
    m_rollerConfig.CurrentLimits = m_rollerCurrentConfig;

    m_leftRollerConfig = m_rollerConfig.clone();
    m_leftRollerConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

    m_rightRollerConfig = m_rollerConfig.clone();
    m_rightRollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    m_rollerFollowerConfig.CurrentLimits = m_rollerCurrentConfig;

    m_kickerConfig.Slot0.kP = KICKER_KP;
    m_kickerConfig.Slot0.kI = KICKER_KI;
    m_kickerConfig.Slot0.kD = KICKER_KD;
    m_kickerConfig.Slot0.kV = KICKER_KV;
    m_kickerConfig.CurrentLimits = m_kickerCurrentConfig;

    m_kickerFollowerConfig.CurrentLimits = m_kickerCurrentConfig;

    m_floorConfig.CurrentLimits = m_floorCurrentConfig;

    // TODO: change brake/coast values depending on what we need
    m_rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_rollerFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_kickerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_kickerFollowerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    m_floorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;

    m_leftRoller.getConfigurator().apply(m_leftRollerConfig);
    m_leftRollerFollow.getConfigurator().apply(m_rollerFollowerConfig);
    m_rightRoller.getConfigurator().apply(m_rightRollerConfig);
    m_rightRollerFollow.getConfigurator().apply(m_rollerFollowerConfig);

    m_kicker.getConfigurator().apply(m_kickerConfig);
    m_kickerFollow.getConfigurator().apply(m_kickerFollowerConfig);

    m_floor.getConfigurator().apply(m_floorConfig);

    Follower leftFollowReq = new Follower(LEFT_ROLLER_ID, MotorAlignmentValue.Aligned);
    m_leftRollerFollow.setControl(leftFollowReq);
    Follower rightFollowReq = new Follower(RIGHT_ROLLER_ID, MotorAlignmentValue.Aligned);
    m_rightRollerFollow.setControl(rightFollowReq);
    Follower floorFollowReq = new Follower(FLOOR_ID, MotorAlignmentValue.Opposed);
    m_kickerFollow.setControl(floorFollowReq);

    m_rollerVelControl = new VelocityVoltage(0);
    m_kickerVelControl = new VelocityVoltage(0);
    m_floorVoltageOut = new VoltageOut(0);
  }

  public void setRollerPID(double new_P, double new_I, double new_D) {
    Slot0Configs slot0Config = new Slot0Configs();
    slot0Config.kP = new_P;
    slot0Config.kI = new_I;
    slot0Config.kD = new_D;
    slot0Config.kV = ROLLER_KV;

    m_leftRoller.getConfigurator().apply(slot0Config);
    m_rightRoller.getConfigurator().apply(slot0Config);
  }

  public void setKickerPID(double new_P, double new_I, double new_D) {
    Slot0Configs slot0Config2 = new Slot0Configs();
    slot0Config2.kP = new_P;
    slot0Config2.kI = new_I;
    slot0Config2.kD = new_D;
    slot0Config2.kV = ROLLER_KV;

    m_kicker.getConfigurator().apply(slot0Config2);
  }

  public double getRollerRPM() {
    return m_leftRoller.getVelocity().getValueAsDouble() * 60;
  }

  public double getRollerTargetRPM() {
    return m_rollerTargetRPM;
  }

  public double getKickerRPM() {
    return m_kicker.getVelocity().getValueAsDouble() * 60;
  }

  public double getKickerTargetRPM() {
    return m_rollerTargetRPM;
  }

  public double getFloorTargetVoltage() {
    return m_floorTargetVoltage;
  }

  public Command stopCommand() {
    return run(() -> {
      m_rollerTargetRPM = 0;
      m_kickerTargetRPM = 0;
    });
  }

  public Command setRollerRPM(double rpm) {
    return startEnd(() -> {
      m_rollerTargetRPM = rpm;
    }, () -> {
      m_rollerTargetRPM = 0;
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
    m_leftRoller.setControl(m_rollerVelControl.withVelocity(m_rollerTargetRPM / 60));
    m_rightRoller.setControl(m_rollerVelControl.withVelocity(m_rollerTargetRPM / 60));

    m_kicker.setControl(m_kickerVelControl.withVelocity(m_kickerTargetRPM / 60));

    m_floor.setControl(m_floorVoltageOut.withOutput(m_floorTargetVoltage));
  }
}
