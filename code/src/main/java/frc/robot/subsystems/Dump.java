package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import static frc.robot.util.Constants.RioConstants.*;
import static frc.robot.util.Constants.DumpConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Dump extends SubsystemBase {
  private TalonFX m_leftRoller;
  private TalonFX m_leftRollerFollow;
  private TalonFX m_rightRoller;
  private TalonFX m_rightRollerFollow;
  private TalonFX m_floor;
  private TalonFX m_floorFollow;

  private TalonFXConfiguration m_leftRollerConfig;
  private TalonFXConfiguration m_rightRollerConfig;
  private TalonFXConfiguration m_floorConfig;

  private CurrentLimitsConfigs m_rollerCurrentConfig;
  private CurrentLimitsConfigs m_floorCurrentConfig;

  private double m_rollerTargetRPM;
  private double m_floorTargetRPM;

  private VelocityVoltage m_rollerVelControl;
  private VelocityVoltage m_floorVelControl;

  public Dump() {
    m_leftRollerConfig = new TalonFXConfiguration();
    m_rightRollerConfig = new TalonFXConfiguration();
    m_floorConfig = new TalonFXConfiguration();

    m_rollerCurrentConfig = new CurrentLimitsConfigs();
    m_floorCurrentConfig = new CurrentLimitsConfigs();

    m_leftRoller = new TalonFX(LEFT_ROLLER_ID, RIO_BUS);
    m_leftRollerFollow = new TalonFX(LEFT_ROLLER_FOLLOWER_ID, RIO_BUS);
    m_rightRoller = new TalonFX(RIGHT_ROLLER_ID, RIO_BUS);
    m_rightRollerFollow = new TalonFX(RIGHT_ROLLER_FOLLOWER_ID, RIO_BUS);
    m_floor = new TalonFX(FLOOR_ID, RIO_BUS);
    m_floorFollow = new TalonFX(FLOOR_FOLLOWER_ID, RIO_BUS);

    m_rollerCurrentConfig.withSupplyCurrentLimit(ROLLER_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(ROLLER_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);
    m_floorCurrentConfig.withSupplyCurrentLimit(FLOOR_CURRENT_LIMIT).withSupplyCurrentLimitEnable(true)
        .withStatorCurrentLimit(FLOOR_CURRENT_LIMIT).withStatorCurrentLimitEnable(true);

    m_leftRollerConfig.Slot0.kP = ROLLER_KP;
    m_leftRollerConfig.Slot0.kI = ROLLER_KI;
    m_leftRollerConfig.Slot0.kD = ROLLER_KD;
    m_leftRollerConfig.Slot0.kV = ROLLER_KV;
    m_leftRollerConfig.CurrentLimits = m_rollerCurrentConfig;

    m_rightRollerConfig = m_leftRollerConfig;

    m_leftRoller.getConfigurator().apply(m_leftRollerConfig);
    m_leftRollerFollow.getConfigurator().apply(m_leftRollerConfig);
    m_rightRoller.getConfigurator().apply(m_rightRollerConfig);
    m_rightRollerFollow.getConfigurator().apply(m_rightRollerConfig);

    m_floorConfig.CurrentLimits = m_floorCurrentConfig;
    m_floor.getConfigurator().apply(m_floorConfig);

    Follower leftFollowReq = new Follower(LEFT_ROLLER_ID, MotorAlignmentValue.Opposed);
    m_leftRollerFollow.setControl(leftFollowReq);
    Follower rightFollowReq = new Follower(RIGHT_ROLLER_ID, MotorAlignmentValue.Opposed);
    m_rightRollerFollow.setControl(rightFollowReq);
    Follower floorFollowReq = new Follower(FLOOR_ID, MotorAlignmentValue.Opposed);
    m_floorFollow.setControl(floorFollowReq);

    m_rollerVelControl = new VelocityVoltage(0);
    m_floorVelControl = new VelocityVoltage(0);
  }

  public void setRollerPID(double new_P, double new_I, double new_D) {
    m_leftRollerConfig.Slot0.kP = new_P;
    m_leftRollerConfig.Slot0.kI = new_I;
    m_leftRollerConfig.Slot0.kD = new_D;

    m_rightRollerConfig = m_leftRollerConfig;
    m_leftRoller.getConfigurator().apply(m_leftRollerConfig);
    m_rightRoller.getConfigurator().apply(m_rightRollerConfig);
  }

  public double getRollerRPM() {
    return m_leftRoller.getVelocity().getValueAsDouble() * 60;
  }

  public double getTargetRPM() {
    return m_rollerTargetRPM;
  }

  public Command setRollerRPM(double rpm) {
    return run(() -> {
      m_rollerTargetRPM = rpm;
    });
  }

  public Command setFloorRPM(double rpm) {
    return run(() -> {
      m_rollerTargetRPM = rpm;
    });
  }

  @Override
  public void periodic() {
    m_leftRoller.setControl(m_rollerVelControl.withVelocity(m_rollerTargetRPM / 60));
    m_rightRoller.setControl(m_rollerVelControl.withVelocity(m_rollerTargetRPM / 60));
    m_floor.setControl(m_floorVelControl.withVelocity(m_floorTargetRPM / 60));

  }
}
